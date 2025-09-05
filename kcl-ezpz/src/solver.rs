use faer::sparse::{Pair, SymbolicSparseColMat};
use newton_faer::{JacobianCache, NonlinearSystem, RowMap};

use crate::{Constraint, NonLinearSystemError, id::Id};

// Roughly. Most constraints will only involve roughly 4 variables.
// May as well round up to the nearest power of 2.
const NONZEROES_PER_ROW: usize = 8;

pub struct Layout {
    /// Equivalent to number of rows in the matrix being solved.
    total_num_residuals: usize,
    /// One variable per column of the matrix.
    all_variables: Vec<Id>,
}

impl RowMap for Layout {
    type Var = Id;

    // `faer_newton` stores variables in a vec, refers to them only by their offset.
    // So this function lets you look up the index of a particular variable in that vec.
    // `bus` is the row index and `var` is the variable being looked up,
    // and you get the index (column) of the variable in that row.
    fn row(&self, _row_number: usize, var: Self::Var) -> Option<usize> {
        // In our system, variables are the same in every row.
        Some(var as usize)
    }

    fn n_variables(&self) -> usize {
        self.all_variables.len()
    }

    fn n_residuals(&self) -> usize {
        self.total_num_residuals
    }
}

impl Layout {
    pub fn index_of(&self, var: <Layout as RowMap>::Var) -> usize {
        var as usize
    }
}

/// A Jacobian cache.
/// Stores the Jacobian so we don't constantly reallocate it.
/// Required by newton_faer.
struct Jc {
    /// The symbolic structure of the matrix (i.e. which cells are non-zero).
    /// This way the matrix's structure is only allocated once, and reused
    /// between different Jacobian calculations.
    sym: SymbolicSparseColMat<usize>,
    /// The values which belong in that symbolic matrix, sorted in column-major order.
    /// Must be column-major because faer expects that.
    vals: Vec<f64>,
}

impl JacobianCache<f64> for Jc {
    /// Self owns the symbolic pattern, so it can
    /// give out a reference to it.
    fn symbolic(&self) -> &SymbolicSparseColMat<usize> {
        &self.sym
    }

    /// Lets newton-faer read the current values.
    fn values(&self) -> &[f64] {
        &self.vals
    }

    /// Lets newton-faer overwrite the previous values.
    fn values_mut(&mut self) -> &mut [f64] {
        &mut self.vals
    }
}

/// The problem to actually solve.
pub struct Model<'c> {
    layout: Layout,
    jc: Jc,
    constraints: &'c [Constraint],
}

impl<'c> Model<'c> {
    pub fn new(
        constraints: &'c [Constraint],
        all_variables: Vec<Id>,
    ) -> Result<Self, NonLinearSystemError> {
        /*
        Firstly, find the size of the relevant matrices.
        Each constraint yields 1 or more residual function f.
        Each residual function f is summed to form the overall residual F.
        Each residual function yields a derivative f'.
        The overall Jacobian is a matrix where
            each row is one of the residual functions.
            each column is a variable
            each cell represents the partial derivative of that column's variable,
            in that row's equation.
        Thus the Jacobian has
            num_rows = number of residual functions,
                       which is >= number of constraints
                       (as each constraint yields 1 or more residual functions)
            num_cols = total number of variables
                       which is = total number of "involved primitive IDs"
        */
        let num_residuals: usize = constraints.iter().map(|c| c.residual_dim()).sum();
        let num_cols = all_variables.len();
        let num_rows = num_residuals;
        let layout = Layout {
            total_num_residuals: num_rows,
            all_variables,
        };

        // Generate the Jacobian matrix structure.
        let mut nonzero_cells: Vec<Pair<usize, usize>> =
            Vec::with_capacity(NONZEROES_PER_ROW * num_rows);
        let mut row_num = 0;
        let mut nonzeroes_scratch = Vec::with_capacity(NONZEROES_PER_ROW);
        for constraint in constraints {
            nonzeroes_scratch.clear();
            constraint.nonzeroes(&mut nonzeroes_scratch);
            debug_assert_eq!(
                constraint.residual_dim(),
                1,
                "Constraint {} has {} rows but we only passed scratch room for 1, pls update this code",
                constraint.constraint_kind(),
                constraint.residual_dim(),
            );

            // Right now, all constraints have a single row,
            // but we know that soon we'll add constraints with more.
            let rows = [&nonzeroes_scratch];
            for row in rows {
                let this_row = row_num;
                row_num += 1;
                for var in row {
                    let col = layout.index_of(*var);
                    nonzero_cells.push(Pair { row: this_row, col });
                }
            }
        }
        // Create symbolic structure; this will automatically deduplicate and sort.
        let (sym, _) =
            SymbolicSparseColMat::try_new_from_indices(num_rows, num_cols, &nonzero_cells).unwrap();

        // All done.
        Ok(Self {
            layout,
            jc: Jc {
                vals: vec![0.0; sym.compute_nnz()], // We have a nonzero count util.
                sym,
            },
            constraints,
        })
    }
}

/// Connect the model to newton_faer's solver.
impl NonlinearSystem for Model<'_> {
    /// What number type we're using.
    type Real = f64;
    type Layout = Layout;

    fn layout(&self) -> &Self::Layout {
        &self.layout
    }

    /// Let the solver read the Jacobian cache.
    fn jacobian(&self) -> &dyn JacobianCache<Self::Real> {
        &self.jc
    }

    /// Let the solver write into the Jacobian cache.
    fn jacobian_mut(&mut self) -> &mut dyn JacobianCache<Self::Real> {
        &mut self.jc
    }

    /// Compute the residual F, figuring out how close the problem is to being solved.
    fn residual(&self, current_assignments: &[Self::Real], out: &mut [Self::Real]) {
        // Each row of `out` corresponds to one row of the matrix, i.e. one equation.
        // Each item of `current_assignments` corresponds to one column of the matrix, i.e. one variable.
        let mut row_num = 0;
        let mut residuals = Vec::new();
        for constraint in self.constraints {
            residuals.clear();
            constraint.residual(&self.layout, current_assignments, &mut residuals);
            debug_assert_eq!(
                residuals.len(),
                constraint.residual_dim(),
                "Constraint {} should have {} residuals but actually had {}",
                constraint.constraint_kind(),
                constraint.residual_dim(),
                residuals.len(),
            );
            for residual in residuals.iter().copied() {
                out[row_num] = residual;
                row_num += 1;
            }
        }
    }

    /// Update the values of a cached sparse Jacobian.
    fn refresh_jacobian(&mut self, current_assignments: &[Self::Real]) {
        // To enable per-variable partial derivative accumulation (i.e. local to global
        // Jacobian assembly), we need to zero out the Jacobian values first.
        self.jc.vals.fill(0.0);

        // Allocate some scratch space for the Jacobian calculations, so that we can
        // do one allocation here and then won't need any allocations per-row or per-column.
        let mut row0_scratch = Vec::with_capacity(NONZEROES_PER_ROW);

        // Build values by iterating through constraints in the same order as their construction.
        for (row_num, constraint) in self.constraints.iter().enumerate() {
            // At present, we only have constraints with a single row but if we expand
            // we could iterate across each constraint row here.
            row0_scratch.clear();
            constraint.jacobian_rows(&self.layout, current_assignments, &mut row0_scratch);

            debug_assert_eq!(
                1,
                constraint.residual_dim(),
                "Constraint {} should have 1 Jacobian rows but actually had {}, update the code to pass more scratch rows.",
                constraint.constraint_kind(),
                constraint.residual_dim(),
            );

            // For each variable in this constraint's set of partial derivatives (Jacobian slice).
            for jacobian_var in row0_scratch.iter() {
                let col = self.layout.index_of(jacobian_var.id);

                // Find where this (row_num, col) entry should go in the sparse structure.
                let col_range = self.jc.sym.col_range(col);
                let row_indices = self.jc.sym.row_idx();

                // Search for our row within this column's entries.
                for idx in col_range {
                    if row_indices[idx] == row_num {
                        // Found the right position; accumulate the partials.
                        self.jc.vals[idx] += jacobian_var.partial_derivative;
                        break;
                    }
                }
            }
        }
    }
}
