use std::sync::Mutex;

use faer::sparse::{Pair, SymbolicSparseColMat};
use newton_faer::{JacobianCache, NonlinearSystem, RowMap};

use crate::{
    Constraint, NonLinearSystemError, Warning, WarningContent, constraints::JacobianVar, id::Id,
};

// Roughly. Most constraints will only involve roughly 4 variables.
// May as well round up to the nearest power of 2.
const NONZEROES_PER_ROW: usize = 8;

// Tikhonov regularization configuration. Note that some texts use lambda^2 as their
// scaling parameter, but it's a magic constant we have to tune either way so who cares.
// Ref: https://people.csail.mit.edu/jsolomon/share/book/numerical_book.pdf, 4.1.3
const REGULARIZATION_LAMBDA: f64 = 1e-9;

#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// Use Tikhonov regularization to solve underdetermined systems.
    pub regularization_enabled: bool,
    /// How many iteration rounds before the solver gives up?
    pub max_iterations: usize,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            regularization_enabled: true,
            max_iterations: 35,
        }
    }
}

pub struct Layout {
    /// Equivalent to number of rows in the matrix being solved.
    pub total_num_residuals: usize,
    /// One variable per column of the matrix.
    pub num_variables: usize,
    num_residuals_constraints: usize,
}

impl Layout {
    pub fn new(all_variables: &[Id], constraints: &[Constraint], config: Config) -> Self {
        // We'll have different numbers of rows in the system depending on whether
        // or not regularization is enabled.
        let num_residuals_constraints: usize = constraints.iter().map(|c| c.residual_dim()).sum();
        let num_residuals_regularization = if config.regularization_enabled {
            all_variables.len()
        } else {
            0
        };

        // Build the full system.
        let num_residuals = num_residuals_constraints + num_residuals_regularization;
        let num_rows = num_residuals;
        Self {
            total_num_residuals: num_rows,
            num_variables: all_variables.len(),
            num_residuals_constraints,
        }
    }
    pub fn index_of(&self, var: <Layout as RowMap>::Var) -> usize {
        var as usize
    }
    pub fn num_rows(&self) -> usize {
        self.total_num_residuals
    }
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
        self.num_variables
    }

    fn n_residuals(&self) -> usize {
        self.num_rows()
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
/// Note that the initial values of each variable are required for Tikhonov regularization.
pub(crate) struct Model<'c> {
    layout: Layout,
    jc: Jc,
    constraints: &'c [Constraint],
    row0_scratch: Vec<JacobianVar>,
    row1_scratch: Vec<JacobianVar>,
    initial_values: Vec<f64>,
    config: Config,
    pub(crate) warnings: Mutex<Vec<Warning>>,
}

fn validate_variables(
    constraints: &[Constraint],
    all_variables: &[Id],
    initial_values: &[f64],
) -> Result<(), NonLinearSystemError> {
    if all_variables.len() != initial_values.len() {
        return Err(NonLinearSystemError::WrongNumberGuesses {
            labels: all_variables.len(),
            guesses: initial_values.len(),
        });
    }
    let mut row0 = Vec::with_capacity(NONZEROES_PER_ROW);
    let mut row1 = Vec::with_capacity(NONZEROES_PER_ROW);
    for (c, constraint) in constraints.iter().enumerate() {
        row0.clear();
        row1.clear();
        constraint.nonzeroes(&mut row0, &mut row1);
        for v in &row0 {
            if !all_variables.contains(v) {
                return Err(NonLinearSystemError::MissingGuess { c, v: *v });
            }
        }
        for v in &row1 {
            if !all_variables.contains(v) {
                return Err(NonLinearSystemError::MissingGuess { c, v: *v });
            }
        }
    }
    Ok(())
}

impl<'c> Model<'c> {
    pub fn new(
        constraints: &'c [Constraint],
        all_variables: Vec<Id>,
        initial_values: Vec<f64>,
        config: Config,
    ) -> Result<Self, NonLinearSystemError> {
        validate_variables(constraints, &all_variables, &initial_values)?;
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

        let num_cols = all_variables.len();
        let layout = Layout::new(&all_variables, constraints, config);

        // Generate the Jacobian matrix structure.
        let mut nonzero_cells: Vec<Pair<usize, usize>> =
            Vec::with_capacity(NONZEROES_PER_ROW * layout.total_num_residuals);
        let mut row_num = 0;
        let mut nonzeroes_scratch0 = Vec::with_capacity(NONZEROES_PER_ROW);
        let mut nonzeroes_scratch1 = Vec::with_capacity(NONZEROES_PER_ROW);
        for constraint in constraints {
            nonzeroes_scratch0.clear();
            nonzeroes_scratch1.clear();
            constraint.nonzeroes(&mut nonzeroes_scratch0, &mut nonzeroes_scratch1);

            let rows = [&nonzeroes_scratch0, &nonzeroes_scratch1];
            for row in rows.iter().take(constraint.residual_dim()) {
                let this_row = row_num;
                row_num += 1;
                for var in row.iter() {
                    let col = layout.index_of(*var);
                    nonzero_cells.push(Pair { row: this_row, col });
                }
            }
        }

        // Stack our regularization rows below the constraint rows.
        if config.regularization_enabled {
            for col in 0..num_cols {
                let reg_row = layout.num_residuals_constraints + col;
                nonzero_cells.push(Pair { row: reg_row, col });
            }
        }

        // Create symbolic structure; this will automatically deduplicate and sort.
        let (sym, _) = SymbolicSparseColMat::try_new_from_indices(
            layout.num_rows(),
            num_cols,
            &nonzero_cells,
        )?;

        // All done.
        Ok(Self {
            warnings: Default::default(),
            config,
            layout,
            jc: Jc {
                vals: vec![0.0; sym.compute_nnz()], // We have a nonzero count util.
                sym,
            },
            constraints,
            row0_scratch: Vec::with_capacity(NONZEROES_PER_ROW),
            row1_scratch: Vec::with_capacity(NONZEROES_PER_ROW),
            initial_values,
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
        let mut residuals0;
        let mut residuals1;

        // Compute constraint residuals.
        for (i, constraint) in self.constraints.iter().enumerate() {
            let mut degenerate = false;
            residuals0 = 0.0;
            residuals1 = 0.0;
            constraint.residual(
                &self.layout,
                current_assignments,
                &mut residuals0,
                &mut residuals1,
                &mut degenerate,
            );
            if degenerate {
                let mut warnings = self.warnings.lock().unwrap();
                warnings.push(Warning {
                    about_constraint: Some(i),
                    content: WarningContent::Degenerate,
                })
            }
            for row in [&residuals0, &residuals1]
                .iter()
                .take(constraint.residual_dim())
            {
                let this_row = row_num;
                row_num += 1;
                out[this_row] = **row;
            }
        }

        // Add Tikhonov regularization residuals: lambda * (x - x0).
        if self.config.regularization_enabled {
            for (&val, &val_init) in current_assignments.iter().zip(self.initial_values.iter()) {
                out[row_num] = REGULARIZATION_LAMBDA * (val - val_init);
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
        // TODO: Should this be stored in the model?

        // Build values by iterating through constraints in the same order as their construction.
        let mut row_num = 0;
        for (i, constraint) in self.constraints.iter().enumerate() {
            let mut degenerate = false;
            self.row0_scratch.clear();
            self.row1_scratch.clear();
            constraint.jacobian_rows(
                &self.layout,
                current_assignments,
                &mut self.row0_scratch,
                &mut self.row1_scratch,
                &mut degenerate,
            );
            if degenerate {
                let mut warnings = self.warnings.lock().unwrap();
                warnings.push(Warning {
                    about_constraint: Some(i),
                    content: WarningContent::Degenerate,
                })
            }

            // For each variable in this constraint's set of partial derivatives (Jacobian slice).
            for row in [&self.row0_scratch, &self.row1_scratch]
                .into_iter()
                .take(constraint.residual_dim())
            {
                let this_row = row_num;
                row_num += 1;
                for jacobian_var in row {
                    let col = self.layout.index_of(jacobian_var.id);

                    // Find where this (row_num, col) entry should go in the sparse structure.
                    let mut col_range = self.jc.sym.col_range(col);
                    let row_indices = self.jc.sym.row_idx();

                    // Search for our row within this column's entries.
                    let idx = col_range.find(|idx| row_indices[*idx] == this_row).unwrap();
                    // Found the right position; accumulate the partials.
                    self.jc.vals[idx] += jacobian_var.partial_derivative;
                }
            }
        }

        // Add regularization values.
        if self.config.regularization_enabled {
            let num_constraint_residuals: usize =
                self.constraints.iter().map(|c| c.residual_dim()).sum();

            for col in 0..self.layout.n_variables() {
                let reg_row = num_constraint_residuals + col;

                // Find where this (reg_row, col) entry should go in the sparse structure.
                let mut col_range = self.jc.sym.col_range(col);
                let row_indices = self.jc.sym.row_idx();

                // Search for our regularization row within this column's entries and set the regularization Jacobian
                // entry to lambda. (Because derivative of lambda*(x-x0) w.r.t. x = lambda.)
                let idx = col_range.find(|idx| row_indices[*idx] == reg_row).unwrap();

                self.jc.vals[idx] = REGULARIZATION_LAMBDA;
            }
        }
    }
}
