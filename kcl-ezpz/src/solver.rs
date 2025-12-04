use std::sync::Mutex;

use faer::sparse::{Pair, SymbolicSparseColMat};

use crate::{
    Constraint, ConstraintEntry, NonLinearSystemError, Warning, WarningContent,
    constraints::JacobianVar, id::Id,
};

mod newton;

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
    /// How close can the residual be to 0 before we declare the system is solved?
    /// Smaller number means more precise solves.
    pub convergence_tolerance: f64,
    /// Stop iterating if the step size becomes negligible (relative infinity norm).
    pub step_tolerance: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            regularization_enabled: true,
            max_iterations: 35,
            convergence_tolerance: 1e-8,
            step_tolerance: 1e-12,
        }
    }
}

pub struct Layout {
    /// Equivalent to number of rows in the matrix being solved.
    pub total_num_residuals: usize,
    /// One variable per column of the matrix.
    pub num_variables: usize,
    // num_residuals_constraints: usize,
}

impl Layout {
    pub fn new(all_variables: &[Id], constraints: &[&Constraint], _config: Config) -> Self {
        // We'll have different numbers of rows in the system depending on whether
        // or not regularization is enabled.
        let num_residuals_constraints: usize = constraints.iter().map(|c| c.residual_dim()).sum();

        // Build the full system.
        let num_residuals = num_residuals_constraints;
        let num_rows = num_residuals;
        Self {
            total_num_residuals: num_rows,
            num_variables: all_variables.len(),
            // num_residuals_constraints,
        }
    }

    pub fn index_of(&self, var: Id) -> usize {
        var as usize
    }

    pub fn num_rows(&self) -> usize {
        self.total_num_residuals
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

/// The problem to actually solve.
/// Note that the initial values of each variable are required for Tikhonov regularization.
pub(crate) struct Model<'c> {
    layout: Layout,
    jc: Jc,
    constraints: &'c [ConstraintEntry<'c>],
    row0_scratch: Vec<JacobianVar>,
    row1_scratch: Vec<JacobianVar>,
    pub(crate) warnings: Mutex<Vec<Warning>>,
    lambda_i: faer::sparse::SparseColMat<usize, f64>,
}

fn validate_variables(
    constraints: &[ConstraintEntry<'_>],
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
    for constraint in constraints {
        row0.clear();
        row1.clear();
        constraint.constraint.nonzeroes(&mut row0, &mut row1);
        for v in &row0 {
            if !all_variables.contains(v) {
                return Err(NonLinearSystemError::MissingGuess {
                    constraint_id: constraint.id,
                    variable: *v,
                });
            }
        }
        for v in &row1 {
            if !all_variables.contains(v) {
                return Err(NonLinearSystemError::MissingGuess {
                    constraint_id: constraint.id,
                    variable: *v,
                });
            }
        }
    }
    Ok(())
}

impl<'c> Model<'c> {
    pub fn new(
        constraints: &'c [ConstraintEntry<'c>],
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
        let cs: Vec<_> = constraints.iter().map(|c| c.constraint).collect();
        let layout = Layout::new(&all_variables, cs.as_slice(), config);

        // Generate the Jacobian matrix structure.
        // This is the nonzeroes of `J`.
        // It's MxN.
        let mut nonzero_cells_j: Vec<Pair<usize, usize>> =
            Vec::with_capacity(NONZEROES_PER_ROW * layout.total_num_residuals);
        let mut row_num = 0;
        let mut nonzeroes_scratch0 = Vec::with_capacity(NONZEROES_PER_ROW);
        let mut nonzeroes_scratch1 = Vec::with_capacity(NONZEROES_PER_ROW);
        for constraint in constraints {
            nonzeroes_scratch0.clear();
            nonzeroes_scratch1.clear();
            constraint
                .constraint
                .nonzeroes(&mut nonzeroes_scratch0, &mut nonzeroes_scratch1);

            let rows = [&nonzeroes_scratch0, &nonzeroes_scratch1];
            for row in rows.iter().take(constraint.constraint.residual_dim()) {
                let this_row = row_num;
                row_num += 1;
                for var in row.iter() {
                    let col = layout.index_of(*var);
                    nonzero_cells_j.push(Pair { row: this_row, col });
                }
            }
        }

        // Create symbolic structure; this will automatically deduplicate and sort.
        let (sym, _) = SymbolicSparseColMat::try_new_from_indices(
            layout.num_rows(),
            num_cols,
            &nonzero_cells_j,
        )?;

        // Preallocate this so we can use it whenever we run a newton solve.
        // This 'damps' the jacobian matrix, ensuring that as its coefficients get smaller,
        // the solver takes smaller and smaller steps.
        let lambda_i = build_lambda_i(layout.num_variables);

        // All done.
        Ok(Self {
            warnings: Default::default(),
            layout,
            jc: Jc {
                vals: vec![0.0; sym.compute_nnz()], // We have a nonzero count util.
                sym,
            },
            constraints,
            row0_scratch: Vec::with_capacity(NONZEROES_PER_ROW),
            row1_scratch: Vec::with_capacity(NONZEROES_PER_ROW),
            lambda_i,
        })
    }
}

fn build_lambda_i(num_variables: usize) -> faer::sparse::SparseColMat<usize, f64> {
    faer::sparse::SparseColMat::<usize, f64>::try_new_from_triplets(
        num_variables,
        num_variables,
        &(0..num_variables)
            .map(|i| faer::sparse::Triplet::new(i, i, REGULARIZATION_LAMBDA))
            .collect::<Vec<_>>(),
    )
    .unwrap()
}

/// Connect the model to newton_faer's solver.
impl Model<'_> {
    /// Compute the residual F, figuring out how close the problem is to being solved.
    /// `out` is the global residual vector.
    fn residual(&self, current_assignments: &[f64], out: &mut [f64]) {
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
            constraint.constraint.residual(
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
                .take(constraint.constraint.residual_dim())
            {
                let this_row = row_num;
                row_num += 1;
                out[this_row] = **row;
            }
        }
    }

    /// Update the values of a cached sparse Jacobian.
    fn refresh_jacobian(&mut self, current_assignments: &[f64]) {
        // To enable per-variable partial derivative accumulation (i.e. local to global
        // Jacobian assembly), we need to zero out the Jacobian values first.
        self.jc.vals.fill(0.0);

        // Allocate some scratch space for the Jacobian calculations, so that we can
        // do one allocation here and then won't need any allocations per-row or per-column.
        // TODO: Should this be stored in the model?

        // Build values by iterating through constraints in the same order as their construction.
        let mut row_num = 0;
        #[cfg(feature = "dbg-jac")]
        let mut dbg_matrix: Vec<Vec<f64>> = vec![];
        for (i, constraint) in self.constraints.iter().enumerate() {
            let mut degenerate = false;
            self.row0_scratch.clear();
            self.row1_scratch.clear();
            constraint.constraint.jacobian_rows(
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
                .take(constraint.constraint.residual_dim())
            {
                let this_row = row_num;
                row_num += 1;
                #[cfg(feature = "dbg-jac")]
                dbg_matrix.push(vec![0.0; self.layout.num_variables]);
                for jacobian_var in row {
                    #[cfg(feature = "dbg-jac")]
                    {
                        dbg_matrix.last_mut().unwrap()[jacobian_var.id as usize] +=
                            jacobian_var.partial_derivative;
                    }
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
        #[cfg(feature = "dbg-jac")]
        assert_eq!(dbg_matrix.len(), self.layout.num_rows());
        #[cfg(feature = "dbg-jac")]
        {
            for (i, dbg_row) in dbg_matrix.into_iter().enumerate() {
                let inner: Vec<_> = dbg_row
                    .into_iter()
                    .map(|d| {
                        if d.is_sign_positive() {
                            format!(" {d:.2}")
                        } else {
                            format!("{d:.2}")
                        }
                    })
                    .collect();
                eprintln!("Row {i}: [{}]", inner.join(" "));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datatypes::DatumPoint;

    #[test]
    fn reports_missing_guess_for_second_row_ids() {
        // PointsCoincident puts X ids in row0 and Y ids in row1; omit the Y ids to hit row1 check.
        let constraint =
            Constraint::PointsCoincident(DatumPoint::new_xy(0, 1), DatumPoint::new_xy(2, 3));
        let entry = ConstraintEntry {
            constraint: &constraint,
            id: 42,
            priority: 0,
        };

        let all_variables = vec![0, 2]; // Only X components, missing Y components.
        let initial_values = vec![0.0, 0.0];

        let err = match Model::new(&[entry], all_variables, initial_values, Config::default()) {
            Ok(_) => panic!("expected missing guess error"),
            Err(e) => e,
        };

        match err {
            NonLinearSystemError::MissingGuess {
                constraint_id,
                variable,
            } => {
                assert_eq!(constraint_id, 42);
                assert_eq!(variable, 1); // First missing Y id encountered from row1 branch.
            }
            other => panic!("unexpected error: {other:?}"),
        }
    }
}
