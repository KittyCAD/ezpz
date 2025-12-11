use faer::{
    ColRef,
    prelude::Solve,
    sparse::{SparseColMatRef, linalg::solvers::Lu},
};

use crate::{Config, FreedomAnalysis, NonLinearSystemError};

use super::Model;

#[derive(Debug)]
pub struct SuccessfulSolve {
    pub iterations: usize,
}

impl Model<'_> {
    #[inline(never)]
    pub fn solve_gauss_newton(
        &mut self,
        current_values: &mut [f64],
        config: Config,
    ) -> Result<SuccessfulSolve, NonLinearSystemError> {
        let m = self.layout.total_num_residuals;

        let mut global_residual = vec![0.0; m];

        for this_iteration in 0..config.max_iterations {
            // Assemble global residual and Jacobian
            // Re-evaluate the global residual.
            self.residual(current_values, &mut global_residual);
            // Re-evaluate the global jacobian, write it into self.jc
            self.refresh_jacobian(current_values);

            // Convergence check: if the residual is within our tolerance,
            // then the system is totally solved and we can return.
            let largest_absolute_elem = global_residual
                .iter()
                .map(|x| x.abs())
                .reduce(libm::fmax)
                .ok_or(NonLinearSystemError::EmptySystemNotAllowed)?;
            if largest_absolute_elem <= config.convergence_tolerance {
                return Ok(SuccessfulSolve {
                    iterations: this_iteration,
                });
            }

            /* NOTE(dr): We solve the following linear system to get the damped Gauss-Newton step d
               (JᵀJ + λI) d = -Jᵀr
               This involves creating a matrix A and rhs b where
               A = JᵀJ + λI
               b = -Jᵀr
            */

            let j = SparseColMatRef::new(self.jc.sym.as_ref(), &self.jc.vals);
            // TODO: Is there any way to transpose `j` and keep it in column-major?
            // Converting from row- to column-major might not be necessary.
            let jtj = j.transpose().to_col_major()? * j;
            let a = jtj + &self.lambda_i;
            let b = j.transpose() * -ColRef::from_slice(&global_residual);

            // Solve linear system
            let factored = Lu::try_new_with_symbolic(self.lu_symbolic.clone(), a.as_ref())?;
            let d = factored.solve(&b);
            assert_eq!(
                d.nrows(),
                current_values.len(),
                "the `d` column must be the same size as the number of variables."
            );
            let current_inf_norm = current_values.iter().map(|v| v.abs()).fold(0.0, libm::fmax);
            let step_inf_norm = d.iter().map(|d| d.abs()).reduce(libm::fmax).unwrap_or(0.0);
            current_values
                .iter_mut()
                .zip(d.iter())
                .for_each(|(curr_val, d)| {
                    *curr_val += d;
                });
            let step_threshold = config.step_tolerance * (current_inf_norm + config.step_tolerance);

            // Convergence check: if `d` is small enough,
            // then the system is at a local minimum. It might be inconsistent, and therefore
            // its residual will never get close to zero, but this is still a good least-squares solution,
            // so we can return.
            if step_inf_norm <= step_threshold {
                return Ok(SuccessfulSolve {
                    iterations: this_iteration,
                });
            }
        }
        Err(NonLinearSystemError::DidNotConverge)
    }

    pub fn freedom_analysis(&self) -> Result<FreedomAnalysis, NonLinearSystemError> {
        // First step is to compute the SVD.
        // Faer doesn't have a sparse SVD algorithm, so let's convert it to a dense matrix.
        // This step is SLOW.
        // Faer maintainer said she has a sparse SVD algorithm she hasn't published yet,
        // so hopefully she will publish it soon and this slow step won't be necessary.
        let j_sparse = SparseColMatRef::new(self.jc.sym.as_ref(), &self.jc.vals);
        let j_dense = j_sparse.to_dense();
        debug_assert_eq!(
            self.layout.num_variables,
            j_dense.ncols(),
            "Jacobian was malformed, Adam messed something up here."
        );
        let svd = j_dense.svd().map_err(NonLinearSystemError::FaerSvd)?;
        let sigma_diags = svd.S();

        // These are the 'singular values'.
        let sigma_col = sigma_diags.column_vector();

        // The system is underconstrained if there's too many singular values
        // close to 0. How close to 0? The tolerance should be derived from
        // the largest singular value.
        let largest_singular_value = sigma_col
            .iter()
            .copied()
            .reduce(libm::fmax)
            .ok_or(NonLinearSystemError::EmptySystemNotAllowed)?;
        let tolerance = 1e-8 * largest_singular_value;

        let rank = sigma_col.iter().filter(|&&s| s > tolerance).count();
        let degrees_of_freedom = self.layout.num_variables - rank;
        let is_underconstrained = degrees_of_freedom > 0;

        Ok(FreedomAnalysis {
            is_underconstrained,
        })
    }
}
