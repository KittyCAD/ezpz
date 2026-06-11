use faer::{
    ColRef,
    prelude::Solve,
    sparse::{SparseColMatRef, linalg::solvers::Lu},
};

use crate::{Config, NonLinearSystemError};

use super::{Model, build_lambda_i};

// Levenberg-Marquardt adaptive damping params
const LM_LAMBDA_INCR: f64 = 10.0;
const LM_LAMBDA_DECR: f64 = 0.1;

#[derive(Debug)]
pub struct SuccessfulSolve {
    /// How many iterations did the solver run for?
    pub iterations: usize,
    /// Did it ultimately converge, or not?
    pub converged: bool,
}

impl Model<'_> {
    #[inline(never)]
    pub(crate) fn solve_gauss_newton(
        &mut self,
        current_values: &mut [f64],
        config: Config,
    ) -> Result<SuccessfulSolve, NonLinearSystemError> {
        let m = self.layout.total_num_residuals;
        let n = current_values.len();

        let mut global_residual = vec![0.0; m];
        let mut step = vec![0.0; n];

        // NOTE(dr): We use a standard Levenberg-Marquardt adaptive damping scheme here where the
        // damping parameter λ is scaled down on accepted steps and up on rejected ones. A step is
        // rejected if it doesn't reduce the squared norm of the residual, which biases toward
        // gradient descent (more stable) near singular configurations where Gauss-Newton
        // overshoots.
        let mut lambda = config.initial_lambda;

        let mut residual_sq = self.eval(current_values, &mut global_residual);
        let mut converged = false;

        for this_iteration in 0..config.max_iterations {
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
                    converged: true,
                });
            } else if converged {
                // The step shrank below tolerance without zeroing the residual indicating a
                // least-squares solution
                return Ok(SuccessfulSolve {
                    iterations: this_iteration,
                    converged: true,
                });
            }

            /*
                NOTE(dr): We solve the following linear system to get the damped Gauss-Newton step d

                    (JᵀJ + λI) d = -Jᵀr

                This involves creating a matrix A and rhs b where

                    A = JᵀJ + λI
                    b = -Jᵀr
            */

            let step_inf_norm = {
                let j = SparseColMatRef::new(
                    self.jacobian_cache.sym.as_ref(),
                    &self.jacobian_cache.vals,
                );
                // TODO: Is there any way to transpose `j` and keep it in column-major?
                // Converting from row- to column-major might not be necessary.
                let jtj = j.transpose().to_col_major()? * j;
                let lambda_i = build_lambda_i(n, lambda);
                let a = jtj + &lambda_i;
                let b = j.transpose() * -ColRef::from_slice(&global_residual);

                // Solve linear system
                let factored = Lu::try_new_with_symbolic(self.lu_symbolic.clone(), a.as_ref())?;
                let d = factored.solve(&b);
                assert_eq!(
                    d.nrows(),
                    n,
                    "the `d` column must be the same size as the number of variables."
                );
                step.iter_mut().zip(d.iter()).for_each(|(s, d)| *s = *d);
                d.iter().map(|d| d.abs()).reduce(libm::fmax).unwrap_or(0.0)
            };

            // Take the tentative step and re-evaluate at the new position.
            current_values
                .iter_mut()
                .zip(step.iter())
                .for_each(|(curr_val, d)| *curr_val += d);
            let residual_sq_new = self.eval(current_values, &mut global_residual);

            if residual_sq_new < residual_sq {
                // Step reduced the residual: accept it and decrease λ.
                lambda *= LM_LAMBDA_DECR;
                residual_sq = residual_sq_new;
            } else {
                // Step didn't reduce the residual: revert it, restore the residual and Jacobian to
                // the old position, and increase λ.
                current_values
                    .iter_mut()
                    .zip(step.iter())
                    .for_each(|(curr_val, d)| *curr_val -= d);
                residual_sq = self.eval(current_values, &mut global_residual);
                lambda *= LM_LAMBDA_INCR;
            }

            // Flag as converged if the step was negligibly small (even if it wasn't taken)
            converged = step_inf_norm <= config.step_tolerance;
        }
        Ok(SuccessfulSolve {
            iterations: config.max_iterations,
            converged: false,
        })
    }

    /// Re-evaluate the global residual and Jacobian at `current_values`, returning the
    /// squared norm of the residual.
    fn eval(&mut self, current_values: &[f64], global_residual: &mut [f64]) -> f64 {
        self.residual(current_values, global_residual);
        self.refresh_jacobian(current_values);
        global_residual.iter().map(|x| x * x).sum()
    }
}
