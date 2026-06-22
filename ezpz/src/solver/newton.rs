use faer::{
    ColRef,
    prelude::Solve,
    sparse::{SparseColMatRef, linalg::solvers::Lu},
};

use crate::{Config, NonLinearSystemError};

use super::Model;

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
    /// Solve via Levenberg-Marquardt algorithm (Gauss-Newton with adaptive damping)
    #[inline(never)]
    pub(crate) fn solve_levenberg_marquardt(
        &mut self,
        current_values: &mut [f64],
        config: Config,
    ) -> Result<SuccessfulSolve, NonLinearSystemError> {
        let m = self.layout.total_num_residuals;
        let n = current_values.len();

        let mut global_residual = vec![0.0; m];
        let mut next_residual = vec![0.0; m];

        // NOTE(dr): We use a standard Levenberg-Marquardt adaptive damping scheme here where the
        // damping parameter λ is scaled down on accepted steps and up on rejected ones. A step is
        // rejected if it doesn't reduce the squared norm of the residual, which biases toward
        // gradient descent near singular configurations where Gauss-Newton tends to overshoot.
        let mut lambda = config.initial_lambda;
        let mut residual_sq = self.eval(current_values, &mut global_residual);

        for this_iteration in 0..config.max_iterations {
            // Convergence check: if the residual is within our tolerance,
            // then the system is totally solved and we can return.
            let largest_absolute_elem = global_residual
                .iter()
                .map(|x| x.abs())
                .reduce(libm::fmax)
                .ok_or(NonLinearSystemError::EmptySystemNotAllowed)?;
            if largest_absolute_elem <= config.residual_tolerance {
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

            let j =
                SparseColMatRef::new(self.jacobian_cache.sym.as_ref(), &self.jacobian_cache.vals);
            // TODO: Is there any way to transpose `j` and keep it in column-major?
            // Converting from row- to column-major might not be necessary.
            let jtj = j.transpose().to_col_major()? * j;

            // Update λI with current damping value
            self.lambda_i.val_mut().fill(lambda);

            // Solve linear system
            let a = jtj + &self.lambda_i;
            let b = j.transpose() * -ColRef::from_slice(&global_residual);

            // Solve the linear system for the step `d`.
            let factored = Lu::try_new_with_symbolic(self.lu_symbolic.clone(), a.as_ref())?;
            let d = factored.solve(&b);
            assert_eq!(
                d.nrows(),
                n,
                "the `d` column must be the same size as the number of variables."
            );
            let step_inf_norm = d.iter().map(|x| x.abs()).reduce(libm::fmax).unwrap_or(0.0);

            // Take the tentative step and evaluate the residual at the new position
            current_values
                .iter_mut()
                .zip(d.iter())
                .for_each(|(curr_val, step)| *curr_val += step);
            self.residual(current_values, &mut next_residual);
            let next_residual_sq: f64 = next_residual.iter().map(|x| x * x).sum();

            if next_residual_sq < residual_sq {
                // Step reduced the residual: accept it and decrease λ.
                std::mem::swap(&mut global_residual, &mut next_residual);
                self.refresh_jacobian(current_values);
                residual_sq = next_residual_sq;
                lambda *= LM_LAMBDA_DECR;
            } else {
                // Step didn't reduce the residual: revert it and increase λ.
                current_values
                    .iter_mut()
                    .zip(d.iter())
                    .for_each(|(curr_val, step)| *curr_val -= step);
                lambda *= LM_LAMBDA_INCR;
            }

            // Also need to check step size to identify convergence in the overconstrained case
            if step_inf_norm <= config.step_tolerance {
                return Ok(SuccessfulSolve {
                    iterations: this_iteration,
                    converged: true,
                });
            }
        }
        Ok(SuccessfulSolve {
            iterations: config.max_iterations,
            converged: false,
        })
    }

    /// Solve via damped Gauss-Newton algorithm (retained for reference)
    #[allow(dead_code)]
    #[inline(never)]
    pub(crate) fn solve_gauss_newton(
        &mut self,
        current_values: &mut [f64],
        config: Config,
    ) -> Result<SuccessfulSolve, NonLinearSystemError> {
        let m = self.layout.total_num_residuals;
        let n = current_values.len();

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
            if largest_absolute_elem <= config.residual_tolerance {
                return Ok(SuccessfulSolve {
                    iterations: this_iteration,
                    converged: true,
                });
            }

            /* NOTE(dr): We solve the following linear system to get the damped Gauss-Newton step d
               (JᵀJ + λI) d = -Jᵀr
               This involves creating a matrix A and rhs b where
               A = JᵀJ + λI
               b = -Jᵀr
            */

            let j =
                SparseColMatRef::new(self.jacobian_cache.sym.as_ref(), &self.jacobian_cache.vals);
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
                n,
                "the `d` column must be the same size as the number of variables."
            );
            let step_inf_norm = d.iter().map(|d| d.abs()).reduce(libm::fmax).unwrap_or(0.0);
            current_values
                .iter_mut()
                .zip(d.iter())
                .for_each(|(curr_val, d)| {
                    *curr_val += d;
                });

            // Convergence check: if `d` is small enough,
            // then the system is at a local minimum. It might be inconsistent, and therefore
            // its residual will never get close to zero, but this is still a good least-squares solution,
            // so we can return.
            if step_inf_norm <= config.step_tolerance {
                return Ok(SuccessfulSolve {
                    iterations: this_iteration,
                    converged: true,
                });
            }
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
