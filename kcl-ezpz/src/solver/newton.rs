use faer::{
    ColRef, get_global_parallelism,
    matrix_free::eigen::partial_eigen_scratch,
    prelude::Solve,
    sparse::{SparseColMatRef, linalg::solvers::Lu},
};

use crate::NonLinearSystemError;

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
    ) -> Result<SuccessfulSolve, NonLinearSystemError> {
        let m = self.layout.total_num_residuals;

        let mut global_residual = vec![0.0; m];

        for this_iteration in 0..self.config.max_iterations {
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
                .reduce(f64::max)
                .ok_or(NonLinearSystemError::EmptySystemNotAllowed)?;
            if largest_absolute_elem <= self.config.convergence_tolerance {
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
            let current_inf_norm = current_values.iter().map(|v| v.abs()).fold(0.0, f64::max);
            let step_inf_norm = d.iter().map(|d| d.abs()).reduce(f64::max).unwrap_or(0.0);
            current_values
                .iter_mut()
                .zip(d.iter())
                .for_each(|(curr_val, d)| {
                    *curr_val += d;
                });
            let step_threshold =
                self.config.step_tolerance * (current_inf_norm + self.config.step_tolerance);

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

    pub fn is_underconstrained(&self) -> Result<bool, NonLinearSystemError> {
        // First step is to compute the SVD.
        // If the problem is very very small, just use dense SVD.
        let n = self.layout.num_variables;
        let m = self.layout.num_rows();
        let k = usize::min(n, m);
        eprintln!("{n}, {m}, {k}");

        let sigma_col = if k <= 2 {
            eprintln!("Dense SVD");
            // Small, use sparse SVD
            let j_sparse = SparseColMatRef::new(self.jc.sym.as_ref(), &self.jc.vals);
            let j_dense = j_sparse.to_dense();
            debug_assert_eq!(
                self.layout.num_variables,
                j_dense.ncols(),
                "Jacobian was malformed, Adam messed something up here."
            );
            let svd = j_dense.svd().map_err(NonLinearSystemError::FaerSvd)?;
            let v: Vec<_> = svd.S().column_vector().iter().copied().collect();
            v
        } else {
            eprintln!("Sparse SVD");
            // Large, use dense SVD
            let j = SparseColMatRef::new(self.jc.sym.as_ref(), &self.jc.vals);
            let jtj = j.transpose().to_col_major()? * j;
            let a = jtj + &self.lambda_i;

            // Figure out the parameters
            let mut u = faer::Mat::zeros(n, k);
            let mut v = faer::Mat::zeros(n, k);
            let mut singular_values = vec![0.0; k];
            let par = get_global_parallelism();

            // Make a unit basis vector, with length n, it should be normalized (unit).
            // It's trivially normalized if we set all entries to 0 and the first one to 1.
            let mut v0_buf = vec![0.0_f64; n];
            v0_buf[0] = 1.0;
            let v0 = faer::ColRef::from_slice(&v0_buf);

            // Allocate the scratch space
            let estimated_memory_needed =
                partial_eigen_scratch(&a, n.max(m), par, Default::default());
            let mut memory = faer::dyn_stack::MemBuffer::new(estimated_memory_needed);
            let stack = faer::dyn_stack::MemStack::new(&mut memory);

            let _svd_info = faer::matrix_free::eigen::partial_svd(
                // Output matrix for U (n by k), the left_singular_rows
                u.as_mut(),
                // Output matrix for V (n by k), the right_singular_rows
                v.as_mut(),
                // length k, output sigma (largest first)
                &mut singular_values,
                // square operator, n by n
                &a,
                // length n start vector (normalized)
                v0,
                // convergence threshold for residuals
                self.config.convergence_tolerance,
                par,
                // scratch space
                stack,
                // Parameters
                Default::default(),
            );
            singular_values
        };

        // The system is underconstrained if there's too many singular values
        // close to 0. How close to 0? The tolerance should be derived from
        // the largest singular value.
        let largest_singular_value = sigma_col
            .iter()
            .copied()
            .reduce(f64::max)
            .ok_or(NonLinearSystemError::EmptySystemNotAllowed)?;
        let tolerance = 1e-8 * largest_singular_value;

        let rank = sigma_col.iter().filter(|&&s| s > tolerance).count();
        let degrees_of_freedom = n - rank;
        Ok(degrees_of_freedom > 0)
    }
}
