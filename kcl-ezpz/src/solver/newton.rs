use faer::{
    ColRef,
    prelude::Solve,
    sparse::{SparseColMat, SparseColMatRef},
};

use crate::{Config, NonLinearSystemError, solver::REGULARIZATION_LAMBDA};

use super::Model;

impl Model<'_> {
    pub fn run_newtons_method(
        &mut self,
        current_values: &mut [f64],
        config: Config,
    ) -> Result<usize, NonLinearSystemError> {
        let m = self.layout.total_num_residuals;
        let n = self.layout.num_variables;

        let mut global_residual = vec![0.0; m];

        // Used in the matrix math below.
        // This 'damps' the jacobian matrix, ensuring that as its coefficients get smaller,
        // the solver takes smaller and smaller steps.
        let lambda_i = SparseColMat::<usize, f64>::try_new_from_triplets(
            n,
            n,
            &(0..n)
                .map(|i| faer::sparse::Triplet::new(i, i, REGULARIZATION_LAMBDA))
                .collect::<Vec<_>>(),
        )
        .unwrap();

        for this_iteration in 0..config.max_iterations {
            // Assemble global residual and Jacobian
            // Re-evaluate the global residual.
            self.residual(current_values, &mut global_residual);
            // Re-evaluate the global jacobian, write it into self.jc
            self.refresh_jacobian(current_values);

            // Converged if residual is within tolerance
            // TODO: Is there a way to do this in faer, treating global_residual as a 1xN matrix
            // or a 1D vec?
            // David's code:
            // if (r.array().abs().maxCoeff() <= params.tolerance)
            // let largest = ColRef::from_slice(&global_residual)
            let largest_absolute_elem = global_residual
                .iter()
                .map(|x| x.abs())
                .reduce(f64::max)
                .unwrap();
            if largest_absolute_elem <= config.convergence_tolerance {
                return Ok(this_iteration);
            }

            /* NOTE(dr): We solve the following linear system to get the damped Gauss-Newton step d
               (JᵀJ + λI) d = -Jᵀr
               This involves creating a matrix A and rhs b where
               A = JᵀJ + λI
               b = -Jᵀr
            */

            let j = SparseColMatRef::new(self.jc.sym.as_ref(), &self.jc.vals);
            let jtj = j.transpose().to_col_major().unwrap() * j;
            let a = jtj + &lambda_i;
            let b = j.transpose() * -ColRef::from_slice(&global_residual);

            // Solve linear system
            // David's code: `solver.compute(A)`;
            let factored = a.sp_lu().unwrap();
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
            let step_threshold = config.step_tolerance * (current_inf_norm + config.step_tolerance);
            if step_inf_norm <= step_threshold {
                return Ok(this_iteration);
            }
        }
        Err(NonLinearSystemError::DidNotConverge)
    }
}
