use faer::{
    Accum, ColRef, Par,
    dyn_stack::{MemBuffer, MemStack},
    get_global_parallelism,
    prelude::Solve,
    sparse::{
        SparseColMatMut, SparseColMatRef,
        linalg::{matmul, solvers::Lu},
        ops,
    },
};

use crate::{Config, NonLinearSystemError};

use super::Model;

impl Model<'_> {
    #[inline(never)]
    pub fn solve_gauss_newton(
        &mut self,
        current_values: &mut [f64],
        config: Config,
    ) -> Result<usize, NonLinearSystemError> {
        let m = self.layout.total_num_residuals;
        let mut global_residual = vec![0.0; m];

        // Preallocate scratch space for computing JᵀJ.
        let jtj_nnz = self.jtj_symbolic.0.compute_nnz();
        let jtj_scratch_req = {
            let (jtj_sym, _) = &self.jtj_symbolic;
            matmul::sparse_sparse_matmul_numeric_scratch::<usize, f64>(jtj_sym.as_ref(), Par::Seq)
        };
        let mut jtj_vals = vec![0.0; jtj_nnz];
        let mut jtj_mem = MemBuffer::new(jtj_scratch_req);
        let mut a_vals = vec![0.0; self.a_sym.compute_nnz()];
        let mut jt_vals = vec![0.0; self.jt_sym.compute_nnz()];

        for this_iteration in 0..config.max_iterations {
            // Assemble global residual and Jacobian
            // Re-evaluate the global residual.
            self.residual(current_values, &mut global_residual);
            // Re-evaluate the global jacobian, write it into self.jc
            self.refresh_jacobian(current_values);
            let (jtj_sym, jtj_info) = &self.jtj_symbolic;
            for (jt_idx, jc_idx) in self.jt_value_indices.iter().copied().enumerate() {
                jt_vals[jt_idx] = self.jc.vals[jc_idx];
            }

            // Convergence check: if the residual is within our tolerance,
            // then the system is totally solved and we can return.
            let largest_absolute_elem = global_residual
                .iter()
                .map(|x| x.abs())
                .reduce(f64::max)
                .ok_or(NonLinearSystemError::EmptySystemNotAllowed)?;
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
            let jt = SparseColMatRef::new(self.jt_sym.as_ref(), &jt_vals);

            // Compute JᵀJ, reusing its symbolic structure.
            let jtj_stack = MemStack::new(&mut jtj_mem);
            matmul::sparse_sparse_matmul_numeric(
                SparseColMatMut::new(jtj_sym.as_ref(), &mut jtj_vals),
                Accum::Replace,
                jt.as_ref(),
                j,
                1.0,
                jtj_info,
                get_global_parallelism(),
                jtj_stack,
            );
            let jtj = SparseColMatRef::new(jtj_sym.as_ref(), &jtj_vals);

            a_vals.fill(0.0);
            ops::binary_op_assign_into(
                SparseColMatMut::new(self.a_sym.as_ref(), &mut a_vals),
                jtj,
                |dst, src| {
                    *dst = *src.unwrap_or(&0.0);
                },
            );
            ops::binary_op_assign_into(
                SparseColMatMut::new(self.a_sym.as_ref(), &mut a_vals),
                self.lambda_i.as_ref(),
                |dst, src| {
                    if let Some(val) = src {
                        *dst += *val;
                    }
                },
            );
            let a = SparseColMatRef::new(self.a_sym.as_ref(), &a_vals);
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
            let step_threshold = config.step_tolerance * (current_inf_norm + config.step_tolerance);

            // Convergence check: if `d` is small enough,
            // then the system is at a local minimum. It might be inconsistent, and therefore
            // its residual will never get close to zero, but this is still a good least-squares solution,
            // so we can return.
            if step_inf_norm <= step_threshold {
                return Ok(this_iteration);
            }
        }
        Err(NonLinearSystemError::DidNotConverge)
    }
}
