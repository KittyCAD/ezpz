use faer::{
    ColRef, Par,
    dyn_stack::{MemBuffer, MemStack, StackReq},
    get_global_parallelism,
    linalg::{svd::SvdError, temp_mat_scratch, temp_mat_zeroed},
    mat::AsMatMut,
    matrix_free::{
        BiLinOp, LinOp,
        eigen::{PartialEigenParams, partial_svd},
    },
    prelude::{Reborrow, ReborrowMut, Solve},
    sparse::{SparseColMatRef, linalg::solvers::Lu},
};

use crate::{Config, NonLinearSystemError};

use super::Model;

#[derive(Debug)]
pub struct SuccessfulSolve {
    pub iterations: usize,
}

#[derive(Debug, Clone, Copy)]
struct NormalOp<'a> {
    a: SparseColMatRef<'a, usize, f64>,
}

impl<'a> LinOp<f64> for NormalOp<'a> {
    #[inline]
    fn apply_scratch(&self, rhs_ncols: usize, par: Par) -> StackReq {
        temp_mat_scratch::<f64>(self.a.nrows(), rhs_ncols)
            .and(self.a.apply_scratch(rhs_ncols, par))
            .and(self.a.transpose_apply_scratch(rhs_ncols, par))
    }

    #[inline]
    fn nrows(&self) -> usize {
        self.a.ncols()
    }

    #[inline]
    fn ncols(&self) -> usize {
        self.a.ncols()
    }

    #[inline]
    fn apply(
        &self,
        out: faer::MatMut<'_, f64>,
        rhs: faer::MatRef<'_, f64>,
        par: Par,
        stack: &mut MemStack,
    ) {
        let (mut temp, stack) = temp_mat_zeroed(self.a.nrows(), rhs.ncols(), stack);
        let mut temp = temp.as_mat_mut();
        self.a.apply(temp.rb_mut(), rhs, par, stack);
        self.a.adjoint_apply(out, temp.rb(), par, stack);
    }

    #[inline]
    fn conj_apply(
        &self,
        out: faer::MatMut<'_, f64>,
        rhs: faer::MatRef<'_, f64>,
        par: Par,
        stack: &mut MemStack,
    ) {
        self.apply(out, rhs, par, stack);
    }
}

impl<'a> BiLinOp<f64> for NormalOp<'a> {
    #[inline]
    fn transpose_apply_scratch(&self, rhs_ncols: usize, par: Par) -> StackReq {
        self.apply_scratch(rhs_ncols, par)
    }

    #[inline]
    fn transpose_apply(
        &self,
        out: faer::MatMut<'_, f64>,
        rhs: faer::MatRef<'_, f64>,
        par: Par,
        stack: &mut MemStack,
    ) {
        self.apply(out, rhs, par, stack);
    }

    #[inline]
    fn adjoint_apply(
        &self,
        out: faer::MatMut<'_, f64>,
        rhs: faer::MatRef<'_, f64>,
        par: Par,
        stack: &mut MemStack,
    ) {
        self.apply(out, rhs, par, stack);
    }
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
                .reduce(f64::max)
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
                return Ok(SuccessfulSolve {
                    iterations: this_iteration,
                });
            }
        }
        Err(NonLinearSystemError::DidNotConverge)
    }

    pub fn is_underconstrained(&self) -> Result<bool, NonLinearSystemError> {
        // Estimate the singular values with a matrix-free SVD of the normal operator AᵀA.
        let j_sparse = SparseColMatRef::new(self.jc.sym.as_ref(), &self.jc.vals);
        let m = j_sparse.nrows();
        let n = j_sparse.ncols();
        let k = usize::min(m, n);
        if k == 0 {
            return Err(NonLinearSystemError::EmptySystemNotAllowed);
        }

        let normal_op = NormalOp { a: j_sparse };

        let mut u = faer::Mat::<f64>::zeros(n, k);
        let mut v = faer::Mat::<f64>::zeros(n, k);
        let mut s = vec![0.0_f64; k];
        let mut v0_buf = vec![0.0_f64; n];
        v0_buf[0] = 1.0;
        let v0 = ColRef::from_slice(&v0_buf);

        let min_dim = usize::min(n, usize::max(32, k));
        let max_dim = usize::min(n, usize::max(min_dim * 2, k));
        let params = PartialEigenParams {
            min_dim,
            max_dim,
            ..Default::default()
        };

        let mut mem = MemBuffer::new(StackReq::new::<u8>(512 * 1024 * 1024));
        let spectral_tolerance = 1e-12;
        let info = partial_svd(
            u.as_mut(),
            v.as_mut(),
            &mut s,
            &normal_op,
            v0,
            spectral_tolerance,
            get_global_parallelism(),
            MemStack::new(&mut mem),
            params,
        );
        if info.n_converged_eigen == 0 {
            return Err(NonLinearSystemError::FaerSvd(SvdError::NoConvergence));
        }

        // Holds the singular values for JᵀJ.
        let s_slice = &s[..info.n_converged_eigen];
        let largest_singular_value = s_slice
            .iter()
            .copied()
            .reduce(f64::max)
            .ok_or(NonLinearSystemError::EmptySystemNotAllowed)?;
        let rank_tol = 1e-8 * largest_singular_value;
        let rank = s_slice.iter().filter(|&&sv| sv > rank_tol).count();

        Ok(rank < self.layout.num_variables)
    }
}
