use super::{
    LinearSolver, Mat, NonlinearSystem, RowMap, SolverError, SolverResult, SparseColMatRef,
    init_global_parallelism,
    linalg::{DenseLu, FaerLu, SparseQr},
};
use error_stack::Report;
use faer::mat::Mat as FaerMat;
use faer_traits::ComplexField;
use num_traits::{Float, One, ToPrimitive, Zero};

const AUTO_DENSE_THRESHOLD: usize = 100;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MatrixFormat {
    Sparse,
    Dense,
    Auto,
}

impl Default for MatrixFormat {
    fn default() -> Self {
        // We'll drive sparse/dense decision based on AUTO_DENSE_THRESHOLD.
        Self::Auto
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NormType {
    L2,
    LInf,
}

#[derive(Clone, Copy, Debug)]
pub struct NewtonCfg<T> {
    pub tol: T,
    pub tol_grad: T,
    pub tol_step: T,
    pub damping: T,
    pub max_iter: usize,
    pub format: MatrixFormat,

    // step control
    pub adaptive: bool,
    pub min_damping: T,
    pub max_damping: T,
    pub grow: T,
    pub shrink: T,
    pub divergence_ratio: T,
    pub ls_backtrack: T,
    pub ls_max_steps: usize,

    pub n_threads: usize,
}

impl<T: Float> Default for NewtonCfg<T> {
    fn default() -> Self {
        let _ = init_global_parallelism(0);
        Self {
            tol: T::from(1e-8).expect("Type must support 1e-8 for default tolerance"),
            tol_grad: T::from(1e-8).expect("Type must support 1e-8 for default gradient tolerance"),
            tol_step: T::from(1e-8).expect("Type must support 1e-8 for default step tolerance"),
            damping: T::one(),
            max_iter: 50,
            format: MatrixFormat::default(),
            adaptive: false,
            min_damping: T::from(0.1).unwrap(),
            max_damping: T::one(),
            grow: T::from(1.1).unwrap(),
            shrink: T::from(0.5).unwrap(),
            divergence_ratio: T::from(3.0).unwrap(),
            ls_backtrack: T::from(0.5).unwrap(),
            ls_max_steps: 10,
            n_threads: 0,
        }
    }
}

impl<T: Float> NewtonCfg<T> {
    pub fn sparse() -> Self {
        Self {
            format: MatrixFormat::Sparse,
            ..Default::default()
        }
    }
    pub fn dense() -> Self {
        Self {
            format: MatrixFormat::Dense,
            ..Default::default()
        }
    }
    pub fn with_format(mut self, format: MatrixFormat) -> Self {
        self.format = format;
        self
    }
    pub fn with_adaptive(mut self, enabled: bool) -> Self {
        self.adaptive = enabled;
        self
    }
    pub fn with_threads(mut self, n_threads: usize) -> Self {
        init_global_parallelism(n_threads);
        self.n_threads = n_threads;
        self
    }
    pub fn with_tol(mut self, tol: T) -> Self {
        self.tol = tol;
        self
    }
    pub fn with_tol_grad(mut self, tol_grad: T) -> Self {
        self.tol_grad = tol_grad;
        self
    }
    pub fn with_tol_step(mut self, tol_step: T) -> Self {
        self.tol_step = tol_step;
        self
    }
}

pub type Iterations = usize;

#[derive(Clone, Debug)]
pub struct IterationStats<T> {
    pub iter: usize,
    pub residual: T,
    pub damping: T,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Control {
    Continue,
    Cancel,
}

fn compute_residual_norm<T: Float>(f: &[T], norm_kind: NormType) -> T {
    match norm_kind {
        NormType::LInf => f.iter().map(|&v| v.abs()).fold(T::zero(), |a, b| a.max(b)),
        NormType::L2 => f
            .iter()
            .map(|&v| v.powi(2))
            .fold(T::zero(), |a, b| a + b)
            .sqrt(),
    }
}

fn compute_gradient_norm_sparse<T: Float>(
    jacobian: &SparseColMatRef<'_, usize, T>,
    residual: &[T],
) -> T {
    // Compute ||J^T * r||_Inf (infinity norm of the gradient).
    let mut max_grad = T::zero();

    for col in 0..jacobian.ncols() {
        let mut grad_component = T::zero();
        let range = jacobian.col_range(col);
        let row_idx = jacobian.symbolic().row_idx();
        let vals = jacobian.val();

        for idx in range {
            grad_component = grad_component + vals[idx] * residual[row_idx[idx]];
        }

        let abs_grad = grad_component.abs();
        if abs_grad > max_grad {
            max_grad = abs_grad;
        }
    }

    return max_grad;
}

fn compute_gradient_norm_dense<T: Float>(jacobian: &Mat<T>, residual: &[T]) -> T {
    // Compute ||J^T * r||_Inf (infinity norm of the gradient).
    let mut max_grad = T::zero();

    for col in 0..jacobian.ncols() {
        let mut grad_component = T::zero();
        for row in 0..jacobian.nrows() {
            grad_component = grad_component + jacobian[(row, col)] * residual[row];
        }

        let abs_grad = grad_component.abs();
        if abs_grad > max_grad {
            max_grad = abs_grad;
        }
    }

    return max_grad;
}

fn compute_step_norm<T: Float>(step: &[T], x: &[T]) -> T {
    // Compute ||dx|| / (||x|| + tol_step) similar to scipy's approach
    let step_norm = step
        .iter()
        .map(|&v| v * v)
        .fold(T::zero(), |a, b| a + b)
        .sqrt();
    let x_norm = x
        .iter()
        .map(|&v| v * v)
        .fold(T::zero(), |a, b| a + b)
        .sqrt();

    return step_norm / (x_norm + T::from(1e-8).unwrap_or_else(|| T::zero()));
}

fn newton_iterate_sparse<M, L, Cb>(
    model: &mut M,
    x: &mut [M::Real],
    lin: &mut L,
    cfg: super::NewtonCfg<M::Real>,
    norm_kind: NormType,
    mut on_iter: Cb,
) -> SolverResult<Iterations>
where
    M: NonlinearSystem,
    L: for<'a> LinearSolver<M::Real, SparseColMatRef<'a, usize, M::Real>>,
    M::Real: ComplexField<Real = M::Real> + Float + Zero + One + ToPrimitive,
    Cb: FnMut(&IterationStats<M::Real>) -> Control,
{
    let n_vars = model.layout().n_variables();
    let n_res = model.layout().n_residuals();

    // `f` holds residuals, `dx` holds the step for the variables.
    let mut f = vec![M::Real::zero(); n_res];
    let mut dx = vec![M::Real::zero(); n_vars];
    let mut damping = cfg.damping;
    let mut last_res = M::Real::infinity();

    // buffers for line search
    let mut x_trial = vec![M::Real::zero(); n_vars];
    let mut f_trial = vec![M::Real::zero(); n_res];
    let mut rhs = FaerMat::<M::Real>::zeros(n_res, 1);

    for iter in 0..cfg.max_iter {
        model.residual(x, &mut f);
        let res = compute_residual_norm(&f, norm_kind);

        if matches!(
            on_iter(&IterationStats {
                iter,
                residual: res,
                damping
            }),
            Control::Cancel
        ) {
            return Err(Report::new(SolverError).attach_printable("solve cancelled"));
        }

        // Check residual tolerance; ftol in scipy world.
        if res < cfg.tol {
            return Ok(iter + 1);
        }

        // Update Jacobian and factor.
        model.refresh_jacobian(x);
        lin.factor(&model.jacobian().attach())?;

        // Check gradient tolerance; gtol equivalent.
        if cfg.tol_grad > M::Real::zero() {
            let grad_norm = compute_gradient_norm_sparse(&model.jacobian().attach(), &f);
            if grad_norm < cfg.tol_grad {
                return Ok(iter + 1);
            }
        }

        // Solve for step.
        rhs.col_mut(0)
            .as_mut()
            .iter_mut()
            .zip(f.iter())
            .for_each(|(dst, &src)| *dst = -src);

        lin.solve_in_place(rhs.as_mut())?;

        // Extract step from solution.
        for (i, &val) in rhs.col(0).iter().take(n_vars).enumerate() {
            dx[i] = val;
        }

        // Check step tolerance; xtol equivalent.
        if cfg.tol_step > M::Real::zero() {
            let step_norm = compute_step_norm(&dx, x);
            if step_norm < cfg.tol_step {
                return Ok(iter + 1);
            }
        }

        let mut step_applied = false;

        if cfg.adaptive {
            if res < last_res {
                let nd = damping * cfg.grow;
                damping = if nd > cfg.max_damping {
                    cfg.max_damping
                } else {
                    nd
                };
            } else {
                let nd = damping * cfg.shrink;
                damping = if nd < cfg.min_damping {
                    cfg.min_damping
                } else {
                    nd
                };
            }

            if last_res.is_finite() && res > last_res * cfg.divergence_ratio {
                let mut alpha = if damping * cfg.shrink < cfg.min_damping {
                    cfg.min_damping
                } else {
                    damping * cfg.shrink
                };

                for _ in 0..cfg.ls_max_steps {
                    for i in 0..n_vars {
                        x_trial[i] = x[i] + alpha * dx[i];
                    }
                    model.residual(&x_trial, &mut f_trial);
                    let res_try = compute_residual_norm(&f_trial, norm_kind);

                    if res_try < res {
                        x.copy_from_slice(&x_trial);
                        damping = alpha;
                        step_applied = true;
                        break;
                    }
                    alpha = alpha * cfg.ls_backtrack;
                    if alpha < cfg.min_damping {
                        break;
                    }
                }

                if !step_applied {
                    return Err(Report::new(SolverError)
                        .attach_printable("divergence guard: line search failed"));
                }
            }
        }

        if !step_applied {
            for (xi, &dxi) in x.iter_mut().zip(dx.iter()) {
                *xi = *xi + damping * dxi;
            }
        }

        last_res = res;
    }

    Err(Report::new(SolverError).attach_printable(format!(
        "Newton solver did not converge after {} iterations",
        cfg.max_iter
    )))
}

fn newton_iterate_dense<M, L, Cb>(
    model: &mut M,
    x: &mut [M::Real],
    lu: &mut L,
    cfg: super::NewtonCfg<M::Real>,
    norm_kind: NormType,
    mut on_iter: Cb,
) -> SolverResult<Iterations>
where
    M: NonlinearSystem,
    L: LinearSolver<M::Real, Mat<M::Real>>,
    M::Real: ComplexField<Real = M::Real> + Float + Zero + One + ToPrimitive,
    Cb: FnMut(&IterationStats<M::Real>) -> Control,
{
    let n = model.layout().n_variables();

    // `f` holds residuals, `dx` holds the step for the variables.
    let mut f = vec![M::Real::zero(); n];
    let mut dx = vec![M::Real::zero(); n];
    let mut damping = cfg.damping;
    let mut last_res = M::Real::infinity();

    // buffers for line search
    let mut x_trial = vec![M::Real::zero(); n];
    let mut f_trial = vec![M::Real::zero(); n];
    let mut jac = FaerMat::<M::Real>::zeros(n, n);
    let mut rhs = FaerMat::<M::Real>::zeros(n, 1);

    for iter in 0..cfg.max_iter {
        model.residual(x, &mut f);
        let res = compute_residual_norm(&f, norm_kind);

        if matches!(
            on_iter(&IterationStats {
                iter,
                residual: res,
                damping
            }),
            Control::Cancel
        ) {
            return Err(Report::new(SolverError).attach_printable("solve cancelled"));
        }

        // Check residual tolerance; ftol in scipy world.
        if res < cfg.tol {
            return Ok(iter + 1);
        }

        // Update Jacobian and factor
        model.jacobian_dense(x, &mut jac);
        lu.factor(&jac)?;

        // Check gradient tolerance (gtol equivalent)
        if cfg.tol_grad > M::Real::zero() {
            let grad_norm = compute_gradient_norm_dense(&jac, &f);
            if grad_norm < cfg.tol_grad {
                return Ok(iter + 1);
            }
        }

        // Solve for step
        for (i, &fi) in f.iter().enumerate() {
            rhs[(i, 0)] = -fi;
        }
        lu.solve_in_place(rhs.as_mut())?;

        // Extract step from solution
        for (i, &val) in rhs.col(0).iter().enumerate() {
            dx[i] = val;
        }

        // Check step tolerance (xtol equivalent)
        if cfg.tol_step > M::Real::zero() {
            let step_norm = compute_step_norm(&dx, x);
            if step_norm < cfg.tol_step {
                return Ok(iter + 1);
            }
        }

        let mut step_applied = false;

        if cfg.adaptive {
            if res < last_res {
                let nd = damping * cfg.grow;
                damping = if nd > cfg.max_damping {
                    cfg.max_damping
                } else {
                    nd
                };
            } else {
                let nd = damping * cfg.shrink;
                damping = if nd < cfg.min_damping {
                    cfg.min_damping
                } else {
                    nd
                };
            }

            if last_res.is_finite() && res > last_res * cfg.divergence_ratio {
                let mut alpha = if damping * cfg.shrink < cfg.min_damping {
                    cfg.min_damping
                } else {
                    damping * cfg.shrink
                };

                for _ in 0..cfg.ls_max_steps {
                    for i in 0..n {
                        x_trial[i] = x[i] + alpha * dx[i];
                    }
                    model.residual(&x_trial, &mut f_trial);
                    let res_try = compute_residual_norm(&f_trial, norm_kind);

                    if res_try < res {
                        x.copy_from_slice(&x_trial);
                        damping = alpha;
                        step_applied = true;
                        break;
                    }
                    alpha = alpha * cfg.ls_backtrack;
                    if alpha < cfg.min_damping {
                        break;
                    }
                }

                if !step_applied {
                    return Err(Report::new(SolverError)
                        .attach_printable("divergence guard: line search failed"));
                }
            }
        }

        if !step_applied {
            for (xi, &dxi) in x.iter_mut().zip(dx.iter()) {
                *xi = *xi + damping * dxi;
            }
        }

        last_res = res;
    }

    Err(Report::new(SolverError).attach_printable(format!(
        "Newton solver did not converge after {} iterations",
        cfg.max_iter
    )))
}

pub fn solve<M>(
    model: &mut M,
    x: &mut [M::Real],
    cfg: super::NewtonCfg<M::Real>,
) -> SolverResult<Iterations>
where
    M: NonlinearSystem,
    M::Real: ComplexField<Real = M::Real> + Float + Zero + One + ToPrimitive,
{
    solve_cb(model, x, cfg, |_| Control::Continue)
}

pub fn solve_cb<M, Cb>(
    model: &mut M,
    x: &mut [M::Real],
    cfg: super::NewtonCfg<M::Real>,
    on_iter: Cb,
) -> SolverResult<Iterations>
where
    M: NonlinearSystem,
    M::Real: ComplexField<Real = M::Real> + Float + Zero + One + ToPrimitive,
    Cb: FnMut(&IterationStats<M::Real>) -> Control,
{
    let n_vars = model.layout().n_variables();
    let n_res = model.layout().n_residuals();
    let is_square = n_vars == n_res;

    // We support: dense LU, sparse LU.
    let use_dense = if cfg.format == MatrixFormat::Dense {
        // User explicitly requested dense format.
        // Only allow if system is square; our dense methods can't deal with non-square.
        is_square
    } else if cfg.format == MatrixFormat::Sparse {
        // User explicitly requested sparse format.
        false
    } else {
        // Auto mode: use dense for smaller problems.
        is_square && n_vars < AUTO_DENSE_THRESHOLD
    };

    if use_dense {
        let mut lu = DenseLu::<M::Real>::default();
        solve_dense_cb(model, x, &mut lu, cfg, NormType::LInf, on_iter)
    } else if n_vars == n_res {
        // Square system: use LU factorization.
        let mut lu = FaerLu::<M::Real>::default();
        solve_sparse_cb(model, x, &mut lu, cfg, NormType::LInf, on_iter)
    } else {
        // Non-square system: use QR factorization for least squares.
        let mut qr = SparseQr::<M::Real>::default();
        solve_sparse_cb(model, x, &mut qr, cfg, NormType::L2, on_iter)
    }
}

pub fn solve_sparse_cb<M, L, Cb>(
    model: &mut M,
    x: &mut [M::Real],
    lin: &mut L,
    cfg: super::NewtonCfg<M::Real>,
    norm_kind: NormType,
    on_iter: Cb,
) -> SolverResult<Iterations>
where
    M: NonlinearSystem,
    L: for<'a> LinearSolver<M::Real, SparseColMatRef<'a, usize, M::Real>>,
    M::Real: ComplexField<Real = M::Real> + Float + Zero + One + ToPrimitive,
    Cb: FnMut(&IterationStats<M::Real>) -> Control,
{
    newton_iterate_sparse(model, x, lin, cfg, norm_kind, on_iter)
}

pub fn solve_dense_cb<M, L, Cb>(
    model: &mut M,
    x: &mut [M::Real],
    factorization: &mut L,
    cfg: super::NewtonCfg<M::Real>,
    norm_kind: NormType,
    on_iter: Cb,
) -> SolverResult<Iterations>
where
    M: NonlinearSystem,
    L: LinearSolver<M::Real, Mat<M::Real>>,
    M::Real: ComplexField<Real = M::Real> + Float + Zero + One + ToPrimitive,
    Cb: FnMut(&IterationStats<M::Real>) -> Control,
{
    newton_iterate_dense(model, x, factorization, cfg, norm_kind, on_iter)
}
