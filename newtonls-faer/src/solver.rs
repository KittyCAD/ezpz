use super::{
    LinearSolver, NonlinearSystem, RowMap, SolverError, SolverResult, SparseColMatRef,
    init_global_parallelism,
    linalg::{DenseLu, FaerLu, SparseQr},
};
use error_stack::Report;
use faer::mat::Mat as FaerMat;
use faer_traits::ComplexField;
use num_traits::{Float, One, ToPrimitive, Zero};
use std::panic;

const AUTO_DENSE_THRESHOLD: usize = 100;
const FTOL_DEFAULT: f64 = 1e-8;
const XTOL_DEFAULT: f64 = 1e-8;
const GTOL_DEFAULT: f64 = 1e-8;

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
            tol: T::from(FTOL_DEFAULT).unwrap(),
            tol_grad: T::from(GTOL_DEFAULT).unwrap(),
            tol_step: T::from(XTOL_DEFAULT).unwrap(),
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

fn compute_residual_norm<T: Float>(f: &[T], norm_type: NormType) -> T {
    match norm_type {
        NormType::LInf => f.iter().map(|&v| v.abs()).fold(T::zero(), |a, b| a.max(b)),
        NormType::L2 => f
            .iter()
            .map(|&v| v.powi(2))
            .fold(T::zero(), |a, b| a + b)
            .sqrt(),
    }
}

fn compute_step_norm<T: Float>(step: &[T], x: &[T], tol: T) -> T {
    // Compute ||dx|| / (||x|| + tol) similar to scipy's approach.
    // Basically, we want to check for very small steps relative to the current solution.
    // We do this by checking the L2 norm of the step vector, then comparing that
    // to the current solution vector... with some divide by zero guard using the tol.
    let step_norm = step
        .iter()
        .map(|&v| v.powi(2))
        .fold(T::zero(), |a, b| a + b)
        .sqrt();
    let x_norm = x
        .iter()
        .map(|&v| v.powi(2))
        .fold(T::zero(), |a, b| a + b)
        .sqrt();

    step_norm / (x_norm + tol)
}

fn compute_gradient_norm_sparse<T: Float>(
    jacobian: &SparseColMatRef<'_, usize, T>,
    residual: &[T],
) -> T {
    // Compute ||J^T * r||_Inf (infinity norm of the gradient).
    // We want the largest absolute component of the least-squares gradient g = J^T r.
    // We do this by looking through each column of J, computing g_j = dot(J[:, j], residual),
    // then tracking the maximum absolute value over all j.
    // If the value is very small, we're at or around a stationary point.
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

    max_grad
}

fn compute_gradient_norm_dense<T: Float>(jacobian: &FaerMat<T>, residual: &[T]) -> T {
    // Compute ||J^T * r||_Inf (infinity norm of the gradient).
    // We want the largest absolute component of the least-squares gradient g = J^T r.
    // For dense matrices, we compute g_j = sum(J[i,j] * residual[i]) for each column j.
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

    max_grad
}

fn compute_gradient_norm<M>(
    model: &mut M,
    residual: &[M::Real],
    dense_jacobian: Option<&FaerMat<M::Real>>,
) -> SolverResult<M::Real>
where
    M: NonlinearSystem,
    M::Real: Float,
{
    if let Some(jac_dense) = dense_jacobian {
        // Dense case: use provided Jacobian matrix
        Ok(compute_gradient_norm_dense(jac_dense, residual))
    } else {
        // Sparse case: use model's Jacobian
        let jac_ref = model.jacobian().attach();
        Ok(compute_gradient_norm_sparse(&jac_ref, residual))
    }
}

fn newton_iterate<M, F, Cb>(
    model: &mut M,
    x: &mut [M::Real],
    cfg: NewtonCfg<M::Real>,
    norm_type: NormType,
    mut solve: F,
    mut on_iter: Cb,
) -> SolverResult<Iterations>
where
    M: NonlinearSystem,
    M::Real: ComplexField<Real = M::Real> + Float + Zero + One + ToPrimitive,
    F: FnMut(
        &mut M,
        &[M::Real],
        &[M::Real],
        &mut [M::Real],
    ) -> SolverResult<Option<FaerMat<M::Real>>>,
    Cb: FnMut(&IterationStats<M::Real>) -> Control,
{
    let n_vars = model.layout().n_variables();
    let n_res = model.layout().n_residuals();

    // `f` holds residuals, `dx` holds the step for the variables.
    let mut f = vec![M::Real::zero(); n_res];
    let mut dx = vec![M::Real::zero(); n_vars];
    let mut damping = cfg.damping;
    let mut last_res = M::Real::infinity();

    // Buffers for line search.
    let mut x_trial = vec![M::Real::zero(); n_vars];
    let mut f_trial = vec![M::Real::zero(); n_res];

    for iter in 0..cfg.max_iter {
        model.residual(x, &mut f);
        let res = compute_residual_norm(&f, norm_type);

        // First convergence check: just check residual (ftol). If we're close enough,
        // we don't actually need to run the step.
        if res < cfg.tol {
            return Ok(iter);
        }

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

        // Solve linear system: J(x) * dx = -f(x).
        // TODO: This is kinda clumsy and inconsistent. Our dense version will return a
        // Jacobian, sparse won't; it just uses model.jacobian() directly.
        let jacobian = solve(model, x, &f, &mut dx)?;

        // Second convergence check: now we have dx (step size), check for small step (xtol).
        // This would really apply at the _next_ iteration, but we can catch it here and
        // save some work.
        // TODO: Maybe this should consider damping.
        if cfg.tol_step > M::Real::zero() {
            let step_norm = compute_step_norm(&dx, x, cfg.tol_step);
            if step_norm < cfg.tol_step {
                return Ok(iter + 1);
            }
        }

        // Third convergence check: check gradient norm via Jacobian we have
        // just updated as part of solve (gtol). This would really apply at the _next_
        // iteration, but we can catch it here and save some work.
        if cfg.tol_grad > M::Real::zero() {
            let grad_norm = compute_gradient_norm(model, &f, jacobian.as_ref())?;
            if grad_norm < cfg.tol_grad {
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
                    let res_try = compute_residual_norm(&f_trial, norm_type);

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
    cfg: NewtonCfg<M::Real>,
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
    cfg: NewtonCfg<M::Real>,
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
        solve_dense_lu(model, x, cfg, on_iter)
    } else if is_square {
        solve_sparse_lu_with_qr_fallback(model, x, cfg, on_iter)
    } else {
        solve_sparse_qr(model, x, cfg, on_iter)
    }
}

fn solve_dense_lu<M, Cb>(
    model: &mut M,
    x: &mut [M::Real],
    cfg: NewtonCfg<M::Real>,
    on_iter: Cb,
) -> SolverResult<Iterations>
where
    M: NonlinearSystem,
    M::Real: ComplexField<Real = M::Real> + Float + Zero + One + ToPrimitive,
    Cb: FnMut(&IterationStats<M::Real>) -> Control,
{
    let n = model.layout().n_variables();
    let mut lu = DenseLu::<M::Real>::default();
    let mut jac = FaerMat::<M::Real>::zeros(n, n);
    let mut rhs = FaerMat::<M::Real>::zeros(n, 1);

    #[allow(clippy::too_many_arguments)]
    fn solve_inner<T>(
        model: &mut impl NonlinearSystem<Real = T>,
        x: &[T],
        f: &[T],
        dx: &mut [T],
        lu: &mut DenseLu<T>,
        jac: &mut FaerMat<T>,
        rhs: &mut FaerMat<T>,
    ) -> SolverResult<Option<FaerMat<T>>>
    where
        T: ComplexField<Real = T> + Float + Zero + One + ToPrimitive,
    {
        // Update Jacobian and solve.
        model.jacobian_dense(x, jac);
        lu.factor(jac)?;

        for (i, &fi) in f.iter().enumerate() {
            rhs[(i, 0)] = -fi;
        }
        lu.solve_in_place(rhs.as_mut())?;

        for (i, &val) in rhs.col(0).iter().enumerate() {
            dx[i] = val;
        }

        // Return a copy of the Jacobian for gradient computation.
        Ok(Some(jac.clone()))
    }

    // Run iterative loop.
    newton_iterate(
        model,
        x,
        cfg,
        NormType::LInf,
        |model, x, f, dx| solve_inner(model, x, f, dx, &mut lu, &mut jac, &mut rhs),
        on_iter,
    )
}

fn solve_sparse<M, S, Cb>(
    model: &mut M,
    x: &mut [M::Real],
    cfg: NewtonCfg<M::Real>,
    norm_type: NormType,
    mut solver: S,
    on_iter: Cb,
) -> SolverResult<Iterations>
where
    M: NonlinearSystem,
    M::Real: ComplexField<Real = M::Real> + Float + Zero + One + ToPrimitive,
    S: for<'a> LinearSolver<M::Real, SparseColMatRef<'a, usize, M::Real>>,
    Cb: FnMut(&IterationStats<M::Real>) -> Control,
{
    let n_vars = model.layout().n_variables();
    let n_res = model.layout().n_residuals();
    let mut rhs = FaerMat::<M::Real>::zeros(n_res, 1);

    #[allow(clippy::too_many_arguments)]
    fn solve_inner<T, S>(
        model: &mut impl NonlinearSystem<Real = T>,
        x: &[T],
        f: &[T],
        dx: &mut [T],
        solver: &mut S,
        rhs: &mut FaerMat<T>,
        n_vars: usize,
    ) -> SolverResult<Option<FaerMat<T>>>
    where
        T: ComplexField<Real = T> + Float + Zero + One + ToPrimitive,
        S: for<'a> LinearSolver<T, SparseColMatRef<'a, usize, T>>,
    {
        // Update Jacobian and solve.
        model.refresh_jacobian(x);
        let jac_ref = model.jacobian().attach();
        solver.factor(&jac_ref)?;

        rhs.col_mut(0)
            .as_mut()
            .iter_mut()
            .zip(f.iter())
            .for_each(|(dst, &src)| *dst = -src);

        solver.solve_in_place(rhs.as_mut())?;

        for (i, &val) in rhs.col(0).iter().take(n_vars).enumerate() {
            dx[i] = val;
        }

        // Sparse systems use model.jacobian() directly.
        Ok(None)
    }

    // Run iterative loop.
    newton_iterate(
        model,
        x,
        cfg,
        norm_type,
        |model, x, f, dx| solve_inner(model, x, f, dx, &mut solver, &mut rhs, n_vars),
        on_iter,
    )
}

fn solve_sparse_lu<M, Cb>(
    model: &mut M,
    x: &mut [M::Real],
    cfg: NewtonCfg<M::Real>,
    on_iter: Cb,
) -> SolverResult<Iterations>
where
    M: NonlinearSystem,
    M::Real: ComplexField<Real = M::Real> + Float + Zero + One + ToPrimitive,
    Cb: FnMut(&IterationStats<M::Real>) -> Control,
{
    solve_sparse(
        model,
        x,
        cfg,
        NormType::LInf,
        FaerLu::<M::Real>::default(),
        on_iter,
    )
}

fn solve_sparse_qr<M, Cb>(
    model: &mut M,
    x: &mut [M::Real],
    cfg: NewtonCfg<M::Real>,
    on_iter: Cb,
) -> SolverResult<Iterations>
where
    M: NonlinearSystem,
    M::Real: ComplexField<Real = M::Real> + Float + Zero + One + ToPrimitive,
    Cb: FnMut(&IterationStats<M::Real>) -> Control,
{
    solve_sparse(
        model,
        x,
        cfg,
        NormType::L2,
        SparseQr::<M::Real>::default(),
        on_iter,
    )
}

fn solve_sparse_lu_with_qr_fallback<M, Cb>(
    model: &mut M,
    x: &mut [M::Real],
    cfg: NewtonCfg<M::Real>,
    mut on_iter: Cb,
) -> SolverResult<Iterations>
where
    M: NonlinearSystem,
    M::Real: ComplexField<Real = M::Real> + Float + Zero + One + ToPrimitive,
    Cb: FnMut(&IterationStats<M::Real>) -> Control,
{
    // Try LU with panic catching.
    let lu_result = panic::catch_unwind(panic::AssertUnwindSafe(|| {
        solve_sparse_lu(model, x, cfg, &mut on_iter)
    }));

    match lu_result {
        Ok(Ok(iterations)) => Ok(iterations),
        Ok(Err(lu_error)) => Err(lu_error), // Normal error
        Err(_panic) => {
            // Panic occurred (likely singular matrix), try QR.
            solve_sparse_qr(model, x, cfg, on_iter)
        }
    }
}
