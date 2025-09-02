use super::{
    LinearSolver, Mat, NonlinearSystem, RowMap, SolverError, SolverResult, SparseColMatRef,
    init_global_parallelism,
    linalg::{DenseLu, FaerLu, SparseQr},
};
use error_stack::Report;
use faer::mat::Mat as FaerMat;
use faer_traits::ComplexField;
use num_traits::{Float, One, ToPrimitive, Zero};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MatrixFormat {
    Sparse,
    Dense,
    Auto(usize),
}

impl Default for MatrixFormat {
    fn default() -> Self {
        Self::Auto(100)
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
            damping: T::one(),
            max_iter: 25,
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

fn newton_iterate<M, F, Cb>(
    model: &mut M,
    x: &mut [M::Real],
    cfg: super::NewtonCfg<M::Real>,
    norm_kind: NormType,
    mut solve_into: F,
    mut on_iter: Cb,
) -> SolverResult<Iterations>
where
    M: NonlinearSystem,
    M::Real: ComplexField<Real = M::Real> + Float + Zero + One + ToPrimitive,
    F: FnMut(&mut M, &[M::Real], &[M::Real], &mut [M::Real]) -> SolverResult<()>,
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
        if res < cfg.tol {
            return Ok(iter + 1);
        }

        solve_into(model, x, &f, &mut dx)?;

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

    let use_dense = match cfg.format {
        super::MatrixFormat::Dense => true,
        super::MatrixFormat::Sparse => false,
        super::MatrixFormat::Auto(threshold) => n_vars < threshold,
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
    let n_vars = model.layout().n_variables();
    let n_res = model.layout().n_residuals();
    let mut rhs = FaerMat::<M::Real>::zeros(n_res, 1);

    newton_iterate(
        model,
        x,
        cfg,
        norm_kind,
        |model, x, f, dx| {
            model.refresh_jacobian(x);
            lin.factor(&model.jacobian().attach())?;

            rhs.col_mut(0)
                .as_mut()
                .iter_mut()
                .zip(f.iter())
                .for_each(|(dst, &src)| *dst = -src);

            // rhs is n_residuals x 1; smash -f in there.
            rhs.col_mut(0)
                .as_mut()
                .iter_mut()
                .zip(f.iter())
                .for_each(|(dst, &src)| *dst = -src);

            // In-place solve.
            // For QR least-squares, the top n_vars rows now contain the solution.
            lin.solve_in_place(rhs.as_mut())?;

            // Chop those top rows out and copy into dx.
            for (i, &val) in rhs.col(0).iter().take(n_vars).enumerate() {
                dx[i] = val;
            }

            Ok(())
        },
        on_iter,
    )
}

pub fn solve_dense_cb<M, L, Cb>(
    model: &mut M,
    x: &mut [M::Real],
    lu: &mut L,
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
    // This system is square so we can use m and n interchangeably;
    // n_variables == n_residuals.
    let n = model.layout().n_variables();
    let mut jac = FaerMat::<M::Real>::zeros(n, n);
    let mut rhs = FaerMat::<M::Real>::zeros(n, 1);

    newton_iterate(
        model,
        x,
        cfg,
        norm_kind,
        |model, x, f, dx| {
            model.jacobian_dense(x, &mut jac);
            lu.factor(&jac)?;
            for (i, &fi) in f.iter().enumerate() {
                rhs[(i, 0)] = -fi;
            }
            lu.solve_in_place(rhs.as_mut())?;

            // Copy back to dx.
            for (i, &val) in rhs.col(0).iter().enumerate() {
                dx[i] = val;
            }

            Ok(())
        },
        on_iter,
    )
}
