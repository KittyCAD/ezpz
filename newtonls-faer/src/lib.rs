#![doc = include_str!("../README.md")]
#![cfg_attr(docsrs, feature(doc_cfg))]
mod linalg;
mod solver;

pub use linalg::{DenseLu, FaerLu, SparseQr};
pub use solver::{
    Control, IterationStats, Iterations, MatrixFormat, NewtonCfg, solve, solve_cb, solve_dense_cb,
    solve_sparse_cb,
};

use core::fmt::{self, Display, Formatter};
use core::num::NonZeroUsize;
use faer::Mat;
use faer::mat::MatMut;
use faer::prelude::SparseColMatRef;
use faer::sparse::SymbolicSparseColMat;
use faer_traits::ComplexField;
use num_traits::Zero;
use std::sync::OnceLock;

pub trait RowMap {
    type Var: Copy + Eq;
    fn n_variables(&self) -> usize;
    fn n_residuals(&self) -> usize;
    fn row(&self, bus: usize, var: Self::Var) -> Option<usize>;
}

#[derive(Debug, Clone)]
pub struct Pattern<T> {
    pub symbolic: SymbolicSparseColMat<usize>,
    pub values: Vec<T>,
}

impl<T> Pattern<T> {
    #[inline]
    pub fn attach_values(&self) -> SparseColMatRef<'_, usize, T> {
        SparseColMatRef::new(self.symbolic.as_ref(), &self.values)
    }
    #[inline]
    pub fn values_mut(&mut self) -> &mut [T] {
        &mut self.values
    }
}

pub trait NonlinearSystem {
    type Real: num_traits::Float;
    type Layout: RowMap;

    fn layout(&self) -> &Self::Layout;
    fn jacobian(&self) -> &dyn JacobianCache<Self::Real>;
    fn jacobian_mut(&mut self) -> &mut dyn JacobianCache<Self::Real>;
    fn residual(&self, x: &[Self::Real], out: &mut [Self::Real]);
    fn refresh_jacobian(&mut self, x: &[Self::Real]);

    fn jacobian_dense(&mut self, x: &[Self::Real], jac: &mut faer::mat::Mat<Self::Real>) {
        self.refresh_jacobian(x);
        let sparse = self.jacobian().attach();
        jac.fill(Self::Real::zero());
        let row_idx = sparse.symbolic().row_idx();
        let vals = sparse.val();
        for col in 0..sparse.ncols() {
            let range = sparse.col_range(col);
            for idx in range.clone() {
                jac[(row_idx[idx], col)] = vals[idx];
            }
        }
    }
}

pub trait LinearSolver<T: ComplexField<Real = T>, M> {
    fn factor(&mut self, a: &M) -> SolverResult<()>;
    /// Solves in-place.
    /// - LU: overwrites `rhs` with the solution.
    /// - QR least-squares: writes the solution into the top ncols(A) rows of `rhs`.
    fn solve_in_place(&mut self, rhs: MatMut<T>) -> SolverResult<()>;
}

pub trait JacobianCache<T /* Real */> {
    fn symbolic(&self) -> &SymbolicSparseColMat<usize>;
    fn values(&self) -> &[T];
    fn values_mut(&mut self) -> &mut [T];
    #[inline]
    fn attach(&self) -> SparseColMatRef<'_, usize, T> {
        SparseColMatRef::new(self.symbolic().as_ref(), self.values())
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SolverError;

impl Display for SolverError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        f.write_str("solver error")
    }
}

impl std::error::Error for SolverError {}

pub type SolverResult<T> = Result<T, error_stack::Report<SolverError>>;

static RAYON_INIT: OnceLock<usize> = OnceLock::new();

pub fn init_global_parallelism(threads: usize) -> usize {
    if let Some(n) = RAYON_INIT.get().copied() {
        return n;
    }
    let target = if threads == 0 {
        std::thread::available_parallelism()
            .unwrap_or(unsafe { NonZeroUsize::new_unchecked(1) })
            .get()
    } else {
        threads
    };

    let _ = rayon::ThreadPoolBuilder::new()
        .num_threads(target)
        .build_global();

    let actual = rayon::current_num_threads();
    let _ = RAYON_INIT.set(actual);
    actual
}

#[inline]
pub fn current_parallelism() -> usize {
    RAYON_INIT
        .get()
        .copied()
        .unwrap_or_else(rayon::current_num_threads)
}

#[cfg(test)]
mod tests {
    use crate::solver::NormType;

    use super::*;
    use faer::sparse::Pair;
    use faer::sparse::SymbolicSparseColMat;

    #[derive(Clone)]
    struct TwoVarLayout;
    impl RowMap for TwoVarLayout {
        type Var = ();
        fn n_variables(&self) -> usize {
            2
        }
        fn n_residuals(&self) -> usize {
            2
        }
        fn row(&self, _bus: usize, _var: Self::Var) -> Option<usize> {
            None
        }
    }

    #[derive(Clone)]
    struct Jc {
        sym: SymbolicSparseColMat<usize>,
        vals: Vec<f64>,
    }
    impl JacobianCache<f64> for Jc {
        fn symbolic(&self) -> &SymbolicSparseColMat<usize> {
            &self.sym
        }
        fn values(&self) -> &[f64] {
            &self.vals
        }
        fn values_mut(&mut self) -> &mut [f64] {
            &mut self.vals
        }
    }

    struct Model {
        layout: TwoVarLayout,
        jac: Jc,
    }

    impl Model {
        fn new() -> Self {
            let pairs = vec![
                Pair { row: 0, col: 0 },
                Pair { row: 1, col: 0 },
                Pair { row: 0, col: 1 },
                Pair { row: 1, col: 1 },
            ];
            let (sym, _argsort) = SymbolicSparseColMat::try_new_from_indices(2, 2, &pairs).unwrap();
            let nnz = sym.col_ptr()[sym.ncols()];
            Self {
                layout: TwoVarLayout,
                jac: Jc {
                    sym,
                    vals: vec![0.0; nnz],
                },
            }
        }
    }

    impl NonlinearSystem for Model {
        type Real = f64;
        type Layout = TwoVarLayout;

        fn layout(&self) -> &Self::Layout {
            &self.layout
        }

        fn jacobian(&self) -> &dyn JacobianCache<Self::Real> {
            &self.jac
        }
        fn jacobian_mut(&mut self) -> &mut dyn JacobianCache<Self::Real> {
            &mut self.jac
        }

        fn residual(&self, x: &[Self::Real], out: &mut [Self::Real]) {
            let (xx, yy) = (x[0], x[1]);
            out[0] = xx + yy - 3.0;
            out[1] = xx * xx + yy - 3.0;
        }

        fn refresh_jacobian(&mut self, x: &[Self::Real]) {
            let xx = x[0];
            let v = self.jac.values_mut();
            v[0] = 1.0;
            v[1] = 2.0 * xx;
            v[2] = 1.0;
            v[3] = 1.0;
        }
    }

    #[test]
    fn solves_two_equations_sparse() {
        let cfg = NewtonCfg::<f64>::sparse()
            .with_adaptive(true)
            .with_threads(1);

        let mut model = Model::new();
        let mut x = [0.9_f64, 2.1_f64];

        let iters = crate::solve_sparse_cb(
            &mut model,
            &mut x,
            &mut crate::FaerLu::<f64>::default(),
            cfg,
            NormType::LInf,
            |_| Control::Continue,
        )
        .expect("solver");

        assert!(iters > 0 && iters <= 25);
        assert!((x[0] - 1.0).abs() < 1e-10);
        assert!((x[1] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn solves_non_square_system() {
        // A system with 2 variables and 3 residuals (overdetermined).
        struct NonSquareLayout;
        impl RowMap for NonSquareLayout {
            type Var = ();
            fn n_variables(&self) -> usize {
                2
            }
            fn n_residuals(&self) -> usize {
                3
            }
            fn row(&self, _bus: usize, _var: Self::Var) -> Option<usize> {
                None
            }
        }

        struct NonSquareModel {
            layout: NonSquareLayout,
            jac: Jc,
        }

        impl NonSquareModel {
            fn new() -> Self {
                // Jacobian pattern: 3 residuals x 2 variables
                let pairs = vec![
                    Pair { row: 0, col: 0 },
                    Pair { row: 0, col: 1 }, // First residual depends on both vars.
                    Pair { row: 1, col: 0 },
                    Pair { row: 1, col: 1 }, // Second residual depends on both vars.
                    Pair { row: 2, col: 0 },
                    Pair { row: 2, col: 1 }, // Third residual depends on both vars.
                ];
                let (sym, _argsort) =
                    SymbolicSparseColMat::try_new_from_indices(3, 2, &pairs).unwrap();
                let nnz = sym.col_ptr()[sym.ncols()];
                Self {
                    layout: NonSquareLayout,
                    jac: Jc {
                        sym,
                        vals: vec![0.0; nnz],
                    },
                }
            }
        }

        impl NonlinearSystem for NonSquareModel {
            type Real = f64;
            type Layout = NonSquareLayout;

            fn layout(&self) -> &Self::Layout {
                &self.layout
            }
            fn jacobian(&self) -> &dyn JacobianCache<Self::Real> {
                &self.jac
            }
            fn jacobian_mut(&mut self) -> &mut dyn JacobianCache<Self::Real> {
                &mut self.jac
            }
            fn residual(&self, x: &[Self::Real], out: &mut [Self::Real]) {
                let (xx, yy) = (x[0], x[1]);

                // Overdetermined system.
                // x + y = 3
                // x - y = 1
                // 2x + y = 5
                out[0] = xx + yy - 3.0;
                out[1] = xx - yy - 1.0;
                out[2] = 2.0 * xx + yy - 5.0;
            }
            fn refresh_jacobian(&mut self, _x: &[Self::Real]) {
                let v = self.jac.values_mut();
                // Jacobian entries in column-major order.
                // d(r0)/dx = 1
                // d(r1)/dx = 1
                // d(r2)/dx = 2
                // d(r0)/dy = 1
                // d(r1)/dy = -1
                // d(r2)/dy = 1

                v[0] = 1.0;
                v[1] = 1.0;
                v[2] = 2.0;
                v[3] = 1.0;
                v[4] = -1.0;
                v[5] = 1.0;
            }
        }

        let mut model = NonSquareModel::new();
        let mut x = [1.0_f64, 1.0_f64]; // Initial guess
        let cfg = NewtonCfg::<f64>::sparse().with_threads(1);

        let result = crate::solve(&mut model, &mut x, cfg);

        // The solver should now work with QR.
        assert!(result.is_ok());
        let iters = result.unwrap();
        assert!(iters > 0 && iters <= 25);

        // Check that we found a least-squares solution
        // The exact solution would be x=2, y=1 (satisfies first two equations exactly).
        let tol = 1e-6;
        assert!((x[0] - 2.0).abs() < tol);
        assert!((x[1] - 1.0).abs() < tol);
    }

    #[test]
    fn solves_gaussian_peak_fitting() {
        // Fit data to Gaussian: y = a * exp(-((x-mu)/sigma)^2)
        // 3 parameters for amplitude, mean and std dev (a, mu, sigma) with 5 data points (overdetermined).
        // https://en.wikipedia.org/wiki/Gaussian_function
        struct GaussianLayout;
        impl RowMap for GaussianLayout {
            type Var = ();
            fn n_variables(&self) -> usize {
                3
            }
            fn n_residuals(&self) -> usize {
                5
            }
            fn row(&self, _bus: usize, _var: Self::Var) -> Option<usize> {
                None
            }
        }

        struct GaussianModel {
            layout: GaussianLayout,
            jac: Jc,
            data: Vec<(f64, f64)>,
        }

        impl GaussianModel {
            fn new() -> Self {
                // Jacobian: 5 residuals x 3 variables, all related.
                let pairs = vec![
                    Pair { row: 0, col: 0 },
                    Pair { row: 0, col: 1 },
                    Pair { row: 0, col: 2 },
                    Pair { row: 1, col: 0 },
                    Pair { row: 1, col: 1 },
                    Pair { row: 1, col: 2 },
                    Pair { row: 2, col: 0 },
                    Pair { row: 2, col: 1 },
                    Pair { row: 2, col: 2 },
                    Pair { row: 3, col: 0 },
                    Pair { row: 3, col: 1 },
                    Pair { row: 3, col: 2 },
                    Pair { row: 4, col: 0 },
                    Pair { row: 4, col: 1 },
                    Pair { row: 4, col: 2 },
                ];
                let (sym, _argsort) =
                    SymbolicSparseColMat::try_new_from_indices(5, 3, &pairs).unwrap();
                let nnz = sym.col_ptr()[sym.ncols()];

                // Data generated from y = 2.0 * exp(-((x-1.0)/0.8)^2).
                let x_vals = [-1.0, 0.0, 1.0, 2.0, 2.5];
                let a = 2.0;
                let mu = 1.0;
                let sigma = 0.8;

                let data: Vec<(f64, f64)> = x_vals
                    .iter()
                    .map(|x: &f64| {
                        let y = a * (-((x - mu) / sigma).powi(2)).exp();
                        (*x, y)
                    })
                    .collect();

                Self {
                    layout: GaussianLayout,
                    jac: Jc {
                        sym,
                        vals: vec![0.0; nnz],
                    },
                    data,
                }
            }
        }

        impl NonlinearSystem for GaussianModel {
            type Real = f64;
            type Layout = GaussianLayout;

            fn layout(&self) -> &Self::Layout {
                &self.layout
            }
            fn jacobian(&self) -> &dyn JacobianCache<Self::Real> {
                &self.jac
            }
            fn jacobian_mut(&mut self) -> &mut dyn JacobianCache<Self::Real> {
                &mut self.jac
            }

            fn residual(&self, x: &[Self::Real], out: &mut [Self::Real]) {
                let (a, mu, sigma) = (x[0], x[1], x[2]);

                for (i, &(xi, yi)) in self.data.iter().enumerate() {
                    let z = (xi - mu) / sigma;
                    let gaussian = a * (-z * z).exp();
                    out[i] = gaussian - yi;
                }
            }

            fn refresh_jacobian(&mut self, x: &[Self::Real]) {
                let (a, mu, sigma) = (x[0], x[1], x[2]);
                let v = self.jac.values_mut();

                for (i, &(xi, _)) in self.data.iter().enumerate() {
                    let z = (xi - mu) / sigma;
                    let exp_term = (-z * z).exp();
                    let gaussian = a * exp_term;
                    let n_eqn = 5;

                    // dr/da = exp(-z^2)
                    v[i] = exp_term;

                    // dr/dmu = a * exp(-z^2) * 2z/sigma = gaussian * 2(xi-mu)/sigma^2
                    v[i + n_eqn] = gaussian * 2.0 * (xi - mu) / (sigma * sigma);

                    // dr/dsigma = a * exp(-z^2) * 2z^2/sigma = gaussian * 2(xi-mu)^2/sigma^3
                    v[i + n_eqn * 2] =
                        gaussian * 2.0 * (xi - mu) * (xi - mu) / (sigma * sigma * sigma);
                }
            }
        }

        let mut model = GaussianModel::new();

        // Initial guess: amplitude, mean, std_dev.
        let mut x = [1.8_f64, 0.5_f64, 1.2_f64];
        let cfg = NewtonCfg::<f64>::sparse()
            .with_adaptive(true)
            .with_threads(1);

        let callback = |stats: &IterationStats<f64>| {
            println!(
                "Iter: {:>2}, Residual: {:.4e}, Damping: {:.4}",
                stats.iter, stats.residual, stats.damping
            );
            Control::Continue
        };

        // Use callback version for reporting.
        let result = crate::solve_cb(&mut model, &mut x, cfg, callback);

        // The solver should converge to the true parameters.
        assert!(result.is_ok());
        let iters = result.unwrap();
        assert!(iters > 0 && iters <= 50);

        // Check that we recovered the original Gaussian parameters:
        // True values: a=2.0, mu=1.0, sigma=0.8.
        println!(
            "Fitted parameters: a={:.4}, mu={:.4}, sigma={:.4}",
            x[0], x[1], x[2]
        );

        let tol = 1e-6;

        assert!(
            (x[0] - 2.0).abs() < tol,
            "Amplitude should be 2.0, got {}",
            x[0]
        );
        assert!((x[1] - 1.0).abs() < tol, "Mean should be 1.0, got {}", x[1]);
        assert!(
            (x[2] - 0.8).abs() < tol,
            "Std dev should be 0.8, got {}",
            x[2]
        );
    }

    #[test]
    fn solves_simple_circle_line_system() {
        // Equivalent test exists in geometric constraint playground.
        // System: x^2 + y^2 = 1 (circle), x - y = 0 (line)
        struct SimpleLayout;
        impl RowMap for SimpleLayout {
            type Var = ();
            fn n_variables(&self) -> usize {
                2
            }
            fn n_residuals(&self) -> usize {
                2
            }
            fn row(&self, _bus: usize, _var: Self::Var) -> Option<usize> {
                None
            }
        }

        struct SimpleSystem {
            layout: SimpleLayout,
            jac: Jc,
        }

        impl SimpleSystem {
            fn new() -> Self {
                let pairs = vec![
                    faer::sparse::Pair { row: 0, col: 0 },
                    faer::sparse::Pair { row: 1, col: 0 },
                    faer::sparse::Pair { row: 0, col: 1 },
                    faer::sparse::Pair { row: 1, col: 1 },
                ];
                let (sym, _) = SymbolicSparseColMat::try_new_from_indices(2, 2, &pairs).unwrap();
                let nnz = sym.col_ptr()[sym.ncols()];
                Self {
                    layout: SimpleLayout,
                    jac: Jc {
                        sym,
                        vals: vec![0.0; nnz],
                    },
                }
            }
        }

        impl NonlinearSystem for SimpleSystem {
            type Real = f64;
            type Layout = SimpleLayout;

            fn layout(&self) -> &Self::Layout {
                &self.layout
            }
            fn jacobian(&self) -> &dyn JacobianCache<Self::Real> {
                &self.jac
            }
            fn jacobian_mut(&mut self) -> &mut dyn JacobianCache<Self::Real> {
                &mut self.jac
            }
            fn residual(&self, x: &[Self::Real], out: &mut [Self::Real]) {
                out[0] = x[0] * x[0] + x[1] * x[1] - 1.0;
                out[1] = x[0] - x[1];
            }
            fn refresh_jacobian(&mut self, x: &[Self::Real]) {
                let v = self.jac.values_mut();
                // d(r0)/dx = 2x
                // d(r1)/dx = 1
                // d(r0)/dy = 2y
                // d(r1)/dy = -1
                v[0] = 2.0 * x[0];
                v[1] = 1.0;
                v[2] = 2.0 * x[1];
                v[3] = -1.0;
            }
        }

        let mut system = SimpleSystem::new();
        let mut x = [0.5_f64, 0.5_f64];

        let cfg = NewtonCfg::<f64>::sparse()
            .with_adaptive(true)
            .with_threads(1);

        let callback = |stats: &IterationStats<f64>| {
            println!(
                "Iteration {}: residual = {:.2e}, damping = {:.3}",
                stats.iter, stats.residual, stats.damping
            );
            Control::Continue
        };

        let result = crate::solve_cb(&mut system, &mut x, cfg, callback);

        assert!(result.is_ok(), "Solver failed: {:?}", result);
        let iters = result.unwrap();
        assert!(iters > 0 && iters <= 25);

        // The solution should be close to [0.7071, 0.7071].
        let expected = std::f64::consts::FRAC_1_SQRT_2;
        let tol = 1e-8;
        assert!(
            (x[0] - expected).abs() < tol,
            "x[0] = {}, expected {}",
            x[0],
            expected
        );
        assert!(
            (x[1] - expected).abs() < tol,
            "x[1] = {}, expected {}",
            x[1],
            expected
        );

        // Print final result and residuals.
        println!("Converged in {} iterations", iters);
        println!("Solution: x = {:?}", x);
        let mut res = [0.0; 2];
        system.residual(&x, &mut res);
        println!("Residual: {:?}", res);
    }

    #[test]
    fn solves_inconsistent_system_to_least_squares() {
        // Inconsistent system:
        //   r0 = x^2 + y^2 - 1          (unit circle)
        //   r1 = x - y                  (x and y equal)
        //   r2 = x + y - 2              (sum equals 2)
        //
        // No exact solution exists. The least-squares minimiser satisfies.
        //   J^T r = 0  =>  x = y = (1/2)^(1/3) ≈ 0.793700526.
        struct Layout;
        impl RowMap for Layout {
            type Var = ();
            fn n_variables(&self) -> usize {
                2
            }
            fn n_residuals(&self) -> usize {
                3
            }
            fn row(&self, _bus: usize, _var: Self::Var) -> Option<usize> {
                None
            }
        }

        #[derive(Clone)]
        struct Jc {
            sym: SymbolicSparseColMat<usize>,
            vals: Vec<f64>,
        }
        impl JacobianCache<f64> for Jc {
            fn symbolic(&self) -> &SymbolicSparseColMat<usize> {
                &self.sym
            }
            fn values(&self) -> &[f64] {
                &self.vals
            }
            fn values_mut(&mut self) -> &mut [f64] {
                &mut self.vals
            }
        }

        struct InconsistentSystem {
            layout: Layout,
            jac: Jc,
        }

        impl InconsistentSystem {
            fn new() -> Self {
                // Full 3x2 Jacobian sparsity, column-major (rows 0..2 for each col)
                let pairs = vec![
                    faer::sparse::Pair { row: 0, col: 0 },
                    faer::sparse::Pair { row: 1, col: 0 },
                    faer::sparse::Pair { row: 2, col: 0 },
                    faer::sparse::Pair { row: 0, col: 1 },
                    faer::sparse::Pair { row: 1, col: 1 },
                    faer::sparse::Pair { row: 2, col: 1 },
                ];
                let (sym, _) = SymbolicSparseColMat::try_new_from_indices(3, 2, &pairs).unwrap();
                let nnz = sym.col_ptr()[sym.ncols()];
                Self {
                    layout: Layout,
                    jac: Jc {
                        sym,
                        vals: vec![0.0; nnz],
                    },
                }
            }
        }

        impl NonlinearSystem for InconsistentSystem {
            type Real = f64;
            type Layout = Layout;

            fn layout(&self) -> &Self::Layout {
                &self.layout
            }
            fn jacobian(&self) -> &dyn JacobianCache<Self::Real> {
                &self.jac
            }
            fn jacobian_mut(&mut self) -> &mut dyn JacobianCache<Self::Real> {
                &mut self.jac
            }

            fn residual(&self, x: &[Self::Real], out: &mut [Self::Real]) {
                // r0 = x^2 + y^2 - 1
                // r1 = x - y
                // r2 = x + y - 2
                out[0] = x[0] * x[0] + x[1] * x[1] - 1.0;
                out[1] = x[0] - x[1];
                out[2] = x[0] + x[1] - 2.0;
            }

            fn refresh_jacobian(&mut self, x: &[Self::Real]) {
                // Column 0 (x): [dr0/dx, dr1/dx, dr2/dx] = [2x, 1, 1]
                // Column 1 (y): [dr0/dy, dr1/dy, dr2/dy] = [2y, -1, 1]

                let v = self.jac.values_mut();
                v[0] = 2.0 * x[0];
                v[1] = 1.0; // (row 1, col 0)
                v[2] = 1.0; // (row 2, col 0)
                v[3] = 2.0 * x[1]; // (row 0, col 1)
                v[4] = -1.0; // (row 1, col 1)
                v[5] = 1.0; // (row 2, col 1)
            }
        }

        let mut system = InconsistentSystem::new();
        let mut x = [0.5_f64, 0.5_f64]; // Initial guess

        let cfg = NewtonCfg::<f64>::sparse()
            .with_adaptive(true)
            .with_tol_grad(1e-8)
            .with_tol_step(0.0) // Disable step tolerance to focus on gradient tolerance.
            .with_threads(1);

        let callback = |stats: &IterationStats<f64>| {
            println!(
                "Iteration {}: residual = {:.2e}, damping = {:.3}",
                stats.iter, stats.residual, stats.damping
            );
            Control::Continue
        };

        let result = crate::solve_cb(&mut system, &mut x, cfg, callback);

        assert!(result.is_ok(), "Solver failed: {:?}", result);
        let iters = result.unwrap();
        assert!(
            iters > 0 && iters <= 50,
            "Unexpected iteration count: {}",
            iters
        );

        // Expected least-squares solution: x = y = (1/2)^(1/3)
        let expected = 0.5_f64.powf(1.0 / 3.0);
        let tol = 1e-6;

        assert!(
            (x[0] - expected).abs() < tol,
            "x[0] = {}, expected {}",
            x[0],
            expected
        );
        assert!(
            (x[1] - expected).abs() < tol,
            "x[1] = {}, expected {}",
            x[1],
            expected
        );
        assert!(
            (x[0] - x[1]).abs() < 1e-8,
            "x and y should be essentially equal"
        );

        // Verify we reached a (near) least-squares stationary point: J^T r ~= 0.
        let mut r = [0.0; 3];
        system.residual(&x, &mut r);
        // J^T r components for this problem:
        let g_x = 2.0 * x[0] * r[0] + r[1] + r[2];
        let g_y = 2.0 * x[1] * r[0] - r[1] + r[2];
        let grad_norm = (g_x * g_x + g_y * g_y).sqrt();
        assert!(
            grad_norm < 1e-8,
            "Not at a least-squares stationary point: ||J^T r|| = {}",
            grad_norm
        );

        // Print final result and residuals / SSE.
        println!("Converged in {} iterations", iters);
        println!("Solution (least squares): x = {:?}", x);
        println!("Residuals: {:?}", r);
        let sse = r.iter().map(|ri| ri * ri).sum::<f64>();
        println!("Sum of squared residuals: {:.8}", sse);
    }
}
