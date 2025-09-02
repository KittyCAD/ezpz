# newton-faer

## Why another Newton solver?
Most Rust numerical computing is still young. The existing options:
- roots — great for 1D, but not systems
- argmin — focused on optimization (line searches, trust regions), not sparse Newton for general nonlinear systems
- nalgebra — has solvers, but not a sparse-aware Newton–Raphson with reusable symbolic factorizations

This crate is a thin, reusable Newton core that leans on faer for world-class sparse linear algebra, while keeping all domain logic (residual/Jacobian) outside the engine. You get:
- Separation of concerns: engine = iteration policy + linear solves; models = residual/Jacobian.
- Sparse-first: symbolic LU reused across solves when the sparsity pattern is unchanged.
- Production knobs: adaptive damping, divergence guard + backtracking, cancellation, and progress callbacks.
- Parallelism control: slam all cores for big single cases, or run many small cases in parallel with single-threaded LU (no oversubscription).

Architecture at a glance
- linalg: adapters over faer (sparse LU) and dense LU; keeps symbolic factorization cached per pattern.
- solver: Newton loop, damping policy, divergence guard, callbacks, and thread init.

Your model implements NonlinearSystem (residual + Jacobian/refresh), optionally with its own Jacobian cache.

## Interface split: model ↔ engine (sparse-first)
The engine doesn’t know your math. You provide two tiny pieces:

1/ NonlinearSystem (your model)
- layout() → problem size / indexing (RowMap)
- residual(x, out) → compute 𝐹(𝑥)
- refresh_jacobian(x) → update values of a cached sparse Jacobian
- jacobian() / jacobian_mut() → hand back the cache handle

2/ JacobianCache (your storage)
- Owns the symbolic pattern (SymbolicSparseColMat) once
- Exposes a mutable values slice each iteration
- Engine calls attach() to get a SparseColMatRef for factorization

```rust
use newton_faer::{NonlinearSystem, RowMap, JacobianCache, NewtonCfg, solve};
use faer::sparse::{SymbolicSparseColMat, Pair};

struct Layout;
impl RowMap for Layout {
    type Var = ();
    fn dim(&self) -> usize { 2 }
    fn row(&self, _i: usize, _v: ()) -> Option<usize> { None }
}

struct Jc { sym: SymbolicSparseColMat<usize>, vals: Vec<f64> }
impl JacobianCache<f64> for Jc {
    fn symbolic(&self) -> &SymbolicSparseColMat<usize> { &self.sym }
    fn values(&self) -> &[f64] { &self.vals }
    fn values_mut(&mut self) -> &mut [f64] { &mut self.vals }
}

struct Model { lay: Layout, jac: Jc }

impl Model {
    fn new() -> Self {
        let pairs = [
            Pair{row:0, col:0}, Pair{row:1, col:0},
            Pair{row:0, col:1}, Pair{row:1, col:1},
        ];
        let (sym, _) = SymbolicSparseColMat::try_new_from_indices(2, 2, &pairs).unwrap();
        let nnz = sym.col_ptr()[sym.ncols()];
        Self { lay: Layout, jac: Jc { sym, vals: vec![0.0; nnz] } }
    }
}

impl NonlinearSystem for Model {
    type Real = f64;
    type Layout = Layout;

    fn layout(&self) -> &Self::Layout { &self.lay }
    fn jacobian(&self) -> &dyn JacobianCache<Self::Real> { &self.jac }
    fn jacobian_mut(&mut self) -> &mut dyn JacobianCache<Self::Real> { &mut self.jac }

    fn residual(&self, x: &[f64], out: &mut [f64]) {
        // f1 = sin(x) + y
        // f2 = x + exp(y) - 1
        out[0] = x[0].sin() + x[1];
        out[1] = x[0] + x[1].exp() - 1.0;
    }

    fn refresh_jacobian(&mut self, x: &[f64]) {
        // J = [[cos(x), 1],
        //      [     1, exp(y)]] in CSC (col-major): (0,0),(1,0),(0,1),(1,1)
        let v = self.jac.values_mut();
        v[0] = x[0].cos();
        v[1] = 1.0;
        v[2] = 1.0;
        v[3] = x[1].exp();
    }
}

// usage
fn main() -> Result<(), Box<dyn std::error::Error>> {
    newton_faer::init_global_parallelism(0); // use all cores
    let mut model = Model::new();
    let mut x = [0.2, 0.0]; // initial guess
    let iters = solve(&mut model, &mut x, NewtonCfg::sparse().with_adaptive(true))?;
    println!("iters={iters}, x={:.6}, y={:.6}", x[0], x[1]);
    Ok(())
}
```


## Parallelism

Parallelism is a policy choice: inside LU for one/few huge systems, or over cases for large batches. We auto-init Rayon once; you can override the global thread count via config.

```rust
use rayon::prelude::*;
use newton_faer::NewtonCfg;

// batch mode: many cases, maximize throughput
let cfg_batch = NewtonCfg::<f64>::default().with_threads(1); // LU single-thread
let results: Vec<_> = contingencies
    .par_iter()
    .map(|&br| {
        let mut pf = base_pf.clone();
        pf.update_for_outage(br)?;
        pf.solve_with_cfg(cfg_batch) // your wrapper that passes cfg in
    })
    .collect();

// big single case mode: few huge systems
let cfg_big = NewtonCfg::<f64>::default().with_threads(0); // use all cores in LU
let mut pf = big_system_pf.clone();
let res = pf.solve_with_cfg(cfg_big)?;
```
Guideline:
- Tons of scenarios? with_threads(1) + par_iter() over cases.
- One gigantic system? with_threads(0) (all cores) + sequential cases.


## Adaptive Damping

Divergence guard + backtracking are enabled when adaptive=true and steered by:
min_damping, max_damping, grow, shrink, divergence_ratio, ls_backtrack, ls_max_steps.

```rust
let cfg = NewtonCfg::default().with_adaptive(true);
```


## Reuse LU

Keep a solver instance and reuse the symbolic factorization across solves with the same sparsity. Only the numeric phase is recomputed.

```rust
let mut lu = FaerLu::<f64>::default();
for case in cases {
    let mut x = init_state(&case);
    let _iters = solver::solve_sparse_cb(&mut model, &mut x, &mut lu, cfg, |_| Control::Continue)?;
}
```

## Solver Progress

Stream iteration stats to your UI, support cancellation, and run the solve on a worker thread.

```rust
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, mpsc,
};
use newton_faer::{Control, IterationStats};

let (tx, rx) = mpsc::channel::<IterationStats<f64>>();
let cancel = Arc::new(AtomicBool::new(false));
let cancel_flag = cancel.clone();

let mut model = /* build system */;
std::thread::spawn(move || {
    let _ = solver::solve_cb(&mut model, &mut x, cfg.with_adaptive(true), |st| {
        let _ = tx.send(st.clone());
        if cancel_flag.load(Ordering::Relaxed) { Control::Cancel } else { Control::Continue }
    });
});

while let Ok(st) = rx.recv() {
    println!("iter={} residual={:.3e} damping={:.2}", st.iter, st.residual, st.damping);
    // if user hits "Stop": cancel.store(true, Ordering::Relaxed);
}
```

## Performance notes
- Symbolic reuse is the big win in multi-scenario studies (fixed sparsity).
- Preallocated buffers and in-place solves avoid per-iteration allocations.
- Threading: pick one level of parallelism—inside LU or over cases—to avoid oversubscription.
- Ordering (AMD/ND/…) can drastically reduce fill; we expose knobs for faer’s symbolic parameters if you want to tune.

## Acknowledgments
Big thanks to the [faer] team. newton-faer leans on faer’s fast, well-designed sparse linear algebra.

[faer]: https://github.com/sarah-quinones/faer-rs
