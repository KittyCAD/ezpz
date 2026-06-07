use faer::{
    ColRef,
    prelude::Solve,
    sparse::{SparseColMatRef, linalg::solvers::Lu},
};

use crate::{Config, NonLinearSystemError};

use super::Model;

#[derive(Debug)]
pub struct SuccessfulSolve {
    /// How many iterations did the solver run for?
    pub iterations: usize,
    /// Did it ultimately converge, or not?
    pub converged: bool,
    /// Estimated 2-norm condition number of the linear system solved at each
    /// iteration, in iteration order. Empty unless
    /// [`crate::Config::with_condition_number_estimates`] was enabled.
    pub condition_numbers: Vec<f64>,
}

impl Model<'_> {
    #[inline(never)]
    pub(crate) fn solve_gauss_newton(
        &mut self,
        current_values: &mut [f64],
        config: Config,
    ) -> Result<SuccessfulSolve, NonLinearSystemError> {
        let m = self.layout.total_num_residuals;

        let mut global_residual = vec![0.0; m];

        // Per-iteration condition-number diagnostics (opt-in via Config).
        let mut condition_numbers = if config.estimate_condition_number {
            Vec::with_capacity(config.max_iterations)
        } else {
            Vec::new()
        };

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
            if largest_absolute_elem <= config.convergence_tolerance {
                return Ok(SuccessfulSolve {
                    iterations: this_iteration,
                    converged: true,
                    condition_numbers,
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
            if config.estimate_condition_number {
                condition_numbers.push(estimate_spd_condition_number(
                    a.as_ref(),
                    &factored,
                    current_values.len(),
                ));
            }
            let d = factored.solve(&b);
            assert_eq!(
                d.nrows(),
                current_values.len(),
                "the `d` column must be the same size as the number of variables."
            );
            let current_inf_norm = current_values.iter().map(|v| v.abs()).fold(0.0, libm::fmax);
            let step_inf_norm = d.iter().map(|d| d.abs()).reduce(libm::fmax).unwrap_or(0.0);
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
                    converged: true,
                    condition_numbers,
                });
            }
        }
        Ok(SuccessfulSolve {
            iterations: config.max_iterations,
            converged: false,
            condition_numbers,
        })
    }
}

/// Estimate the 2-norm condition number of the SPD normal-equations matrix
/// `A = JᵀJ + λI`, reusing the LU factorization computed for the Newton step.
///
/// For an SPD matrix the condition number is `λ_max / λ_min`. We get the two
/// extreme eigenvalues with a few rounds of power iteration: directly on `A`
/// for `λ_max`, and on `A⁻¹` for `λ_min` (`λ_min = 1 / λ_max(A⁻¹)`). The
/// inverse step just applies the existing LU factorization, so it costs a
/// couple of triangular solves rather than a fresh factorization.
///
/// `A = JᵀJ`, so this is the square of the Jacobian's condition number; take
/// the square root if you want `cond(J)`.
///
/// Returns infinity if `A` is numerically singular, and `1.0` for an empty
/// system.
fn estimate_spd_condition_number(
    a: SparseColMatRef<'_, usize, f64>,
    factored: &Lu<usize, f64>,
    n: usize,
) -> f64 {
    // Number of power-iteration rounds. A dozen is plenty for an order-of-
    // magnitude diagnostic; this is not a high-precision eigensolve.
    const POWER_ITERS: usize = 12;
    if n == 0 {
        return 1.0;
    }

    // Reusable buffers so the per-round work allocates nothing.
    let mut v = vec![0.0; n];
    let mut work = vec![0.0; n];

    // λ_max(A): apply A directly.
    let lambda_max = power_iteration(POWER_ITERS, &mut v, &mut work, |x, out| {
        let prod = a * ColRef::from_slice(x);
        out.iter_mut().zip(prod.iter()).for_each(|(o, p)| *o = *p);
    });

    // λ_max(A⁻¹) = 1 / λ_min(A): apply A⁻¹ via the existing factorization.
    let lambda_max_inv = power_iteration(POWER_ITERS, &mut v, &mut work, |x, out| {
        let prod = factored.solve(ColRef::from_slice(x));
        out.iter_mut().zip(prod.iter()).for_each(|(o, p)| *o = *p);
    });

    if lambda_max_inv <= f64::EPSILON || !lambda_max_inv.is_finite() {
        return f64::INFINITY;
    }
    let lambda_min = 1.0 / lambda_max_inv;
    if lambda_min <= f64::EPSILON {
        return f64::INFINITY;
    }
    lambda_max / lambda_min
}

/// Power iteration for the dominant eigenvalue magnitude of an SPD operator,
/// where `apply` computes `out = M x` for the operator `M`.
///
/// `v` and `work` are caller-owned scratch buffers (length = operator
/// dimension) and are both overwritten. The start vector is uniform so the
/// result is deterministic. Returns the dominant eigenvalue magnitude, or `0.0`
/// if the operator sends the iterate to (numerically) zero.
fn power_iteration(
    iters: usize,
    v: &mut [f64],
    work: &mut [f64],
    mut apply: impl FnMut(&[f64], &mut [f64]),
) -> f64 {
    let n = v.len();
    // Deterministic, non-degenerate start: the uniform unit vector.
    let start = 1.0 / (n as f64).sqrt();
    v.iter_mut().for_each(|x| *x = start);

    let mut lambda = 0.0;
    for _ in 0..iters {
        apply(v, work);
        let norm = work.iter().map(|x| x * x).sum::<f64>().sqrt();
        if norm <= f64::EPSILON {
            return 0.0;
        }
        lambda = norm;
        let inv_norm = 1.0 / norm;
        v.iter_mut()
            .zip(work.iter())
            .for_each(|(vi, &wi)| *vi = wi * inv_norm);
    }
    lambda
}

#[cfg(test)]
mod cond_number_tests {
    use super::{estimate_spd_condition_number, power_iteration};
    use faer::sparse::{
        SparseColMat, Triplet,
        linalg::solvers::{Lu, SymbolicLu},
    };

    /// Build an SPD matrix from explicit entries and run the estimator on it,
    /// going through the same LU path the solver uses.
    fn estimate(entries: &[(usize, usize, f64)], n: usize) -> f64 {
        let triplets: Vec<_> = entries
            .iter()
            .map(|&(i, j, v)| Triplet::new(i, j, v))
            .collect();
        let a = SparseColMat::<usize, f64>::try_new_from_triplets(n, n, &triplets).unwrap();
        let symbolic = SymbolicLu::try_new(a.symbolic()).unwrap();
        let factored = Lu::try_new_with_symbolic(symbolic, a.as_ref()).unwrap();
        estimate_spd_condition_number(a.as_ref(), &factored, n)
    }

    #[test]
    fn recovers_known_condition_numbers() {
        // Identity: every singular value is 1, so the condition number is 1.
        let id = estimate(&[(0, 0, 1.0), (1, 1, 1.0), (2, 2, 1.0)], 3);
        assert!((id - 1.0).abs() < 1e-6, "identity κ should be 1, got {id}");

        // Diagonal diag(1, 8, 64, 512): the eigenvalues are the diagonal, so the
        // condition number is 512 / 1 = 512.
        let diag = estimate(&[(0, 0, 1.0), (1, 1, 8.0), (2, 2, 64.0), (3, 3, 512.0)], 4);
        assert!(
            (diag - 512.0).abs() / 512.0 < 1e-3,
            "diagonal κ should be 512, got {diag}"
        );

        // A dense (non-diagonal) SPD matrix with a known condition number:
        // rotate diag(1, 1000) by θ = 0.4 rad. Orthogonal similarity leaves the
        // eigenvalues at 1 and 1000 (κ = 1000) but fills in every entry.
        //   A = Rᵀ diag(1, 1000) R,  R = [[c, -s], [s, c]]
        let c = 0.921_060_994_002_885_1_f64; // cos(0.4)
        let s = 0.389_418_342_308_650_5_f64; // sin(0.4)
        let (d0, d1) = (1.0_f64, 1000.0_f64);
        let a00 = c * c * d0 + s * s * d1;
        let a11 = s * s * d0 + c * c * d1;
        let a01 = c * s * (d1 - d0);
        let rotated = estimate(&[(0, 0, a00), (0, 1, a01), (1, 0, a01), (1, 1, a11)], 2);
        assert!(
            (rotated - 1000.0).abs() / 1000.0 < 1e-2,
            "rotated κ should be 1000, got {rotated}"
        );
    }

    #[test]
    fn power_iteration_recovers_dominant_eigenvalue() {
        // Applying diag(2, 5, 9) is per-component scaling; the dominant
        // eigenvalue is 9.
        let diag = [2.0_f64, 5.0, 9.0];
        let mut v = vec![0.0; 3];
        let mut work = vec![0.0; 3];
        let lambda = power_iteration(50, &mut v, &mut work, |x, out| {
            out.iter_mut()
                .zip(diag.iter())
                .zip(x.iter())
                .for_each(|((o, &d), &xi)| *o = d * xi);
        });
        assert!(
            (lambda - 9.0).abs() < 1e-9,
            "dominant eigenvalue should be 9, got {lambda}"
        );
    }
}
