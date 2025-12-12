//! Finding degrees of freedom and assessing which variables are underconstrained.
use faer::sparse::SparseColMatRef;

use crate::{FreedomAnalysis, NonLinearSystemError, solver::Model};

impl Model<'_> {
    pub fn freedom_analysis(&self) -> Result<FreedomAnalysis, NonLinearSystemError> {
        // First step is to compute the SVD.
        // Faer has a sparse SVD algorithm called `partial_svd`, but I haven't been
        // able to get it working properly yet.
        // For now, we'll just use a dense SVD algorithm.
        // This is VERY SLOW for large matrices.
        let j_sparse = SparseColMatRef::new(self.jc.sym.as_ref(), &self.jc.vals);
        let j_dense = j_sparse.to_dense();
        let nvars = self.layout.num_variables;
        debug_assert_eq!(
            nvars,
            j_dense.ncols(),
            "Jacobian was malformed, Adam messed something up here."
        );

        // SVD decomposes `J` into `J = UΣVᵀ`.
        let svd = j_dense.svd().map_err(NonLinearSystemError::FaerSvd)?;
        let sigma_diags = svd.S();

        // These are the 'singular values'.
        let sigma_col = sigma_diags.column_vector();
        let (m, n) = (j_dense.nrows(), j_dense.ncols());

        // The system is underconstrained if there's too many singular values
        // close to 0. How close to 0? The tolerance should be derived from
        // the largest singular value, scaled by machine epsilon and matrix size,
        // mirroring LAPACK's recommended rank-revealing cutoff.
        let largest_singular_value = sigma_col
            .iter()
            .copied()
            .reduce(libm::fmax)
            .ok_or(NonLinearSystemError::EmptySystemNotAllowed)?;
        let tolerance = f64::EPSILON * (m.max(n) as f64) * largest_singular_value;

        let rank = sigma_col.iter().filter(|&&s| s > tolerance).count();

        // The degrees of freedom = nvars - rank;
        // The rank is a measure of how sensitive the Jacobian is in each direction.
        // If there's any direction where the Jacobian is sensitive, then tweaking the values
        // in that dimension will change the result. This is what we'd expect in a well-constrained system.
        // On the other hand, if the Jacobian DOESN'T change along one direction, that implies the direction
        // doesn't affect the residual at all. That's basically exactly what a degree of freedom means.

        // Compute participation norm for each variable.
        // If a variable's participation is basically zero, then it's constrained.
        // If it's nonzero, then it moves in some DOF and is unconstrained.
        let participation: Vec<_> = (0..nvars)
            .map(|j| {
                let mut sum_sq = 0.0;

                for k in rank..nvars {
                    // V[j, k] is the component of variable j for the k-th DOF.
                    let v_jk = svd.V().get(j, k);
                    sum_sq += v_jk * v_jk;
                }
                sum_sq.sqrt()
            })
            .collect();
        let max_participation = participation.iter().cloned().fold(0.0, libm::fmax);

        // Relative threshold to classify variables; also guard with an absolute floor tied to
        // numerical noise so tiny leakage from near-null directions doesn't mark a variable.
        let noise_floor = 10.0 * libm::sqrt(nvars as f64) * f64::EPSILON;
        let var_tol = libm::fmax(1e-3 * max_participation, noise_floor);

        let underconstrained: Vec<crate::Id> = (0..nvars)
            .filter(|&j| participation[j] > var_tol)
            .map(|x| x as u32)
            .collect();

        Ok(FreedomAnalysis { underconstrained })
    }
}
