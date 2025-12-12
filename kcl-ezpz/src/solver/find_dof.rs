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
        debug_assert_eq!(
            self.layout.num_variables,
            j_dense.ncols(),
            "Jacobian was malformed, Adam messed something up here."
        );

        // SVD decomposes `J` into `J = UΣVᵀ`.
        let svd = j_dense.svd().map_err(NonLinearSystemError::FaerSvd)?;
        let sigma_diags = svd.S();

        // These are the 'singular values'.
        let sigma_col = sigma_diags.column_vector();

        // The system is underconstrained if there's too many singular values
        // close to 0. How close to 0? The tolerance should be derived from
        // the largest singular value.
        let largest_singular_value = sigma_col
            .iter()
            .copied()
            .reduce(libm::fmax)
            .ok_or(NonLinearSystemError::EmptySystemNotAllowed)?;
        let tolerance = 1e-8 * largest_singular_value;

        let rank = sigma_col.iter().filter(|&&s| s > tolerance).count();
        let degrees_of_freedom = self.layout.num_variables - rank;
        let is_underconstrained = degrees_of_freedom > 0;

        // Early return if the system is fully constrained,
        // i.e. has no underconstrained variables.
        if !is_underconstrained {
            return Ok(FreedomAnalysis {
                underconstrained: Vec::new(),
            });
        }

        // 3. Degrees of freedom
        let nvars = self.layout.num_variables;
        // The degrees of freedom = nvars - rank;
        // The rank is a measure of how sensitive the Jacobian is in each direction.
        // If there's any direction where the Jacobian is sensitive, then tweaking the values
        // in that dimension will change the result. This is what we'd expect in a well-constrained system.
        // On the other hand, if the Jacobian DOESN'T change along one direction, that implies the direction
        // doesn't affect the residual at all. That's basically exactly what a degree of freedom means.

        // Nullspace column indices in V, as in J = U.sigma.V in the SVD decomposition.
        let degrees_of_freedom: Vec<usize> = (rank..nvars).collect();

        // Compute participation norm for each variable.
        // If a variable's participation is basically zero, then it's constrained.
        // If it's nonzero, then it moves in some DOF and is unconstrained.
        let mut participation = Vec::with_capacity(nvars);
        for j in 0..nvars {
            let mut sum_sq = 0.0;

            for &k in &degrees_of_freedom {
                // V[j, k] is the component of variable j for the k-th DOF.
                let v_jk = svd.V().get(j, k);
                sum_sq += v_jk * v_jk;
            }
            participation.push(sum_sq.sqrt());
        }
        let max_participation = participation.iter().cloned().fold(0.0, libm::fmax);

        // Relative threshold to classify variables
        let var_tol = 1e-3 * max_participation;

        let underconstrained: Vec<crate::Id> = (0..nvars)
            .filter(|&j| participation[j] > var_tol)
            .map(|x| x as u32)
            .collect();

        Ok(FreedomAnalysis { underconstrained })
    }
}
