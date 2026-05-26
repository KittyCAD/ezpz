//! Finding degrees of freedom and assessing which variables are underconstrained.
use faer::{
    Mat, MatRef,
    linalg::solvers::{ColPivQr, Qr},
    mat::{AsMatMut, AsMatRef},
    perm::permute_rows,
    sparse::SparseColMatRef,
};

use crate::{FreedomAnalysis, NonLinearSystemError, solver::Model};

const TOLERANCE_BASE: f64 = 1E-8;

impl Model<'_> {
    pub(crate) fn freedom_analysis(&self) -> Result<FreedomAnalysis, NonLinearSystemError> {
        let j_sparse =
            SparseColMatRef::new(self.jacobian_cache.sym.as_ref(), &self.jacobian_cache.vals);
        let j_dense = j_sparse.to_dense();
        let nvars = self.layout.num_variables;
        debug_assert_eq!(
            nvars,
            j_dense.ncols(),
            "Jacobian was malformed, Adam messed something up here."
        );

        let nullspace = orthonormal_nullspace(j_dense.as_mat_ref(), nvars)?;
        let underconstrained = underconstrained_variables(nullspace.as_mat_ref(), nvars);
        Ok(FreedomAnalysis::new(underconstrained))
    }
}

fn orthonormal_nullspace(
    jacobian: MatRef<'_, f64>,
    nvars: usize,
) -> Result<Mat<f64>, NonLinearSystemError> {
    let qr = ColPivQr::new(jacobian);
    let r = qr.R();
    let ndiag = r.nrows().min(r.ncols());

    let largest_diagonal = (0..ndiag)
        .map(|i| r.get(i, i).abs())
        .reduce(libm::fmax)
        .ok_or(NonLinearSystemError::EmptySystemNotAllowed)?;
    let tolerance = TOLERANCE_BASE * largest_diagonal;
    let rank = (0..ndiag)
        .take_while(|&i| r.get(i, i).abs() > tolerance)
        .count();
    let nullity = nvars - rank;

    let mut permuted_nullspace = Mat::zeros(nvars, nullity);
    for free_col in 0..nullity {
        let free_var = rank + free_col;
        permuted_nullspace[(free_var, free_col)] = 1.0;

        // For J P^T = Q R, solve R11 x + R12 z = 0 with one free
        // coordinate of z set to 1. This gives a basis for null(J P^T).
        for i in (0..rank).rev() {
            let mut rhs = *r.get(i, free_var);
            for j in (i + 1)..rank {
                rhs += r.get(i, j) * permuted_nullspace[(j, free_col)];
            }

            let diagonal = r.get(i, i);
            if diagonal.abs() <= tolerance {
                return Err(NonLinearSystemError::EmptySystemNotAllowed);
            }
            permuted_nullspace[(i, free_col)] = -rhs / diagonal;
        }
    }

    let mut nullspace = Mat::zeros(nvars, nullity);
    permute_rows(
        nullspace.as_mat_mut(),
        permuted_nullspace.as_mat_ref(),
        qr.P().inverse(),
    );

    Ok(Qr::new(nullspace.as_mat_ref()).compute_thin_Q())
}

fn underconstrained_variables(
    nullspace: faer::mat::generic::Mat<faer::mat::Ref<'_, f64>>,
    nvars: usize,
) -> Vec<crate::Id> {
    debug_assert_eq!(nvars, nullspace.nrows());

    // Compute participation norm for each variable.
    // If a variable's participation is basically zero, then it's constrained.
    // If it's nonzero, then it moves in some DOF and is unconstrained.
    let participation: Vec<f64> = nullspace
        .row_iter()
        .map(|row| row.squared_norm_l2())
        .collect();
    let max_participation = participation.iter().copied().fold(0.0, libm::fmax);

    // Relative threshold to classify variables
    let var_tol = 1e-3 * max_participation;
    let squared_tol = var_tol * var_tol;

    (0..nvars)
        .filter(|&j| participation[j] > squared_tol)
        .map(|x| x as u32)
        .collect()
}
