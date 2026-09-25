//! Finding degrees of freedom and assessing which variables are underconstrained.
use faer::{
    Mat, MatRef,
    linalg::solvers::{ColPivQr, Qr},
    mat::{AsMatMut, AsMatRef},
    perm::permute_rows,
    sparse::SparseColMatRef,
};

use crate::{ConstraintEntry, FreedomAnalysis, NonLinearSystemError, solver::Model};

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

        let mut redundant = Vec::new();
        let rank = dependent_constraints(j_dense.as_mat_ref(), self.constraints, &mut redundant);
        Ok(FreedomAnalysis::new(
            underconstrained,
            redundant,
            rank,
            j_dense.nrows(),
        ))
    }
}

/// Walks the Jacobian's rows in constraint order and keeps each row that is
/// linearly independent of the rows kept before it. A row that adds nothing is
/// dependent, and its constraint goes into `redundant`.
/// Returns the rank of the Jacobian, which is the number of rows kept.
///
/// Going in order, rather than taking the pivots of a QR of Jᵀ, means that of two
/// constraints which say the same thing, the later one is reported. That is
/// usually the one the user just added, so it's the one they'd expect to hear about.
fn dependent_constraints(
    jacobian: MatRef<'_, f64>,
    constraints: &[ConstraintEntry<'_>],
    redundant: &mut Vec<usize>,
) -> usize {
    let nvars = jacobian.ncols();
    let largest_row_norm = jacobian
        .row_iter()
        .map(|row| row.norm_l2())
        .fold(0.0, libm::fmax);
    // Same relative tolerance as the rank in `orthonormal_nullspace`, so that
    // both halves of the analysis agree on what "independent" means.
    let tolerance = TOLERANCE_BASE * largest_row_norm;

    // Orthonormal basis of the rows kept so far, one per column.
    let mut basis = Mat::<f64>::zeros(nvars, jacobian.nrows().min(nvars));
    let mut rank = 0;
    let mut candidate = vec![0.0; nvars];
    let mut row = 0;
    for constraint in constraints {
        let mut dependent = false;
        for _ in 0..constraint.constraint.residual_dim() {
            for (c, &j) in candidate.iter_mut().zip(jacobian.row(row).iter()) {
                *c = j;
            }
            row += 1;
            // Gram-Schmidt, twice: one pass loses orthogonality when the
            // candidate is nearly in the span, which is exactly the case we're testing.
            for _ in 0..2 {
                for k in 0..rank {
                    let q = basis.col(k);
                    let dot: f64 = candidate.iter().zip(q.iter()).map(|(c, q)| c * q).sum();
                    for (c, q) in candidate.iter_mut().zip(q.iter()) {
                        *c -= dot * q;
                    }
                }
            }
            let norm = candidate.iter().map(|c| c * c).sum::<f64>().sqrt();
            if norm <= tolerance || rank == nvars {
                dependent = true;
                continue;
            }
            for (k, c) in candidate.iter().enumerate() {
                basis[(k, rank)] = c / norm;
            }
            rank += 1;
        }
        if dependent {
            redundant.push(constraint.id);
        }
    }
    rank
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
