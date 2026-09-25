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
        let rank = redundant_constraints(j_dense.as_mat_ref(), self.constraints, &mut redundant)?;
        Ok(FreedomAnalysis::new(
            underconstrained,
            redundant,
            rank,
            j_dense.nrows(),
        ))
    }
}

/// How much of a constraint's rows must lie in the left null space for it to count as
/// taking part in a dependency. Entries of an orthonormal null-space basis are O(1) on
/// the rows of a genuine dependency, and at the level of the solver's final error on the
/// rest, so anything between the two separates them.
const PARTICIPATION_TOLERANCE: f64 = 1E-6;

/// Finds the constraints which could each be removed without lowering the rank of the
/// Jacobian, i.e. without freeing anything, and pushes their ids onto `redundant`.
/// Returns the rank of the Jacobian.
///
/// With N an orthonormal basis of the left null space of J (the dependencies between
/// its rows), removing a constraint's rows R leaves the rank at
/// `rank(J) - |R| + rank(N[R, :])`. So a constraint is redundant exactly when its block
/// of N has full row rank. For a one-row constraint, that is "the row appears in some
/// dependency". A multi-row constraint can take part in a dependency and still not be
/// redundant: `PointsCoincident` whose x is implied by the rest but whose y is not.
fn redundant_constraints(
    jacobian: MatRef<'_, f64>,
    constraints: &[ConstraintEntry<'_>],
    redundant: &mut Vec<usize>,
) -> Result<usize, NonLinearSystemError> {
    let (neqs, nvars) = (jacobian.nrows(), jacobian.ncols());

    // Which rows depend on which doesn't change with their scale, but the SVD's
    // tolerance does: a weighted constraint, or one in radians next to ones in
    // millimetres, would otherwise dominate it. A zero row stays zero, and is its own
    // dependency. Not EPSILON: a small gradient is still a direction (an angle between
    // long lines has one of about 1/length), and dividing by it is harmless.
    let mut rows = jacobian.to_owned();
    for row in rows.row_iter_mut() {
        let norm = row.norm_l2();
        if norm > 0.0 {
            for x in row.iter_mut() {
                *x /= norm;
            }
        }
    }

    let svd = rows.svd().map_err(NonLinearSystemError::FaerSvd)?;
    let singular = svd.S().column_vector();
    let largest = singular.iter().copied().fold(0.0, libm::fmax);
    let tolerance = TOLERANCE_BASE * largest;
    let rank = singular.iter().filter(|&&s| s > tolerance).count();
    let nullity = neqs - rank;
    debug_assert!(rank <= nvars);
    if nullity == 0 {
        return Ok(rank);
    }
    // U is neqs x neqs, and its columns past the rank span the left null space.
    let left_null = svd.U().subcols(rank, nullity);

    let mut row = 0;
    for constraint in constraints {
        let dim = constraint.constraint.residual_dim();
        let block = left_null.subrows(row, dim);
        row += dim;
        let block_rank = block
            .singular_values()
            .map_err(NonLinearSystemError::FaerSvd)?
            .iter()
            .filter(|&&s| s > PARTICIPATION_TOLERANCE)
            .count();
        if block_rank == dim {
            redundant.push(constraint.id);
        }
    }
    Ok(rank)
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
