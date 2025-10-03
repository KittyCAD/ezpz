use super::{ComplexField, LinearSolver, Mat, SolverError, SolverResult};
use dyn_stack::{MemBuffer, MemStack};
use error_stack::ResultExt;
use faer::{
    Conj, Par,
    linalg::solvers::FullPivLu,
    mat::MatMut,
    prelude::{Solve, SolveLstsq},
    sparse::{
        SparseColMatRef,
        linalg::lu::{LuRef, LuSymbolicParams, NumericLu, SymbolicLu, factorize_symbolic_lu},
        linalg::solvers::{Qr, SymbolicQr},
    },
};

#[inline]
fn fnv1a64_init() -> u64 {
    0xcbf29ce484222325
}
#[inline]
fn fnv1a64_step(mut h: u64, v: u64) -> u64 {
    h ^= v;
    h = h.wrapping_mul(0x100000001b3);
    h
}
#[inline]
fn hash_usize_slice(mut h: u64, s: &[usize]) -> u64 {
    for &x in s {
        #[cfg(target_pointer_width = "64")]
        {
            h = fnv1a64_step(h, x as u64);
        }
        #[cfg(target_pointer_width = "32")]
        {
            h = fnv1a64_step(h, x as u64);
        }
    }
    h
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PatternSig {
    nrows: usize,
    ncols: usize,
    nnz: usize,
    col_ptr_hash: u64,
    row_idx_hash: u64,
    col_ptr_ptr: *const usize,
    row_idx_ptr: *const usize,
}

fn pattern_sig<T>(a: &SparseColMatRef<'_, usize, T>) -> PatternSig {
    let sym = a.symbolic();
    let col_ptr = sym.col_ptr();
    let row_idx = sym.row_idx();

    let col_ptr_ptr = col_ptr.as_ptr();
    let row_idx_ptr = row_idx.as_ptr();

    let mut h = fnv1a64_init();
    let col_ptr_hash = hash_usize_slice(h, col_ptr);
    h = fnv1a64_init();
    let row_idx_hash = hash_usize_slice(h, row_idx);

    PatternSig {
        nrows: a.nrows(),
        ncols: a.ncols(),
        nnz: *col_ptr.last().unwrap_or(&0),
        col_ptr_hash,
        row_idx_hash,
        col_ptr_ptr,
        row_idx_ptr,
    }
}

pub struct FaerLu<T: ComplexField<Real = T>> {
    sym: Option<SymbolicLu<usize>>,
    num: NumericLu<usize, T>,
    scratch: Option<MemBuffer>,
    // Don’t share one FaerLu across threads.
    // It’s a mutable solver with internal scratch;
    // instead create one solver per worker and still reuse within that worker across many solves.
    sig: Option<PatternSig>,
}

impl<T: ComplexField<Real = T>> Default for FaerLu<T> {
    fn default() -> Self {
        Self {
            sym: None,
            num: NumericLu::new(),
            scratch: None,
            sig: None,
        }
    }
}

impl<T: ComplexField<Real = T>> LinearSolver<T, SparseColMatRef<'_, usize, T>> for FaerLu<T> {
    fn factor(&mut self, a: &SparseColMatRef<'_, usize, T>) -> SolverResult<()> {
        let now = pattern_sig(a);
        let par = Par::Seq;

        let need_symbolic = match self.sig {
            None => true,
            Some(prev) => {
                if prev.col_ptr_ptr == now.col_ptr_ptr && prev.row_idx_ptr == now.row_idx_ptr {
                    false
                } else {
                    prev != now
                }
            }
        };

        if need_symbolic {
            self.sym = Some(
                factorize_symbolic_lu(a.symbolic(), LuSymbolicParams::default())
                    .attach_printable("LU symbolic factorization failed")
                    .change_context(SolverError)?,
            );

            let scratch_size = self
                .sym
                .as_ref()
                .ok_or(SolverError)
                .attach_printable("Symbolic factorization missing")?
                .factorize_numeric_lu_scratch::<T>(par, Default::default());
            self.scratch = Some(MemBuffer::new(scratch_size));
            self.sig = Some(now);
        }

        let stack = MemStack::new(
            self.scratch
                .as_mut()
                .ok_or(SolverError)
                .attach_printable("Scratch buffer not initialized")?,
        );

        self.sym
            .as_ref()
            .ok_or(SolverError)
            .attach_printable("Symbolic factorization not available")?
            .factorize_numeric_lu(&mut self.num, *a, par, stack, Default::default())
            .attach_printable("Numeric LU factorization failed")
            .change_context(SolverError)?;

        Ok(())
    }

    fn solve_in_place(&mut self, mut rhs: MatMut<T>) -> SolverResult<()> {
        let stack = MemStack::new(
            self.scratch
                .as_mut()
                .ok_or(SolverError)
                .attach_printable("Scratch buffer not available for solve")?,
        );

        let lu_ref = unsafe {
            LuRef::new_unchecked(
                self.sym
                    .as_ref()
                    .ok_or(SolverError)
                    .attach_printable("Symbolic factorization not available for solve")?,
                &self.num,
            )
        };

        // LU is naturally in-place.
        lu_ref.solve_in_place_with_conj(Conj::No, rhs.as_mut(), Par::Seq, stack);
        Ok(())
    }
}

pub struct SparseQr<T> {
    symbolic: Option<SymbolicQr<usize>>,
    qr: Option<Qr<usize, T>>,
    sig: Option<PatternSig>,
}

impl<T> Default for SparseQr<T> {
    fn default() -> Self {
        Self {
            symbolic: None,
            qr: None,
            sig: None,
        }
    }
}

impl<T: ComplexField<Real = T>> LinearSolver<T, SparseColMatRef<'_, usize, T>> for SparseQr<T> {
    fn factor(&mut self, a: &SparseColMatRef<'_, usize, T>) -> SolverResult<()> {
        let now = pattern_sig(a);

        let need_symbolic = match self.sig {
            None => true,
            Some(prev) => {
                if prev.col_ptr_ptr == now.col_ptr_ptr && prev.row_idx_ptr == now.row_idx_ptr {
                    false
                } else {
                    prev != now
                }
            }
        };

        if need_symbolic {
            self.symbolic = Some(
                SymbolicQr::try_new(a.symbolic())
                    .attach_printable("QR symbolic factorization failed")
                    .change_context(SolverError)?,
            );
            self.sig = Some(now);
        }

        // Create the numeric QR factorization from a symbolic.
        self.qr = Some(
            Qr::try_new_with_symbolic(
                self.symbolic
                    .as_ref()
                    .ok_or(SolverError)
                    .attach_printable("Symbolic factorization not available")?
                    .clone(),
                *a,
            )
            .attach_printable("Numeric QR factorization failed")
            .change_context(SolverError)?,
        );

        Ok(())
    }

    fn solve_in_place(&mut self, mut rhs: MatMut<T>) -> SolverResult<()> {
        let qr = self
            .qr
            .as_ref()
            .ok_or(SolverError)
            .attach_printable("QR factorization not available for solve")?;

        // Least-squares: faer writes the solution into the top ncols(A) rows of `rhs`.
        qr.solve_lstsq_in_place(rhs.as_mut());
        Ok(())
    }
}

pub struct DenseLu<T: ComplexField<Real = T>> {
    lu: Option<FullPivLu<T>>,
}

impl<T: ComplexField<Real = T>> Default for DenseLu<T> {
    fn default() -> Self {
        Self { lu: None }
    }
}

impl<T: ComplexField<Real = T>> LinearSolver<T, Mat<T>> for DenseLu<T> {
    fn factor(&mut self, a: &Mat<T>) -> SolverResult<()> {
        self.lu = Some(a.full_piv_lu());
        Ok(())
    }

    fn solve_in_place(&mut self, mut rhs: MatMut<T>) -> SolverResult<()> {
        let lu = self
            .lu
            .as_ref()
            .ok_or(SolverError)
            .attach_printable("Dense LU not factorized")?;

        // FullPivLu returns a new matrix; copy the result back into `rhs` to keep in-place.
        let solution = lu.solve(rhs.as_ref());
        rhs.copy_from(&solution);
        Ok(())
    }
}
