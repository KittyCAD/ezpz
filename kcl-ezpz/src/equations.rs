// Big thanks to Matt Keeter for inspiring this approach,
// see https://www.mattkeeter.com/projects/constraints/
use indexmap::IndexMap;
use libm::{cos, sin};

use crate::Error;

pub type Label = String;

/// The value of each variable in an equation.
pub type Vars = IndexMap<Label, f64>;

/// Result of evaluating an equation.
#[derive(Debug, PartialEq)]
pub struct Eval {
    /// The value of the equation.
    pub value: f64,
    /// All derivatives of all variables.
    pub derivatives: Vars,
}

/// This is basically a newtype for
/// `Fn(&Vars) -> Result<Eval>`.
trait Evaluate: Fn(&Vars) -> Result<Eval, Error> {}
impl<F> Evaluate for F where F: Fn(&Vars) -> Result<Eval, Error> {}

/// Symbolic equation that can be evaluated.
pub struct Equation {
    /// An equation really is nothing more than something to be evaluated.
    /// So all the significant logic for the equation lives in this closure.
    eval: Box<dyn Evaluate>,
    #[cfg(test)]
    debug_repr: String,
}

#[cfg(test)]
impl std::fmt::Debug for Equation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} = 0", self.debug_repr)
    }
}

impl Equation {
    /// Simplest equation: a constant.
    /// Does not depend on input variables at all.
    pub fn constant(value: f64) -> Self {
        let eval = move |_vars: &Vars| {
            let derivatives = Vars::new();
            Ok(Eval { value, derivatives })
        };
        Self {
            eval: Box::new(eval),
            #[cfg(test)]
            debug_repr: value.to_string(),
        }
    }

    /// Simple equation with a single variable.
    /// E.g. `x`.
    pub fn single_variable(label: Label) -> Self {
        #[cfg(test)]
        let debug_repr = label.clone();
        let label2 = label.clone();
        let eval = move |vars: &Vars| {
            let Some(var_value) = vars.get(&label2).copied() else {
                return Err(Error::NonLinearSystemError(
                    crate::NonLinearSystemError::SymbolNotFound(label2.to_owned()),
                ));
            };

            let mut derivatives = Vars::with_capacity(1);
            derivatives.insert(label2.clone(), 1.0);

            Ok(Eval {
                value: var_value,
                derivatives,
            })
        };
        Self {
            eval: Box::new(eval),
            #[cfg(test)]
            debug_repr,
        }
    }

    pub fn evaluate(&self, vars: &Vars) -> Result<Eval, Error> {
        (self.eval)(vars)
    }
}

impl std::ops::Add for Equation {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        #[cfg(test)]
        let debug_repr = format!("({} + {})", self.debug_repr, rhs.debug_repr);

        let eval = move |vars: &Vars| {
            let Eval {
                value: va,
                derivatives: das,
            } = self.evaluate(vars)?;
            let Eval {
                value: vb,
                derivatives: dbs,
            } = rhs.evaluate(vars)?;
            let derivatives = union_with(das, dbs, |a, b| a + b);
            Ok(Eval {
                value: va + vb,
                derivatives,
            })
        };
        Self {
            eval: Box::new(eval),
            #[cfg(test)]
            debug_repr,
        }
    }
}

impl std::ops::Sub for Equation {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        self + (-rhs)
    }
}

impl Equation {
    /// Assumes radians.
    pub fn sin(self) -> Self {
        #[cfg(test)]
        let debug_repr = format!("sin({})", self.debug_repr);
        let eval = move |vars: &Vars| {
            let Eval {
                value,
                mut derivatives,
            } = self.evaluate(vars)?;
            eprintln!("{derivatives:?}");
            derivatives.values_mut().for_each(|d| *d *= cos(value));
            Ok(Eval {
                value: sin(value),
                derivatives,
            })
        };
        Self {
            eval: Box::new(eval),
            #[cfg(test)]
            debug_repr,
        }
    }

    /// Assumes radians.
    pub fn cos(self) -> Self {
        #[cfg(test)]
        let debug_repr = format!("cos({})", self.debug_repr);
        let eval = move |vars: &Vars| {
            let Eval {
                value,
                mut derivatives,
            } = self.evaluate(vars)?;
            eprintln!("{derivatives:?}");
            derivatives.values_mut().for_each(|d| *d *= sin(value));
            Ok(Eval {
                value: cos(value),
                derivatives,
            })
        };
        Self {
            eval: Box::new(eval),
            #[cfg(test)]
            debug_repr,
        }
    }
}

impl std::ops::Mul for Equation {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        #[cfg(test)]
        let debug_repr = format!("({} * {})", self.debug_repr, rhs.debug_repr);
        let eval = move |vars: &Vars| {
            let Eval {
                value: va,
                derivatives: mut das,
            } = self.evaluate(vars)?;
            let Eval {
                value: vb,
                derivatives: mut dbs,
            } = rhs.evaluate(vars)?;
            // Product rule. Reuse storage for derivatives of A and B
            // so we don't have to reallocate. This saves 30% of time
            // when evaluating on our benchmarks, compared to
            // mapping over the dict and recollecting.
            das.values_mut().for_each(|d| *d *= vb);
            dbs.values_mut().for_each(|d| *d *= va);
            let derivatives = union_with(das, dbs, |a, b| a + b);
            Ok(Eval {
                value: va * vb,
                derivatives,
            })
        };
        Self {
            eval: Box::new(eval),

            #[cfg(test)]
            debug_repr,
        }
    }
}

impl std::ops::Div for Equation {
    type Output = Self;

    fn div(self, rhs: Self) -> Self::Output {
        #[cfg(test)]
        let debug_repr = format!("({} / {})", self.debug_repr, rhs.debug_repr);
        let eval = move |vars: &Vars| {
            let Eval {
                value: va,
                derivatives: mut das,
            } = self.evaluate(vars)?;
            let Eval {
                value: vb,
                derivatives: mut dbs,
            } = rhs.evaluate(vars)?;
            // Quotient rule. Reuse storage for derivatives of A and B
            // so we don't have to reallocate. This saves 30% of time
            // when evaluating on our benchmarks, compared to
            // mapping over the dict and recollecting.
            das.values_mut().for_each(|d| *d *= vb);
            dbs.values_mut().for_each(|d| *d *= -va);
            let mut derivatives = union_with(das, dbs, |a, b| a + b);
            let rb_squared = vb.powf(2.0);
            derivatives.values_mut().for_each(|d| *d /= rb_squared);
            Ok(Eval {
                value: va / vb,
                derivatives,
            })
        };
        Self {
            eval: Box::new(eval),
            #[cfg(test)]
            debug_repr,
        }
    }
}

impl std::ops::Neg for Equation {
    type Output = Self;

    fn neg(self) -> Self::Output {
        #[cfg(test)]
        let debug_repr = format!("-{}", self.debug_repr);
        let eval = move |vars: &Vars| {
            let Eval {
                value: r,
                mut derivatives,
            } = self.evaluate(vars)?;
            derivatives.values_mut().for_each(|d| *d = d.neg());
            Ok(Eval {
                value: -r,
                derivatives,
            })
        };
        Self {
            eval: Box::new(eval),
            #[cfg(test)]
            debug_repr,
        }
    }
}

/// Union two maps. If a value appears in both maps,
/// pass both instances into `f` and insert that value.
fn union_with<K: std::hash::Hash + Eq, V: Copy>(
    a: IndexMap<K, V>,
    b: IndexMap<K, V>,
    f: impl Fn(V, V) -> V,
) -> IndexMap<K, V> {
    let mut out = a;
    out.reserve(b.len());
    for (b_key, b_val) in b {
        if let Some(a_val) = out.get(&b_key) {
            // This requires a copy, but it's actually faster
            // to copy one f64 than to redo the IndexMap by shifting/swapping.
            // At least on the current benchmark suite. Feel free to measure alternatives.
            out.insert(b_key, f(*a_val, b_val));
        } else {
            out.insert(b_key, b_val);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;

    impl From<&str> for Equation {
        fn from(label: &str) -> Self {
            Equation::single_variable(label.to_owned())
        }
    }

    impl From<f64> for Equation {
        fn from(constant: f64) -> Self {
            Equation::constant(constant)
        }
    }

    fn f<T: Into<Equation>>(t: T) -> Equation {
        t.into()
    }

    // Convenience to make tests nicer
    fn vars(s: &str) -> Vars {
        let mut vars = Vars::new();
        for assign in s.replace(' ', "").split(',') {
            let (label, value) = assign.split_once('=').unwrap();
            let value = value.parse().unwrap();
            vars.insert(label.to_owned(), value);
        }
        vars
    }

    #[test]
    fn eval_single_var() {
        let equation = f("a");

        let actual = equation.evaluate(&vars("a=14")).unwrap();
        let expected = Eval {
            value: 14.0,
            derivatives: vars("a=1"),
        };
        assert_eq!(actual, expected);
    }

    #[test]
    fn eval_two_vars() {
        let equation = f("a") + f("b");

        let actual = equation.evaluate(&vars("a=14,b=2")).unwrap();
        let expected = Eval {
            value: 16.0,
            derivatives: vars("a=1,b=1"),
        };
        assert_eq!(actual, expected);
    }

    #[test]
    fn eval_same_var_added() {
        let equation = f("a") + f("a") + f("b");

        let actual = equation.evaluate(&vars("a=14, b=3")).unwrap();
        let expected = Eval {
            value: 31.0,
            derivatives: vars("a=2, b=1"),
        };
        assert_eq!(actual, expected);
    }

    #[test]
    fn eval_divided() {
        let equation = (f("a") + f("a") + f("b")) / f("a");

        let actual = equation.evaluate(&vars("a=3,b=2")).unwrap();
        let expected = Eval {
            value: 8.0 / 3.0,
            derivatives: IndexMap::from([
                ("a".to_owned(), -2.0 / 9.0),
                ("b".to_owned(), 1.0 / 3.0),
            ]),
        };
        assert_eq!(actual, expected);
    }

    #[test]
    fn eval_with_constant() {
        // Basically (x + 5) * (x + y)
        let equation = (f("x") + f(5.0)) * (f("x") + f("y"));

        let actual = equation.evaluate(&vars("x=2,y=3")).unwrap();
        let expected = Eval {
            value: 35.0,
            derivatives: vars("x=12, y=7"),
        };
        assert_eq!(actual, expected);
    }

    #[test]
    fn eval_negated() {
        // These two should be equivalent.
        let equation0 = -f("x");
        let equation1 = f("x") * f(-1.0);

        // So their evaluations should be equivalent.
        let actual0 = equation0.evaluate(&vars("x=2")).unwrap();
        let actual1 = equation1.evaluate(&vars("x=2")).unwrap();

        let expected = Eval {
            value: -2.0,
            derivatives: vars("x=-1"),
        };
        assert_eq!(actual0, expected);
        assert_eq!(actual1, expected);
    }

    #[test]
    fn eval_sin() {
        // f(x) = sin(2x)
        // so its derivative d/dx f(x) = cos(2x)
        let eq0 = (f(2.0) * f("x")).sin();

        // Let's evaluate it when:
        let x = 0.75 * PI;
        let expected_fx = sin(2.0 * x);
        // Remember: d/dx sin(kx) === k.cos(kx)
        let expected_pdx = 2.0 * cos(x * 2.0);
        // That should be 0, but due to imperfect representation of pi it'll actually be some tiny tiny number.
        assert!(expected_pdx < 0.00000000000001);

        let expected = Eval {
            value: expected_fx,
            derivatives: vars(&format!("x={expected_pdx}")),
        };
        let actual = eq0.evaluate(&vars(&format!("x={x}"))).unwrap();
        assert_nearly(actual.value, expected.value);
        assert_nearly(actual.derivatives["x"], expected.derivatives["x"]);
    }

    #[track_caller]
    fn assert_nearly(lhs: f64, rhs: f64) {
        let difference = (lhs - rhs).abs();
        assert!(
            difference < EPSILON,
            "LHS was {lhs}, RHS was {rhs}, difference was {difference}"
        );
    }
    const EPSILON: f64 = 0.0001;
}
