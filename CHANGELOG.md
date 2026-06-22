# 0.2.27

## Changed

 - Change the core numeric solver loop (adaptive damping, scale-covariance) (#270)

# 0.2.26

## Added

 - Per-constraint weights (#260)

## Changed

 - DidNotConverge is no longer an error, instead, check the `.converged` property (#266)
 - Faster freedom analysis (#263)

# 0.2.25

## Changed

 - Bump faer
 - Do floating-point math in libm rather than std::f64

# 0.2.24

## Added

 - Add `PointsAtAngle` constraint (#252)

## Internal changes

 - Disallow std `sin()` `cos()` in favor of libm (#251)

# 0.2.23

## Fixed

 - Improve arc tangency stability (#248)

# 0.1.3

* Report unsatisfied constraints (#124)
* Configurable max number of iterations of Newton's Method (#122)
* Bump faer to 0.23 (#119)
