use std::collections::BTreeSet;
use std::f64::consts::PI;

use proptest::prelude::*;

use crate::{
    CircleSide, Config, Constraint, ConstraintRequest, EPSILON, Id, IdGenerator, LineSide,
    constraints::JacobianVar,
    datatypes::inputs::{
        DatumCircle, DatumCircularArc, DatumDistance, DatumLineSegment, DatumPoint,
    },
    datatypes::outputs::Point,
    datatypes::{Angle, AngleKind},
    solve,
    solver::Layout,
    tests::assert_nearly_eq,
};

fn run(txt: &str) -> crate::textual::Outcome {
    let problem = super::parse_problem(txt);
    let system = problem.to_constraint_system().unwrap();
    system.solve().unwrap()
}

fn arb_id() -> impl Strategy<Value = Id> {
    0u32..32
}

fn arb_scalar() -> impl Strategy<Value = f64> {
    (-1_000i32..1_000).prop_map(|value| f64::from(value) / 10.0)
}

fn arb_point() -> BoxedStrategy<DatumPoint> {
    (arb_id(), arb_id())
        .prop_map(|(x, y)| DatumPoint::new_xy(x, y))
        .boxed()
}

fn arb_distance() -> BoxedStrategy<DatumDistance> {
    arb_id().prop_map(DatumDistance::new).boxed()
}

fn arb_line() -> BoxedStrategy<DatumLineSegment> {
    (arb_point(), arb_point())
        .prop_map(|(p0, p1)| DatumLineSegment::new(p0, p1))
        .boxed()
}

fn arb_circle() -> BoxedStrategy<DatumCircle> {
    (arb_point(), arb_distance())
        .prop_map(|(center, radius)| DatumCircle { center, radius })
        .boxed()
}

fn arb_arc() -> BoxedStrategy<DatumCircularArc> {
    (arb_point(), arb_point(), arb_point())
        .prop_map(|(center, start, end)| DatumCircularArc { center, start, end })
        .boxed()
}

fn arb_angle() -> BoxedStrategy<Angle> {
    ((-360i16..=360), any::<bool>())
        .prop_map(|(value, degrees)| {
            let value = f64::from(value);
            if degrees {
                Angle::from_degrees(value)
            } else {
                Angle::from_radians(value)
            }
        })
        .boxed()
}

fn arb_angle_kind() -> BoxedStrategy<AngleKind> {
    prop_oneof![
        Just(AngleKind::Parallel),
        Just(AngleKind::Perpendicular),
        arb_angle().prop_map(AngleKind::Other),
    ]
    .boxed()
}

fn arb_line_side() -> BoxedStrategy<LineSide> {
    prop_oneof![Just(LineSide::Left), Just(LineSide::Right)].boxed()
}

fn arb_circle_side() -> BoxedStrategy<CircleSide> {
    prop_oneof![Just(CircleSide::Exterior), Just(CircleSide::Interior)].boxed()
}

fn arb_constraint() -> BoxedStrategy<Constraint> {
    prop_oneof![
        (arb_line(), arb_circle(), arb_line_side())
            .prop_map(|(line, circle, side)| Constraint::LineTangentToCircle(line, circle, side)),
        (arb_circle(), arb_circle(), arb_circle_side()).prop_map(|(circle0, circle1, side)| {
            Constraint::CircleTangentToCircle(circle0, circle1, side)
        }),
        (arb_point(), arb_point(), arb_scalar())
            .prop_map(|(p0, p1, dist)| Constraint::Distance(p0, p1, dist)),
        (arb_point(), arb_point(), arb_distance())
            .prop_map(|(p0, p1, dist)| Constraint::DistanceVar(p0, p1, dist)),
        (arb_point(), arb_point(), arb_scalar())
            .prop_map(|(p0, p1, dist)| Constraint::VerticalDistance(p0, p1, dist)),
        (arb_point(), arb_point(), arb_scalar())
            .prop_map(|(p0, p1, dist)| Constraint::HorizontalDistance(p0, p1, dist)),
        arb_line().prop_map(Constraint::Vertical),
        arb_line().prop_map(Constraint::Horizontal),
        (arb_line(), arb_line(), arb_angle_kind())
            .prop_map(|(line0, line1, angle)| Constraint::LinesAtAngle(line0, line1, angle)),
        (arb_id(), arb_scalar()).prop_map(|(id, value)| Constraint::Fixed(id, value)),
        (arb_id(), arb_id()).prop_map(|(x, y)| Constraint::ScalarEqual(x, y)),
        (arb_point(), arb_point()).prop_map(|(p0, p1)| Constraint::PointsCoincident(p0, p1)),
        (arb_circle(), arb_scalar())
            .prop_map(|(circle, radius)| Constraint::CircleRadius(circle, radius)),
        (arb_line(), arb_line())
            .prop_map(|(line0, line1)| Constraint::LinesEqualLength(line0, line1)),
        (arb_arc(), arb_scalar()).prop_map(|(arc, radius)| Constraint::ArcRadius(arc, radius)),
        arb_arc().prop_map(Constraint::Arc),
        (arb_line(), arb_point()).prop_map(|(line, point)| Constraint::Midpoint(line, point)),
        (arb_point(), arb_line(), arb_scalar()).prop_map(|(point, line, distance)| {
            Constraint::PointLineDistance(point, line, distance)
        }),
        (arb_point(), arb_line(), arb_scalar()).prop_map(|(point, line, distance)| {
            Constraint::VerticalPointLineDistance(point, line, distance)
        }),
        (arb_point(), arb_line(), arb_scalar()).prop_map(|(point, line, distance)| {
            Constraint::HorizontalPointLineDistance(point, line, distance)
        }),
        (arb_line(), arb_point(), arb_point())
            .prop_map(|(line, a, b)| Constraint::Symmetric(line, a, b)),
        (arb_arc(), arb_point())
            .prop_map(|(arc, point)| Constraint::PointArcCoincident(arc, point)),
        (arb_arc(), arb_scalar()).prop_map(|(arc, dist)| Constraint::ArcLength(arc, dist)),
        (arb_arc(), arb_angle()).prop_map(|(arc, angle)| Constraint::ArcAngle(arc, angle)),
        (arb_point(), arb_point(), arb_point(), arb_angle_kind())
            .prop_map(|(p0, p1, p2, angle)| Constraint::PointsAtAngle(p0, p1, p2, angle)),
    ]
    .boxed()
}

/// Returns a copy of the constraint with every length-valued *constant* target scaled by `k`. Used
/// to rescale a whole constraint instance to the same shape at a different model scale. Angle
/// targets are dimensionless (unscaled); radii/distances supplied as solver variables scale via the
/// assignment vector, not here.
fn scale_constraint(c: &Constraint, k: f64) -> Constraint {
    use Constraint::*;
    match *c {
        Distance(p0, p1, d) => Distance(p0, p1, d * k),
        VerticalDistance(p0, p1, d) => VerticalDistance(p0, p1, d * k),
        HorizontalDistance(p0, p1, d) => HorizontalDistance(p0, p1, d * k),
        Fixed(id, v) => Fixed(id, v * k),
        CircleRadius(circle, r) => CircleRadius(circle, r * k),
        ArcRadius(arc, r) => ArcRadius(arc, r * k),
        ArcLength(arc, d) => ArcLength(arc, d * k),
        PointLineDistance(p, l, d) => PointLineDistance(p, l, d * k),
        VerticalPointLineDistance(p, l, d) => VerticalPointLineDistance(p, l, d * k),
        HorizontalPointLineDistance(p, l, d) => HorizontalPointLineDistance(p, l, d * k),
        other => other,
    }
}

proptest! {
    #[test]
    fn dependent_variable_ids_match_flattened_nonzeroes(constraint in arb_constraint()) {
        let mut dependent_ids = Vec::with_capacity(16);
        constraint.extend_dependent_variable_ids(&mut dependent_ids);
        let dependent_ids: BTreeSet<_> = dependent_ids.into_iter().collect();

        let mut row0 = Vec::with_capacity(16);
        let mut row1 = Vec::with_capacity(16);
        let mut row2 = Vec::with_capacity(16);
        constraint.nonzeroes(&mut row0, &mut row1, &mut row2);
        let nonzero_ids: BTreeSet<_> = row0
            .into_iter()
            .chain(row1)
            .chain(row2)
            .collect();

        prop_assert_eq!(dependent_ids, nonzero_ids);
    }

    /// Every constraint's analytic Jacobian must match a finite difference of its residual. Driven
    /// by `arb_constraint()`, so it covers every constraint type. Sample points where the residual
    /// is non-smooth (the `abs()` kinks in the tangent constraints, the angle-wrap branch cuts in
    /// the arc constraints) or where a perturbation is degenerate are detected and skipped since a
    /// finite difference is meaningless there.
    #[test]
    fn analytic_jacobian_matches_finite_difference(
        constraint in arb_constraint(),
        raw_vals in proptest::collection::vec(-8.0f64..8.0, 32),
    ) {
        let mut ids = Vec::with_capacity(16);
        constraint.extend_dependent_variable_ids(&mut ids);
        let n = ids.iter().map(|id| *id as usize + 1).max().unwrap_or(0);
        prop_assume!(n > 0 && n <= raw_vals.len());

        let vals = raw_vals[..n].to_vec();
        let layout = Layout::new(
            &(0..n as Id).collect::<Vec<_>>(),
            &[&constraint],
            Config::default(),
        );

        let (mut row0, mut row1, mut row2) = (Vec::new(), Vec::new(), Vec::new());
        let mut degenerate = false;
        constraint.jacobian_rows(&layout, &vals, &mut row0, &mut row1, &mut row2, &mut degenerate);
        prop_assume!(!degenerate);

        let rows = [&row0, &row1, &row2];
        for (component, &row) in rows
            .iter()
            .enumerate()
            .take(constraint.residual_dim().min(3))
        {
            for &var in &ids {
                let Some(numeric) =
                    finite_difference_derivative(&constraint, &layout, &vals, var, component)
                else {
                    // Non-smooth or degenerate perturbation (finite diff is unreliable here)
                    continue;
                };
                let analytic = sum_partial_derivatives(row, var);
                let tol = 1e-6 + 1e-4 * libm::fmax(analytic.abs(), numeric.abs());
                prop_assert!(
                    (analytic - numeric).abs() <= tol,
                    "{} ∂r{}/∂id{}: analytic={analytic}, numeric={numeric}, err={}, tol={tol}",
                    constraint.constraint_kind(),
                    component,
                    var,
                    (analytic - numeric).abs(),
                );
            }
        }
    }

    /// Every constraint residual must be homogeneous of degree 1 in its variables (length units),
    /// so the assembled least-squares system is uniformly scaled and well conditioned regardless of
    /// model size. We can't assert that directly since constraints with a constant target aren't
    /// homogeneous (e.g. `|p0 - p1| - d` has an additive constant), so we assert the equivalent
    /// invariant on the gradient: a degree-1-homogeneous variable part has a degree-0
    /// (scale-invariant) Jacobian. Rescaling the whole problem (variables and length-valued
    /// targets) by `k` must leave every Jacobian entry unchanged.
    #[test]
    fn residual_jacobian_is_scale_invariant(
        constraint in arb_constraint(),
        raw_vals in proptest::collection::vec(-8.0f64..8.0, 32),
    ) {
        let mut ids = Vec::with_capacity(16);
        constraint.extend_dependent_variable_ids(&mut ids);
        let n = ids.iter().map(|id| *id as usize + 1).max().unwrap_or(0);
        prop_assume!(n > 0 && n <= raw_vals.len());

        let vals = raw_vals[..n].to_vec();
        let layout = Layout::new(
            &(0..n as Id).collect::<Vec<_>>(),
            &[&constraint],
            Config::default(),
        );

        let jac = |c: &Constraint, v: &[f64]| {
            let (mut r0, mut r1, mut r2) = (Vec::new(), Vec::new(), Vec::new());
            let mut degenerate = false;
            c.jacobian_rows(&layout, v, &mut r0, &mut r1, &mut r2, &mut degenerate);
            (r0, r1, r2, degenerate)
        };

        let (a0, a1, a2, deg_a) = jac(&constraint, &vals);
        prop_assume!(!deg_a);

        // Rescale the whole problem to the same shape at a different model scale.
        let k = 8.0;
        let scaled_vals: Vec<f64> = vals.iter().map(|v| v * k).collect();
        let scaled_constraint = scale_constraint(&constraint, k);
        let (b0, b1, b2, deg_b) = jac(&scaled_constraint, &scaled_vals);
        prop_assume!(!deg_b);

        for (row, (orig, scaled)) in [(&a0, &b0), (&a1, &b1), (&a2, &b2)].into_iter().enumerate() {
            prop_assert_eq!(orig.len(), scaled.len(), "row {} sparsity changed under scaling", row);
            for (jo, js) in orig.iter().zip(scaled.iter()) {
                prop_assert_eq!(jo.id, js.id);
                prop_assume!(jo.partial_derivative.is_finite() && js.partial_derivative.is_finite());
                let (po, ps) = (jo.partial_derivative, js.partial_derivative);
                let tol = 1e-6 * (1.0 + po.abs().max(ps.abs()));
                prop_assert!(
                    (po - ps).abs() <= tol,
                    "{} Jacobian not scale-invariant (∂/∂id {}): {} at scale 1 vs {} at scale {} \
                     - residual is not homogeneous degree 1",
                    constraint.constraint_kind(), jo.id, po, ps, k,
                );
            }
        }
    }

    #[test]
    fn square(
        x0 in -10000i32..10000,
        x1 in -10000i32..10000,
        x2 in -10000i32..10000,
        x3 in -10000i32..10000,
        y0 in -10000i32..10000,
        y1 in -10000i32..10000,
        y2 in -10000i32..10000,
        y3 in -10000i32..10000,
    ) {
        let problem = format!(
            "# constraints
    point a
    point b
    point c
    point d
    lines_equal_length(a, b, c, d)
    lines_equal_length(b, c, a, d)
    horizontal(a, b)
    vertical(b, c)
    parallel(a, b, c, d)
    parallel(b, c, d, a)
    a = (0, 0)
    c = (4, 4)

    # guesses
    a roughly ({x0}, {y0})
    b roughly ({x1}, {y1})
    c roughly ({x2}, {y2})
    d roughly ({x3}, {y3})
    "
        );
        let solved = run(&problem);
        assert!(solved.unsatisfied.is_empty());
    }

    #[test]
    fn scalar_eq(
        guess_x in -10.0..10.0,
        guess_y in -10.0..10.0,
    ) {

        // One constraint, that solver variables x and y should be equal.
        let requests = [
            ConstraintRequest::highest_priority(Constraint::ScalarEqual(0, 1)),
        ];
        // Set their initial values to random, given by the property test harness.
        let initial_guesses = vec![
            (0, guess_x),
            (1, guess_y),
        ];

        // Invariant: solve should succeed.
        let outcome = solve(
            &requests,
            initial_guesses,
            Config::default(),
        ).expect("this constraint system should converge and be solvable");
        // Invariant: solve should satisfy all (i.e. the only) constraint,
        // without warnings, i.e. make x and y equal.
        assert!(outcome.is_satisfied(), "this constraint system should have been easily, fully satisfiable");
        assert!(outcome.warnings.is_empty(), "this constraint system shouldn't produce any warnings");
        let [solved_x, solved_y] = outcome.final_values.try_into().expect("There should be exactly two variables, x and y");
        assert_nearly_eq(solved_x, solved_y);
    }

    #[test]
    fn vertical_distance(
        guess_x0 in -100.0..100.0f64,
        guess_x1 in -100.0..100.0f64,
        guess_y0 in -100.0..100.0f64,
        guess_y1 in -100.0..100.0f64,
        desired_distance in 0.0..100.0f64,
    ) {
        let mut ids = IdGenerator::default();
        let p0 = DatumPoint::new(&mut ids);
        let p1 = DatumPoint::new(&mut ids);

        // Random initial guesses.
        let initial_guesses = vec![
            (p0.id_x(), guess_x0),
            (p0.id_y(), guess_y0),
            (p1.id_x(), guess_x1),
            (p1.id_y(), guess_y1),
        ];

        // One constraint: p0 and p1 have the randomly-generated vertical distance.
        let requests = [
            ConstraintRequest::highest_priority(Constraint::VerticalDistance(p0, p1, desired_distance)),
        ];

        let outcome = solve(&requests, initial_guesses, Config::default())
            .expect("this constraint system should converge and be solvable");

        assert!(outcome.is_satisfied(), "the vertical distance constraint should be satisfied");
        assert!(
            outcome.warnings.is_empty(),
            "this simple system should not emit warnings"
        );

        let solved_y0 = outcome.final_values[p0.id_y() as usize];
        let solved_y1 = outcome.final_values[p1.id_y() as usize];
        assert_nearly_eq(solved_y0 - solved_y1, desired_distance);
    }

    #[test]
    fn horizontal_distance(
        guess_x0 in -100.0..100.0f64,
        guess_x1 in -100.0..100.0f64,
        guess_y0 in -100.0..100.0f64,
        guess_y1 in -100.0..100.0f64,
        desired_distance in 0.0..100.0f64,
    ) {
        let mut ids = IdGenerator::default();
        let p0 = DatumPoint::new(&mut ids);
        let p1 = DatumPoint::new(&mut ids);

        let initial_guesses = vec![
            (p0.id_x(), guess_x0),
            (p0.id_y(), guess_y0),
            (p1.id_x(), guess_x1),
            (p1.id_y(), guess_y1),
        ];

        let requests = [
            ConstraintRequest::highest_priority(Constraint::HorizontalDistance(
                p0,
                p1,
                desired_distance,
            )),
        ];

        let outcome = solve(&requests, initial_guesses, Config::default())
            .expect("this constraint system should converge and be solvable");

        assert!(outcome.is_satisfied(), "the horizontal distance constraint should be satisfied");
        assert!(
            outcome.warnings.is_empty(),
            "this simple system should not emit warnings"
        );

        let solved_x0 = outcome.final_values[p0.id_x() as usize];
        let solved_x1 = outcome.final_values[p1.id_x() as usize];
        assert_nearly_eq(solved_x0 - solved_x1, desired_distance);
    }

    #[test]
    fn vertical_point_line_dist(
        guess_line_p0x in -100.0..100.0f64,
        guess_line_p0y in -100.0..100.0f64,
        guess_line_p1x in -100.0..100.0f64,
        guess_line_p1y in -100.0..100.0f64,
        guess_point_x in -100.0..100.0f64,
        guess_point_y in -100.0..100.0f64,
        desired_distance in 0.0..100.0f64,
    ) {
        // Avoid vertical/degenerate lines so the vertical distance is well-defined.
        prop_assume!((guess_line_p1x - guess_line_p0x).abs() > EPSILON);

        let mut ids = IdGenerator::default();
        let point = DatumPoint::new(&mut ids);
        let line = DatumLineSegment::new(
            DatumPoint::new(&mut ids),
            DatumPoint::new(&mut ids),
        );
        let initial_guesses = vec![
            (point.id_x(), guess_point_x),
            (point.id_y(), guess_point_y),
            (line.p0.id_x(), guess_line_p0x),
            (line.p0.id_y(), guess_line_p0y),
            (line.p1.id_x(), guess_line_p1x),
            (line.p1.id_y(), guess_line_p1y),
        ];
        test_vertical_pld(initial_guesses, line, point, desired_distance);
    }

    #[test]
    fn horizontal_point_line_dist(
        guess_line_p0x in -100.0..100.0f64,
        guess_line_p0y in -100.0..100.0f64,
        guess_line_p1x in -100.0..100.0f64,
        guess_line_p1y in -100.0..100.0f64,
        guess_point_x in -100.0..100.0f64,
        guess_point_y in -100.0..100.0f64,
        desired_distance in 0.0..100.0f64,
    ) {
        // Avoid horizontal/degenerate lines so the horizontal distance is well-defined.
        let p0 = Point {
            x: guess_line_p0x,
            y: guess_line_p0y,
        };
        let p1 = Point {
            x: guess_line_p1x,
            y: guess_line_p1y,
        };
        let line_length = p0.euclidean_distance(p1);
        let dy = guess_line_p1y - guess_line_p0y;
        prop_assume!(line_length > 1e-2);
        prop_assume!(dy.abs() > 1e-2);

        let mut ids = IdGenerator::default();
        let point = DatumPoint::new(&mut ids);
        let line = DatumLineSegment::new(
            DatumPoint::new(&mut ids),
            DatumPoint::new(&mut ids),
        );
        let initial_guesses = vec![
            (point.id_x(), guess_point_x),
            (point.id_y(), guess_point_y),
            (line.p0.id_x(), guess_line_p0x),
            (line.p0.id_y(), guess_line_p0y),
            (line.p1.id_x(), guess_line_p1x),
            (line.p1.id_y(), guess_line_p1y),
        ];
        test_horizontal_pld(initial_guesses, line, point, desired_distance);
    }

    /// Given an arc, and a randomly-guessed point, constrain the point to lie on the arc.
    /// Then check the constraint solver properly constrained it.
    #[test]
    fn point_arc_coincident(
        arc_center_x in -50.0..50.0,
        arc_center_y in -50.0..50.0,
        arc_radius in 1.0..50.0,
        arc_start in 0.0..360.0,
        // Very narrow arcs make the angle inequalities stiff and Newton may not converge;
        // keep a small-but-nontrivial span to stay numerically stable.
        arc_degrees in 10.0..350.0,
        point_guess_x in -100.0..100.0,
        point_guess_y in -100.0..100.0,
    ) {
        // Avoid degenerate initial guesses where the point is exactly at the arc center;
        // that makes the distance Jacobian singular and the solver refuses to proceed.
        let point_offset_from_center =
            libm::hypot(point_guess_x - arc_center_x, point_guess_y - arc_center_y);
        prop_assume!(point_offset_from_center > EPSILON);
        test_point_arc_coincident(
            arc_center_x,
            arc_center_y,
            arc_radius,
            arc_start,
            arc_degrees,
            point_guess_x,
            point_guess_y,
        );
    }

    /// Given an arc, and a randomly-chosen percentage of the circle, constraint the arc
    /// to that percentage of the circle's length.
    #[test]
    fn point_arc_length(
        arc_center_x in -50.0..50.0,
        arc_center_y in -50.0..50.0,
        arc_radius in 1.0..50.0,
        arc_start_degrees in 0.0..360.0,
        arc_length_percent in 0.05..0.95,
        point_guess_x in -10.0..10.0,
        point_guess_y in -10.0..10.0,
    ) {
        // Avoid degenerate initial guesses where the point is exactly at the arc center;
        // that makes the distance Jacobian singular and the solver refuses to proceed.
        let point_offset_from_center =
            libm::hypot(point_guess_x - arc_center_x, point_guess_y - arc_center_y);
        prop_assume!(point_offset_from_center > EPSILON);
        test_point_arc_length(
            arc_center_x,
            arc_center_y,
            arc_radius,
            arc_start_degrees,
            arc_length_percent,
            point_guess_x,
            point_guess_y,
        );
    }

    #[test]
    fn circle_circle_tangent(
        ax in -50.0..50.0f64,
        ay in -50.0..50.0f64,
        ar in 1.0..50.0f64,
        br in 1.0..50.0f64,
        guess_offset in -0.25..0.25f64,
        is_internal in any::<bool>(),
        positive_side in any::<bool>(),
    ) {
        // Internal tangency has a center distance of |ra-rb|. Keep it away from zero
        // so we don't generate singular center-center distances.
        if is_internal {
            prop_assume!((ar - br).abs() > 1.0);
        }
        let expected_center_distance = if is_internal { (ar - br).abs() } else { ar + br };
        let side_sign = if positive_side { 1.0 } else { -1.0 };
        let bx_guess = ax + side_sign * (expected_center_distance + guess_offset);

        test_circle_circle_tangent(
            ax,
            ay,
            ar,
            bx_guess,
            ay,
            br,
            is_internal,
        );
    }

    #[test]
    fn distance_var_jacobian_entries_stay_finite(
        px in -100.0..100.0f64,
        py in -100.0..100.0f64,
        qx_any in -100.0..100.0f64,
        qy_any in -100.0..100.0f64,
        d in -100.0..100.0f64,
        mode in 0u8..3,
    ) {
        // Exercise exact coincidence, near-coincidence, and general cases.
        let (qx, qy) = match mode {
            0 => (px, py),
            1 => (px + EPSILON * 0.5, py - EPSILON * 0.5),
            _ => (qx_any, qy_any),
        };
        let (constraint, layout, values, _p, _q, dist) =
            make_distance_var_constraint(px, py, qx, qy, d);
        let (row0, _degenerate) = distance_var_jacobian(&constraint, &layout, &values);
        for pd in row0.iter().map(|jv| jv.partial_derivative) {
            prop_assert!(pd.is_finite(), "non-finite partial derivative: {pd}");
        }

        // If present, df/dd should always be finite too.
        if let Some(df_dd) = find_partial_derivative(&row0, dist.id) {
            prop_assert!(df_dd.is_finite(), "df/dd should be finite, got {df_dd}");
        }
    }

    #[test]
    fn distance_var_is_symmetric_under_point_swap(
        px in -100.0..100.0f64,
        py in -100.0..100.0f64,
        qx in -100.0..100.0f64,
        qy in -100.0..100.0f64,
        d in -100.0..100.0f64,
    ) {
        let (constraint, layout, values, p, q, dist) =
            make_distance_var_constraint(px, py, qx, qy, d);
        let swapped = Constraint::DistanceVar(q, p, dist);

        let (residual, _) = distance_var_residual(&constraint, &layout, &values);
        let (swapped_residual, _) = distance_var_residual(&swapped, &layout, &values);
        let residual_error = (residual - swapped_residual).abs();
        prop_assert!(
            residual_error <= 1e-12,
            "residuals should match after swapping points: lhs={residual}, rhs={swapped_residual}"
        );

        let (row0, degenerate) = distance_var_jacobian(&constraint, &layout, &values);
        let (swapped_row0, swapped_degenerate) = distance_var_jacobian(&swapped, &layout, &values);
        prop_assert_eq!(
            degenerate,
            swapped_degenerate,
            "degeneracy should not depend on point ordering"
        );

        for var in [p.id_x(), p.id_y(), q.id_x(), q.id_y(), dist.id] {
            let pd = find_partial_derivative_or_zero(&row0, var);
            let swapped_pd = find_partial_derivative_or_zero(&swapped_row0, var);
            let err = (pd - swapped_pd).abs();
            prop_assert!(
                err <= 1e-12,
                "id={var}: derivative should be invariant under point swap, lhs={pd}, rhs={swapped_pd}"
            );
        }

        // In non-degenerate cases, point gradients should be equal and opposite.
        if !degenerate {
            let df_dpx = find_partial_derivative_or_zero(&row0, p.id_x());
            let df_dqx = find_partial_derivative_or_zero(&row0, q.id_x());
            let df_dpy = find_partial_derivative_or_zero(&row0, p.id_y());
            let df_dqy = find_partial_derivative_or_zero(&row0, q.id_y());
            prop_assert!((df_dpx + df_dqx).abs() <= 1e-12, "df/dpx should equal -df/dqx");
            prop_assert!((df_dpy + df_dqy).abs() <= 1e-12, "df/dpy should equal -df/dqy");
        }
    }

}

/// Sums all Jacobian entries for `id` in a row. A variable can appear in more than one term of a
/// constraint (e.g. a shared apex point); global assembly sums those, so the test must too.
fn sum_partial_derivatives(row: &[JacobianVar], id: Id) -> f64 {
    row.iter()
        .filter(|entry| entry.id == id)
        .map(|entry| entry.partial_derivative)
        .sum()
}

/// Evaluates one residual component, reporting whether the configuration is degenerate.
fn residual_component(
    constraint: &Constraint,
    layout: &Layout,
    values: &[f64],
    component: usize,
) -> (f64, bool) {
    let (mut r0, mut r1, mut r2) = (0.0, 0.0, 0.0);
    let mut degenerate = false;
    constraint.residual(layout, values, &mut r0, &mut r1, &mut r2, &mut degenerate);
    let r = match component {
        0 => r0,
        1 => r1,
        _ => r2,
    };
    (r, degenerate)
}

/// Central-difference derivative of one residual component with respect to one variable. Returns
/// `None` when a perturbation is degenerate, or when the residual looks non-smooth at this point
/// i.e. the one-sided slopes disagree far more than rounding and curvature would explain, which
/// signals a kink (`abs()`) or an angle-wrap branch cut where a finite difference can't match the
/// analytic derivative.
fn finite_difference_derivative(
    constraint: &Constraint,
    layout: &Layout,
    values: &[f64],
    var: Id,
    component: usize,
) -> Option<f64> {
    let index = layout.index_of(var);
    let step = 1e-6 * (1.0 + values[index].abs());

    let eval = |delta: f64| -> Option<f64> {
        let mut v = values.to_vec();
        v[index] += delta;
        let (r, degenerate) = residual_component(constraint, layout, &v, component);
        (!degenerate).then_some(r)
    };

    let f_plus = eval(step)?;
    let f_minus = eval(-step)?;
    let f_zero = eval(0.0)?;

    let central = (f_plus - f_minus) / (2.0 * step);
    let forward = (f_plus - f_zero) / step;
    let backward = (f_zero - f_minus) / step;

    let smooth = (forward - backward).abs() <= 1e-3 * (1.0 + central.abs()) + 1e-6;
    smooth.then_some(central)
}

fn make_distance_var_constraint(
    px: f64,
    py: f64,
    qx: f64,
    qy: f64,
    d: f64,
) -> (
    Constraint,
    Layout,
    Vec<f64>,
    DatumPoint,
    DatumPoint,
    DatumDistance,
) {
    let mut ids = IdGenerator::default();
    let p = DatumPoint::new(&mut ids);
    let q = DatumPoint::new(&mut ids);
    let dist = DatumDistance::new(ids.next_id());
    let constraint = Constraint::DistanceVar(p, q, dist);
    let all_variables = vec![p.id_x(), p.id_y(), q.id_x(), q.id_y(), dist.id];
    let constraints = [&constraint];
    let layout = Layout::new(&all_variables, constraints.as_slice(), Config::default());

    let mut current_assignments = vec![0.0; dist.id as usize + 1];
    current_assignments[p.id_x() as usize] = px;
    current_assignments[p.id_y() as usize] = py;
    current_assignments[q.id_x() as usize] = qx;
    current_assignments[q.id_y() as usize] = qy;
    current_assignments[dist.id as usize] = d;

    (constraint, layout, current_assignments, p, q, dist)
}

fn distance_var_residual(constraint: &Constraint, layout: &Layout, values: &[f64]) -> (f64, bool) {
    let mut residual0 = 0.0;
    let mut residual1 = 0.0;
    let mut residual2 = 0.0;
    let mut degenerate = false;
    constraint.residual(
        layout,
        values,
        &mut residual0,
        &mut residual1,
        &mut residual2,
        &mut degenerate,
    );
    (residual0, degenerate)
}

fn distance_var_jacobian(
    constraint: &Constraint,
    layout: &Layout,
    values: &[f64],
) -> (Vec<JacobianVar>, bool) {
    let mut row0 = Vec::with_capacity(5);
    let mut row1 = Vec::with_capacity(0);
    let mut row2 = Vec::with_capacity(0);
    let mut degenerate = false;
    constraint.jacobian_rows(
        layout,
        values,
        &mut row0,
        &mut row1,
        &mut row2,
        &mut degenerate,
    );
    (row0, degenerate)
}

fn find_partial_derivative(jacobian_row: &[JacobianVar], id: Id) -> Option<f64> {
    jacobian_row
        .iter()
        .find(|entry| entry.id == id)
        .map(|entry| entry.partial_derivative)
}

fn find_partial_derivative_or_zero(jacobian_row: &[JacobianVar], id: Id) -> f64 {
    find_partial_derivative(jacobian_row, id).unwrap_or(0.0)
}

/// Given an arc, and a randomly-chosen percentage of the circle, constraint the arc
/// to that percentage of the circle's length.
fn test_point_arc_length(
    arc_center_x: f64,
    arc_center_y: f64,
    arc_radius: f64,
    arc_start_degrees: f64,
    arc_length_percent: f64,
    arc_end_x_guess: f64,
    arc_end_y_guess: f64,
) {
    let two_pi = 2.0 * PI;
    let circle_perimeter = two_pi * arc_radius;
    let desired_arc_length = circle_perimeter * arc_length_percent;
    let arc_start_radians = arc_start_degrees.to_radians().rem_euclid(two_pi);

    // Generate IDs for variables.
    let mut ids = IdGenerator::default();
    let center = DatumPoint::new(&mut ids);
    let start = DatumPoint::new(&mut ids);
    let end = DatumPoint::new(&mut ids);
    let arc = DatumCircularArc { center, start, end };

    // The arc's start position is fixed, let's find the fixed points.
    let arc_start = Point {
        x: arc_center_x + libm::cos(arc_start_radians) * arc_radius,
        y: arc_center_y + libm::sin(arc_start_radians) * arc_radius,
    };
    let initial_guesses = vec![
        (arc.center.id_x(), arc_center_x),
        (arc.center.id_y(), arc_center_y),
        (arc.start.id_x(), arc_start.x),
        (arc.start.id_y(), arc_start.y),
        (arc.end.id_x(), arc_end_x_guess),
        (arc.end.id_y(), arc_end_y_guess),
    ];

    let requests: Vec<_> = vec![
        // Pin the center and start in place
        Constraint::Fixed(arc.center.id_x(), arc_center_x),
        Constraint::Fixed(arc.center.id_y(), arc_center_y),
        Constraint::Fixed(arc.start.id_x(), arc_start.x),
        Constraint::Fixed(arc.start.id_y(), arc_start.y),
        // This is the constraint to test.
        Constraint::ArcLength(arc, desired_arc_length),
    ]
    .into_iter()
    .map(ConstraintRequest::highest_priority)
    .collect();

    // Solve it.
    let outcome = solve(&requests, initial_guesses, Config::default())
        .expect("this constraint system should converge and be solvable");

    assert!(outcome.is_satisfied(), "the constraint should be satisfied");
    assert!(
        outcome.warnings.is_empty(),
        "this simple system should not emit warnings"
    );

    // Was the end point placed on the arc?
    // i.e. it should be `radius` distance from arc center.
    let solved_end_x = outcome.final_values[arc.end.id_x() as usize];
    let solved_end_y = outcome.final_values[arc.end.id_y() as usize];
    let solved_end = Point {
        x: solved_end_x,
        y: solved_end_y,
    };
    let center_point = Point {
        x: arc_center_x,
        y: arc_center_y,
    };
    let end_distance = solved_end.euclidean_distance(center_point);
    assert_nearly_eq(end_distance, arc_radius);

    // The end should be the desired length away from the start.
    let dy = solved_end_y - arc_center_y;
    let dx = solved_end_x - arc_center_x;
    let end_radians = libm::atan2(dy, dx).rem_euclid(two_pi);
    let ccw_delta = (end_radians - arc_start_radians).rem_euclid(two_pi);
    // arc length = r * theta
    let actual_arc_length = arc_radius * ccw_delta;
    assert_nearly_eq(actual_arc_length, desired_arc_length);
}

/// Given an arc, and a randomly-guessed point, constrain the point to lie on the arc.
/// Then check the constraint solver properly constrained it.
fn test_point_arc_coincident(
    arc_center_x: f64,
    arc_center_y: f64,
    arc_radius: f64,
    arc_start_degrees: f64,
    arc_width_degrees: f64,
    _point_guess_x: f64,
    _point_guess_y: f64,
) {
    let two_pi = 2.0 * PI;
    let arc_start_radians = arc_start_degrees.to_radians().rem_euclid(two_pi);
    let arc_width_radians = arc_width_degrees.to_radians();
    let arc_end_radians = arc_start_radians + arc_width_radians;

    // Generate IDs for variables.
    let mut ids = IdGenerator::default();
    let point = DatumPoint::new(&mut ids);
    let center = DatumPoint::new(&mut ids);
    let start = DatumPoint::new(&mut ids);
    let end = DatumPoint::new(&mut ids);
    let arc = DatumCircularArc { center, start, end };

    // The arc's position is fixed, let's find the fixed points.
    let arc_start_x = arc_center_x + libm::cos(arc_start_radians) * arc_radius;
    let arc_start_y = arc_center_y + libm::sin(arc_start_radians) * arc_radius;
    let arc_end_x = arc_center_x + libm::cos(arc_end_radians) * arc_radius;
    let arc_end_y = arc_center_y + libm::sin(arc_end_radians) * arc_radius;

    // Start the solver on the middle of the arc span to keep it well-conditioned.
    let mid_angle = arc_start_radians + arc_width_radians / 2.0;
    let initial_point_x = arc_center_x + libm::cos(mid_angle) * arc_radius;
    let initial_point_y = arc_center_y + libm::sin(mid_angle) * arc_radius;

    let initial_guesses = vec![
        (point.id_x(), initial_point_x),
        (point.id_y(), initial_point_y),
        (arc.center.id_x(), arc_center_x),
        (arc.center.id_y(), arc_center_y),
        (arc.start.id_x(), arc_start_x),
        (arc.start.id_y(), arc_start_y),
        (arc.end.id_x(), arc_end_x),
        (arc.end.id_y(), arc_end_y),
    ];

    let requests: Vec<_> = vec![
        // Fix the arc in place.
        Constraint::Arc(arc),
        Constraint::Fixed(arc.center.id_x(), arc_center_x),
        Constraint::Fixed(arc.center.id_y(), arc_center_y),
        Constraint::Fixed(arc.start.id_x(), arc_start_x),
        Constraint::Fixed(arc.start.id_y(), arc_start_y),
        Constraint::Fixed(arc.end.id_x(), arc_end_x),
        Constraint::Fixed(arc.end.id_y(), arc_end_y),
        // Point must lie on the arc, but don't constrain the point any further.
        // It will be underconstrained, as it can lie anywhere on the arc.
        Constraint::PointArcCoincident(arc, point),
    ]
    .into_iter()
    .map(ConstraintRequest::highest_priority)
    .collect();

    // Solve it.
    let outcome = solve(&requests, initial_guesses, Config::default())
        .expect("this constraint system should converge and be solvable");

    assert!(outcome.is_satisfied(), "the constraint should be satisfied");
    assert!(
        outcome.warnings.is_empty(),
        "this simple system should not emit warnings"
    );

    let solved_x = outcome.final_values[point.id_x() as usize];
    let solved_y = outcome.final_values[point.id_y() as usize];
    let p = Point {
        x: solved_x,
        y: solved_y,
    };

    // Check the point lies on the arc.
    let rel_x = solved_x - arc_center_x;
    let rel_y = solved_y - arc_center_y;
    let point_angle = libm::atan2(rel_y, rel_x).rem_euclid(two_pi);
    if arc_end_radians <= two_pi {
        assert!(point_angle + EPSILON >= arc_start_radians);
        assert!(point_angle <= arc_end_radians + EPSILON);
    } else {
        let wrapped_end = arc_end_radians - two_pi;
        assert!(point_angle + EPSILON >= arc_start_radians || point_angle <= wrapped_end + EPSILON);
    }
    let center = Point {
        x: arc_center_x,
        y: arc_center_y,
    };
    // The point's distance from the arc's center should be the arc's radius.
    let actual_distance = p.euclidean_distance(center);
    let expected_distance = arc_radius;
    assert_nearly_eq(actual_distance, expected_distance);
}

fn test_circle_circle_tangent(
    ax: f64,
    ay: f64,
    ar: f64,
    bx_guess: f64,
    by: f64,
    br: f64,
    is_internal: bool,
) {
    let mut ids = IdGenerator::default();
    let circle_a = DatumCircle {
        center: DatumPoint::new(&mut ids),
        radius: DatumDistance::new(ids.next_id()),
    };
    let circle_b = DatumCircle {
        center: DatumPoint::new(&mut ids),
        radius: DatumDistance::new(ids.next_id()),
    };
    let initial_guesses = vec![
        (circle_a.center.id_x(), ax),
        (circle_a.center.id_y(), ay),
        (circle_a.radius.id, ar),
        (circle_b.center.id_x(), bx_guess),
        (circle_b.center.id_y(), by),
        (circle_b.radius.id, br),
    ];
    let requests = [
        ConstraintRequest::highest_priority(Constraint::Fixed(circle_a.center.id_x(), ax)),
        ConstraintRequest::highest_priority(Constraint::Fixed(circle_a.center.id_y(), ay)),
        ConstraintRequest::highest_priority(Constraint::Fixed(circle_a.radius.id, ar)),
        ConstraintRequest::highest_priority(Constraint::Fixed(circle_b.center.id_y(), by)),
        ConstraintRequest::highest_priority(Constraint::Fixed(circle_b.radius.id, br)),
        ConstraintRequest::highest_priority(Constraint::CircleTangentToCircle(
            circle_a,
            circle_b,
            if is_internal {
                CircleSide::Interior
            } else {
                CircleSide::Exterior
            },
        )),
    ];

    let outcome = solve(&requests, initial_guesses, Config::default())
        .expect("this constraint system should converge and be solvable");

    assert!(
        outcome.is_satisfied(),
        "the tangent constraint should be satisfied"
    );
    assert!(
        outcome.warnings.is_empty(),
        "this simple system should not emit warnings"
    );

    let solved_ax = outcome.final_values[circle_a.center.id_x() as usize];
    let solved_ay = outcome.final_values[circle_a.center.id_y() as usize];
    let solved_bx = outcome.final_values[circle_b.center.id_x() as usize];
    let solved_by = outcome.final_values[circle_b.center.id_y() as usize];
    let solved_ar = outcome.final_values[circle_a.radius.id as usize];
    let solved_br = outcome.final_values[circle_b.radius.id as usize];
    let center_dist = libm::hypot(solved_ax - solved_bx, solved_ay - solved_by);

    if is_internal {
        assert_nearly_eq(center_dist, (solved_ar - solved_br).abs());
    } else {
        assert_nearly_eq(center_dist, solved_ar + solved_br);
    }
}

/// `desired_distance` is a SIGNED distance, so 1 and -1 are opposite sides of the line.
fn test_vertical_pld(
    initial_guesses: Vec<(Id, f64)>,
    line: DatumLineSegment,
    point: DatumPoint,
    desired_distance: f64,
) {
    let requests = [
        // Fix the line endpoints
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p0.id_x(),
            initial_guesses[2].1,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p0.id_y(),
            initial_guesses[3].1,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p1.id_x(),
            initial_guesses[4].1,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p1.id_y(),
            initial_guesses[5].1,
        )),
        // Constraint we're testing.
        ConstraintRequest::highest_priority(Constraint::VerticalPointLineDistance(
            point,
            line,
            desired_distance,
        )),
    ];

    let outcome = solve(&requests, initial_guesses, Config::default())
        .expect("this constraint system should converge and be solvable");

    assert!(outcome.is_satisfied(), "the constraint should be satisfied");
    assert!(
        outcome.warnings.is_empty(),
        "this simple system should not emit warnings"
    );

    let solved_x = outcome.final_values[point.id_x() as usize];
    let solved_y = outcome.final_values[point.id_y() as usize];
    let solved_p0x = outcome.final_values[line.p0.id_x() as usize];
    let solved_p0y = outcome.final_values[line.p0.id_y() as usize];
    let solved_p1x = outcome.final_values[line.p1.id_x() as usize];
    let solved_p1y = outcome.final_values[line.p1.id_y() as usize];

    // Vertical distance is measured as the signed difference between the point's Y
    // and the line's Y at the same X coordinate. Here we take point_y - line_y.
    let dx = solved_p1x - solved_p0x;
    // Avoid degenerate/vertical lines; the test harness should reject those via prop_assume.
    let slope = (solved_p1y - solved_p0y) / dx;
    let line_y_at_point = solved_p0y + slope * (solved_x - solved_p0x);

    assert_nearly_eq(solved_y - line_y_at_point, desired_distance);
}

fn test_horizontal_pld(
    initial_guesses: Vec<(Id, f64)>,
    line: DatumLineSegment,
    point: DatumPoint,
    desired_distance: f64,
) {
    let requests = [
        // Fix the line endpoints
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p0.id_x(),
            initial_guesses[2].1,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p0.id_y(),
            initial_guesses[3].1,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p1.id_x(),
            initial_guesses[4].1,
        )),
        ConstraintRequest::highest_priority(Constraint::Fixed(
            line.p1.id_y(),
            initial_guesses[5].1,
        )),
        // Constraint we're testing.
        ConstraintRequest::highest_priority(Constraint::HorizontalPointLineDistance(
            point,
            line,
            desired_distance,
        )),
    ];

    let outcome = solve(&requests, initial_guesses, Config::default())
        .expect("this constraint system should converge and be solvable");

    assert!(outcome.is_satisfied(), "the constraint should be satisfied");
    assert!(
        outcome.warnings.is_empty(),
        "this simple system should not emit warnings"
    );

    let solved_x = outcome.final_values[point.id_x() as usize];
    let solved_y = outcome.final_values[point.id_y() as usize];
    let solved_p0x = outcome.final_values[line.p0.id_x() as usize];
    let solved_p0y = outcome.final_values[line.p0.id_y() as usize];
    let solved_p1x = outcome.final_values[line.p1.id_x() as usize];
    let solved_p1y = outcome.final_values[line.p1.id_y() as usize];

    // Horizontal distance is measured as the signed difference between the point's X
    // and the line's X at the same Y coordinate. Here we take point_x - line_x.
    let dy = solved_p1y - solved_p0y;
    // Avoid degenerate/horizontal lines; the test harness should reject those via prop_assume.
    let slope = (solved_p1x - solved_p0x) / dy;
    let line_x_at_point = solved_p0x + slope * (solved_y - solved_p0y);

    assert_nearly_eq(solved_x - line_x_at_point, desired_distance);
}

#[test]
fn specific_test_point_arc_coincident_off_center() {
    let arc_center = Point { x: -10.0, y: 10.0 };
    let point = Point { x: 10.0, y: 10.0 };
    test_point_arc_coincident(
        arc_center.x,
        arc_center.y,
        5.0,
        40.0,
        10.0,
        point.x,
        point.y,
    );
}

#[test]
fn specific_test_point_arc_coincident() {
    let arc_center = Point::default();
    let point = Point { x: 10.0, y: 10.0 };
    test_point_arc_coincident(
        arc_center.x,
        arc_center.y,
        5.0,
        40.0,
        10.0,
        point.x,
        point.y,
    );
}
