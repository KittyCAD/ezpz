use crate::{
    datatypes::Angle,
    datatypes::outputs::{Component, Point},
    textual::{
        ScalarGuess,
        instruction::{
            AngleLine, ArcLength, ArcRadius, CircleRadius, DeclareArc, DeclareCircle, Distance,
            FixCenterPointComponent, IsArc, Line, LinesEqualLength, Midpoint, Parallel,
            Perpendicular, PointArcCoincident, PointLineDistance, PointsCoincident, Symmetric,
            Tangent,
        },
    },
};

use super::{
    Label, PointGuess, Problem,
    instruction::{DeclarePoint, FixPointComponent, Horizontal, Instruction, Vertical},
};
use winnow::{
    ModalResult as WResult,
    ascii::{digit1, newline, space0},
    combinator::{alt, delimited, opt, separated},
    error::{ContextError, ErrMode},
    prelude::*,
    stream::AsChar,
    token::take_while,
};

pub fn parse_problem(i: &mut &str) -> WResult<Problem> {
    constraint_header.parse_next(i)?;
    let instructions: Vec<_> = separated(1.., parse_instruction, newline).parse_next(i)?;
    let mut inner_points = Vec::new();
    let mut inner_circles = Vec::new();
    let mut inner_arcs = Vec::new();
    let mut inner_lines = Vec::new();
    for instr in instructions.iter().flatten() {
        if let Instruction::DeclarePoint(dp) = instr {
            inner_points.push(dp.label.clone());
        }
        if let Instruction::DeclareCircle(dc) = instr {
            inner_circles.push(dc.label.clone());
        }
        if let Instruction::DeclareArc(dc) = instr {
            inner_arcs.push(dc.label.clone());
        }
        if let Instruction::Line(line) = instr {
            inner_lines.push((line.p0.clone(), line.p1.clone()));
        }
    }
    newline.parse_next(i)?;
    newline.parse_next(i)?;
    ignore_ws(i);
    guesses_header.parse_next(i)?;
    let guesses: Vec<_> = separated(1.., parse_guess, newline).parse_next(i)?;
    let (scalar_guesses, point_guesses): (Vec<_>, Vec<_>) = guesses.into_iter().fold(
        (Vec::new(), Vec::new()),
        |(mut scalars, mut points), guess| {
            match guess {
                Guess::Point(point_guess) => points.push(point_guess),
                Guess::Scalar(scalar_guess) => scalars.push(scalar_guess),
            };
            (scalars, points)
        },
    );
    opt(newline).parse_next(i)?;
    ignore_ws(i);
    Ok(Problem {
        instructions: instructions.into_iter().flatten().collect(),
        inner_points,
        inner_circles,
        inner_arcs,
        inner_lines,
        point_guesses,
        scalar_guesses,
    })
}

#[derive(Debug)]
enum Guess {
    Point(PointGuess),
    Scalar(ScalarGuess),
}

fn parse_guess(i: &mut &str) -> WResult<Guess> {
    alt((
        parse_point_guess.map(Guess::Point),
        parse_scalar_guess.map(Guess::Scalar),
    ))
    .parse_next(i)
}

// p roughly (0, 0)
pub fn parse_point_guess(i: &mut &str) -> WResult<PointGuess> {
    ignore_ws(i);
    let mut label = parse_label(i)?;
    let suffix = opt(('.', parse_label)).parse_next(i)?;
    if let Some((a, b)) = suffix {
        label.0.push(a);
        label.0.push_str(&b.0);
    }
    ws.parse_next(i)?;
    let _ = "roughly".parse_next(i)?;
    ws.parse_next(i)?;
    let guess = parse_point(i)?;
    Ok(PointGuess {
        point: label,
        guess,
    })
}

// c.radius roughly 4
pub fn parse_scalar_guess(i: &mut &str) -> WResult<ScalarGuess> {
    ignore_ws(i);
    let mut label = parse_label(i)?;
    let suffix = opt(('.', parse_label)).parse_next(i)?;
    if let Some((a, b)) = suffix {
        label.0.push(a);
        label.0.push_str(&b.0);
    }
    ws.parse_next(i)?;
    let _ = "roughly".parse_next(i)?;
    ws.parse_next(i)?;
    let guess = parse_number(i)?;
    Ok(ScalarGuess {
        scalar: label,
        guess,
    })
}

fn constraint_header(i: &mut &str) -> WResult<()> {
    ('#', ws, "constraints", newline).map(|_| ()).parse_next(i)
}
fn guesses_header(i: &mut &str) -> WResult<()> {
    ('#', ws, "guesses", newline).map(|_| ()).parse_next(i)
}

pub fn parse_declare_point(i: &mut &str) -> WResult<DeclarePoint> {
    ("point", ws, parse_label)
        .map(|(_, _, label)| DeclarePoint { label })
        .parse_next(i)
}

pub fn parse_declare_circle(i: &mut &str) -> WResult<DeclareCircle> {
    ("circle", ws, parse_label)
        .map(|(_, _, label)| DeclareCircle { label })
        .parse_next(i)
}

pub fn parse_declare_arc(i: &mut &str) -> WResult<DeclareArc> {
    ("arc", ws, parse_label)
        .map(|(_, _, label)| DeclareArc { label })
        .parse_next(i)
}

pub fn parse_horizontal(i: &mut &str) -> WResult<Horizontal> {
    let _ = "horizontal".parse_next(i)?;
    ignore_ws(i);
    let [p0, p1] = inside_brackets(two_points, i)?;
    Ok(Horizontal { label: (p0, p1) })
}

pub fn parse_coincident(i: &mut &str) -> WResult<PointsCoincident> {
    let _ = "coincident".parse_next(i)?;
    ignore_ws(i);
    let [point0, point1] = inside_brackets(two_points, i)?;
    Ok(PointsCoincident { point0, point1 })
}

pub fn parse_point_arc_coincident(i: &mut &str) -> WResult<PointArcCoincident> {
    let _ = "point_arc_coincident".parse_next(i)?;
    ignore_ws(i);
    let [point, arc] = inside_brackets(two_points, i)?;
    Ok(PointArcCoincident { point, arc })
}

pub fn parse_midpoint(i: &mut &str) -> WResult<Midpoint> {
    let _ = "midpoint".parse_next(i)?;
    ignore_ws(i);
    let [point0, point1, mp] = inside_brackets(three_points, i)?;
    Ok(Midpoint { point0, point1, mp })
}

pub fn parse_point_line_distance(i: &mut &str) -> WResult<PointLineDistance> {
    let _ = "point_line_distance".parse_next(i)?;
    ignore_ws(i);
    let (point, line_p0, line_p1, distance) = inside_brackets(three_labels_num, i)?;
    Ok(PointLineDistance {
        point,
        line_p0,
        line_p1,
        distance,
    })
}

pub fn parse_symmetric(i: &mut &str) -> WResult<Symmetric> {
    let _ = "symmetric".parse_next(i)?;
    ignore_ws(i);
    let [line_p, line_q, a, b] = inside_brackets(four_points, i)?;
    Ok(Symmetric {
        line: (line_p, line_q),
        p0: a,
        p1: b,
    })
}

pub fn parse_vertical(i: &mut &str) -> WResult<Vertical> {
    let _ = "vertical".parse_next(i)?;
    ignore_ws(i);
    let [p0, p1] = inside_brackets(two_points, i)?;
    Ok(Vertical { label: (p0, p1) })
}

pub fn parse_distance(i: &mut &str) -> WResult<Distance> {
    let _ = "distance".parse_next(i)?;
    ignore_ws(i);
    let ([p0, p1], _, distance) = inside_brackets((two_points, commasep, parse_number_expr), i)?;
    Ok(Distance {
        label: (p0, p1),
        distance,
    })
}

pub fn commasep(i: &mut &str) -> WResult<()> {
    ignore_ws(i);
    ','.parse_next(i)?;
    ignore_ws(i);
    Ok(())
}

pub fn parse_angle_line(i: &mut &str) -> WResult<AngleLine> {
    let _ = "lines_at_angle".parse_next(i)?;
    ignore_ws(i);
    let ([p0, p1, p2, p3], _, angle) = inside_brackets((four_points, commasep, parse_angle), i)?;
    let line0 = (p0, p1);
    let line1 = (p2, p3);
    Ok(AngleLine {
        line0,
        line1,
        angle,
    })
}

pub fn parse_angle(i: &mut &str) -> WResult<Angle> {
    let value = parse_number(i)?;
    let is_degrees = alt(("deg".map(|_| true), "rad".map(|_| false))).parse_next(i)?;
    Ok(if is_degrees {
        Angle::from_degrees(value)
    } else {
        Angle::from_radians(value)
    })
}

pub fn parse_parallel(i: &mut &str) -> WResult<Parallel> {
    let _ = "parallel".parse_next(i)?;
    ignore_ws(i);
    let [p0, p1, p2, p3] = inside_brackets(four_points, i)?;
    let line0 = (p0, p1);
    let line1 = (p2, p3);
    Ok(Parallel { line0, line1 })
}

pub fn parse_circle_radius(i: &mut &str) -> WResult<CircleRadius> {
    let _ = "radius".parse_next(i)?;
    ignore_ws(i);
    let (circle, _, radius) = inside_brackets((parse_label, commasep, parse_number_expr), i)?;
    Ok(CircleRadius { circle, radius })
}

pub fn parse_tangent(i: &mut &str) -> WResult<Tangent> {
    let _ = "tangent".parse_next(i)?;
    ignore_ws(i);
    let (line_p0, _, line_p1, _, circle) = inside_brackets(
        (parse_label, commasep, parse_label, commasep, parse_label),
        i,
    )?;
    Ok(Tangent {
        circle,
        line_p0,
        line_p1,
    })
}

pub fn parse_arc_radius(i: &mut &str) -> WResult<ArcRadius> {
    let _ = "arc_radius".parse_next(i)?;
    ignore_ws(i);
    let (arc_label, _, radius) = inside_brackets((parse_label, commasep, parse_number), i)?;
    Ok(ArcRadius { arc_label, radius })
}

pub fn parse_arc_length(i: &mut &str) -> WResult<ArcLength> {
    let _ = "arc_length".parse_next(i)?;
    ignore_ws(i);
    let (arc, _, distance) = inside_brackets((parse_label, commasep, parse_number), i)?;
    Ok(ArcLength { arc, distance })
}

pub fn parse_is_arc(i: &mut &str) -> WResult<IsArc> {
    let _ = "is_arc".parse_next(i)?;
    ignore_ws(i);
    let arc_label = inside_brackets(parse_label, i)?;
    Ok(IsArc { arc_label })
}

pub fn parse_line(i: &mut &str) -> WResult<Line> {
    let _ = "line".parse_next(i)?;
    ignore_ws(i);
    let (p0, _, p1) = inside_brackets((parse_label, commasep, parse_label), i)?;
    Ok(Line { p0, p1 })
}

pub fn parse_lines_equal_length(i: &mut &str) -> WResult<LinesEqualLength> {
    let _ = "lines_equal_length".parse_next(i)?;
    ignore_ws(i);
    let [p0, p1, p2, p3] = inside_brackets(four_points, i)?;
    let line0 = (p0, p1);
    let line1 = (p2, p3);
    Ok(LinesEqualLength { line0, line1 })
}

pub fn parse_perpendicular(i: &mut &str) -> WResult<Perpendicular> {
    let _ = "perpendicular".parse_next(i)?;
    ignore_ws(i);
    let [p0, p1, p2, p3] = inside_brackets(four_points, i)?;
    let line0 = (p0, p1);
    let line1 = (p2, p3);
    Ok(Perpendicular { line0, line1 })
}

/// Runs the given parser, surrounded by parentheses.
fn inside_brackets<'i, T>(
    mut parser: impl Parser<&'i str, T, ErrMode<ContextError>>,
    i: &mut &'i str,
) -> WResult<T> {
    let _ = '('.parse_next(i)?;
    ignore_ws(i);
    let t = parser.parse_next(i)?;
    let _ = ')'.parse_next(i)?;
    Ok(t)
}

fn four_points(i: &mut &str) -> WResult<[Label; 4]> {
    let p0 = parse_label(i)?;
    commasep(i)?;
    let p1 = parse_label(i)?;
    commasep(i)?;
    let p2 = parse_label(i)?;
    commasep(i)?;
    let p3 = parse_label(i)?;
    ignore_ws(i);
    Ok([p0, p1, p2, p3])
}

fn two_points(i: &mut &str) -> WResult<[Label; 2]> {
    let p0 = parse_label(i)?;
    commasep(i)?;
    let p1 = parse_label(i)?;
    ignore_ws(i);
    Ok([p0, p1])
}

fn three_points(i: &mut &str) -> WResult<[Label; 3]> {
    let p0 = parse_label(i)?;
    commasep(i)?;
    let p1 = parse_label(i)?;
    commasep(i)?;
    let p2 = parse_label(i)?;
    ignore_ws(i);
    Ok([p0, p1, p2])
}

fn three_labels_num(i: &mut &str) -> WResult<(Label, Label, Label, f64)> {
    let p = parse_label(i)?;
    commasep(i)?;
    let lp0 = parse_label(i)?;
    commasep(i)?;
    let lp1 = parse_label(i)?;
    commasep(i)?;
    let d = parse_number(i)?;
    ignore_ws(i);
    Ok((p, lp0, lp1, d))
}

/// Single-element vector
fn sv<T>(t: T) -> Vec<T> {
    vec![t]
}

fn parse_instruction(i: &mut &str) -> WResult<Vec<Instruction>> {
    ignore_ws(i);
    alt((
        parse_declare_point.map(Instruction::DeclarePoint).map(sv),
        parse_declare_circle.map(Instruction::DeclareCircle).map(sv),
        parse_declare_arc.map(Instruction::DeclareArc).map(sv),
        parse_fix_point_component
            .map(Instruction::FixPointComponent)
            .map(sv),
        parse_fix_center_point_component
            .map(Instruction::FixCenterPointComponent)
            .map(sv),
        assign_point,
        parse_horizontal.map(Instruction::Horizontal).map(sv),
        parse_coincident.map(Instruction::PointsCoincident).map(sv),
        parse_point_arc_coincident
            .map(Instruction::PointArcCoincident)
            .map(sv),
        parse_midpoint.map(Instruction::Midpoint).map(sv),
        parse_symmetric.map(Instruction::Symmetric).map(sv),
        parse_vertical.map(Instruction::Vertical).map(sv),
        parse_other_instructions,
    ))
    .parse_next(i)
}

fn parse_other_instructions(i: &mut &str) -> WResult<Vec<Instruction>> {
    alt((
        parse_distance.map(Instruction::Distance).map(sv),
        parse_parallel.map(Instruction::Parallel).map(sv),
        parse_perpendicular.map(Instruction::Perpendicular).map(sv),
        parse_angle_line.map(Instruction::AngleLine).map(sv),
        parse_circle_radius.map(Instruction::CircleRadius).map(sv),
        parse_tangent.map(Instruction::Tangent).map(sv),
        parse_arc_radius.map(Instruction::ArcRadius).map(sv),
        parse_arc_length.map(Instruction::ArcLength).map(sv),
        parse_is_arc.map(Instruction::IsArc).map(sv),
        parse_point_line_distance
            .map(Instruction::PointLineDistance)
            .map(sv),
        parse_line.map(Instruction::Line).map(sv),
        parse_lines_equal_length
            .map(Instruction::LinesEqualLength)
            .map(sv),
    ))
    .parse_next(i)
}

fn ws(i: &mut &str) -> WResult<()> {
    space0.parse_next(i).map(|_| ())
}

fn ignore_ws(i: &mut &str) {
    let _ = ws.parse_next(i);
}

fn assign_point(i: &mut &str) -> WResult<Vec<Instruction>> {
    // p0 = (0, 0)
    let label = parse_label_opt_suffix(i)?;
    ignore_ws(i);
    '='.parse_next(i)?;
    ignore_ws(i);
    let pt = parse_point(i)?;
    Ok(vec![
        Instruction::FixPointComponent(FixPointComponent {
            point: label.clone(),
            component: Component::X,
            value: pt.x,
        }),
        Instruction::FixPointComponent(FixPointComponent {
            point: label,
            component: Component::Y,
            value: pt.y,
        }),
    ])
}

fn parse_component(i: &mut &str) -> WResult<Component> {
    alt(('x'.map(|_| Component::X), 'y'.map(|_| Component::Y))).parse_next(i)
}

fn parse_fix_point_component(i: &mut &str) -> WResult<FixPointComponent> {
    (
        parse_label,
        '.',
        parse_component,
        delimited(space0, '=', space0),
        parse_number,
    )
        .map(
            |(label, _dot, component, _equals, value)| FixPointComponent {
                point: label,
                component,
                value,
            },
        )
        .parse_next(i)
}

fn parse_label(i: &mut &str) -> WResult<Label> {
    take_while(1.., AsChar::is_alphanum)
        .map(|s: &str| Label(s.to_owned()))
        .parse_next(i)
}

fn parse_label_opt_suffix(i: &mut &str) -> WResult<Label> {
    let mut label = parse_label(i)?;
    let suffix = opt(('.', parse_label)).parse_next(i)?;
    if let Some((a, b)) = suffix {
        label.0.push(a);
        label.0.push_str(&b.0);
    }
    Ok(label)
}

pub fn parse_point(input: &mut &str) -> WResult<Point> {
    inside_brackets(
        (parse_number, ',', space0, parse_number).map(|(x, _comma, _space, y)| Point { x, y }),
        input,
    )
}

fn parse_fix_center_point_component(i: &mut &str) -> WResult<FixCenterPointComponent> {
    (
        parse_label,
        ".center.",
        parse_component,
        delimited(space0, '=', space0),
        parse_number,
    )
        .map(
            |(label, _dot, component, _equals, value)| FixCenterPointComponent {
                object: label,
                center_component: component,
                value,
            },
        )
        .parse_next(i)
}

fn parse_number(i: &mut &str) -> WResult<f64> {
    fn myint(input: &mut &str) -> WResult<f64> {
        digit1
            .verify_map(|s: &str| s.parse::<f64>().ok())
            .parse_next(input)
    }

    fn myfloat(i: &mut &str) -> WResult<f64> {
        winnow::ascii::float.parse_next(i)
    }
    alt((myfloat, myint)).parse_next(i)
}

fn parse_number_expr(i: &mut &str) -> WResult<f64> {
    alt((
        parse_number,
        ("sqrt(", parse_number_expr, ')').map(|(_, num, _)| num.sqrt()),
    ))
    .parse_next(i)
}

#[cfg(test)]
mod tests {
    use crate::tests::assert_nearly_eq;

    use super::*;

    #[test]
    fn test_parse_angle() {
        let i = parse_angle(&mut "0deg").unwrap();
        let j = parse_angle(&mut "0rad").unwrap();
        assert_nearly_eq(i.to_degrees(), j.to_degrees());
    }
}
