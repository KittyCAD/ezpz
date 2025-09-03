use crate::textual::instruction::{AngleLine, Distance, Parallel, Perpendicular};

use super::{
    Component, Label, Point, PointGuess, Problem,
    instruction::{DeclarePoint, FixPointComponent, Horizontal, Instruction, Vertical},
};
use kittycad_modeling_cmds::shared::Angle;
use winnow::{
    ModalResult as WResult,
    ascii::{alphanumeric1, digit1, newline, space0},
    combinator::{alt, delimited, opt, separated},
    error::{ContextError, ErrMode},
    prelude::*,
};

pub fn parse_problem(i: &mut &str) -> WResult<Problem> {
    constraint_header.parse_next(i)?;
    let instructions: Vec<_> = separated(1.., parse_instruction, newline).parse_next(i)?;
    let mut inner_points = Vec::new();
    for instr in instructions.iter().flatten() {
        if let Instruction::DeclarePoint(dp) = instr {
            inner_points.push(dp.label.clone());
        }
    }
    newline.parse_next(i)?;
    newline.parse_next(i)?;
    ignore_ws(i);
    guesses_header.parse_next(i)?;
    let point_guesses: Vec<_> = separated(1.., parse_point_guess, newline).parse_next(i)?;
    opt(newline).parse_next(i)?;
    ignore_ws(i);
    Ok(Problem {
        instructions: instructions.into_iter().flatten().collect(),
        inner_points,
        point_guesses,
    })
}

// p roughly (0, 0)
pub fn parse_point_guess(i: &mut &str) -> WResult<PointGuess> {
    ignore_ws(i);
    let label = parse_label(i)?;
    ws.parse_next(i)?;
    let _ = "roughly".parse_next(i)?;
    ws.parse_next(i)?;
    let guess = parse_point(i)?;
    Ok(PointGuess {
        point: label,
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

pub fn parse_horizontal(i: &mut &str) -> WResult<Horizontal> {
    let _ = "horizontal".parse_next(i)?;
    ignore_ws(i);
    let [p0, p1] = inside_brackets(two_points, i)?;
    Ok(Horizontal { label: (p0, p1) })
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

/// Single-element vector
fn sv<T>(t: T) -> Vec<T> {
    vec![t]
}

fn parse_instruction(i: &mut &str) -> WResult<Vec<Instruction>> {
    ignore_ws(i);
    alt((
        parse_declare_point.map(Instruction::DeclarePoint).map(sv),
        parse_fix_point_component
            .map(Instruction::FixPointComponent)
            .map(sv),
        assign_point,
        parse_horizontal.map(Instruction::Horizontal).map(sv),
        parse_vertical.map(Instruction::Vertical).map(sv),
        parse_distance.map(Instruction::Distance).map(sv),
        parse_parallel.map(Instruction::Parallel).map(sv),
        parse_perpendicular.map(Instruction::Perpendicular).map(sv),
        parse_angle_line.map(Instruction::AngleLine).map(sv),
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
    let label = parse_label(i)?;
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
            point: label.clone(),
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
    alphanumeric1
        .map(|s: &str| Label(s.to_owned()))
        .parse_next(i)
}

pub fn parse_point(input: &mut &str) -> WResult<Point> {
    inside_brackets(
        (parse_number, ',', space0, parse_number).map(|(x, _comma, _space, y)| Point { x, y }),
        input,
    )
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
    use super::*;

    #[test]
    fn test_parse_angle() {
        let i = parse_angle(&mut "0deg").unwrap();
        let j = parse_angle(&mut "0rad").unwrap();
        assert_eq!(i.to_degrees(), j.to_degrees());
    }
}
