use super::{
    Component, Label, Point, PointGuess, Problem,
    instruction::{DeclarePoint, FixPointComponent, Horizontal, Instruction, Vertical},
};
use winnow::{
    Result as WResult,
    ascii::{alphanumeric1, digit1, newline, space0},
    combinator::{alt, delimited, eof, opt, separated},
    prelude::*,
};

impl Problem {
    pub fn parse(i: &mut &str) -> WResult<Self> {
        constraint_header.parse_next(i)?;
        let instructions: Vec<_> = separated(1.., Instruction::parse, newline).parse_next(i)?;
        let mut inner_points = Vec::new();
        for instr in &instructions {
            if let Instruction::DeclarePoint(dp) = instr {
                inner_points.push(dp.label.clone());
            }
        }
        newline.parse_next(i)?;
        newline.parse_next(i)?;
        ignore_ws(i);
        guesses_header.parse_next(i)?;
        let point_guesses: Vec<_> = separated(1.., PointGuess::parse, newline).parse_next(i)?;
        opt(newline).parse_next(i)?;
        ignore_ws(i);
        eof.parse_next(i)?;
        Ok(Self {
            instructions,
            inner_points,
            point_guesses,
        })
    }
}

impl PointGuess {
    // p roughly (0, 0)
    pub fn parse(i: &mut &str) -> WResult<Self> {
        ignore_ws(i);
        let label = Label::parse(i)?;
        ws.parse_next(i)?;
        let _ = "roughly".parse_next(i)?;
        ws.parse_next(i)?;
        let guess = Point::parse(i)?;
        Ok(Self {
            point: label,
            guess,
        })
    }
}

fn constraint_header(i: &mut &str) -> WResult<()> {
    ('#', ws, "constraints", newline).map(|_| ()).parse_next(i)
}
fn guesses_header(i: &mut &str) -> WResult<()> {
    ('#', ws, "guesses", newline).map(|_| ()).parse_next(i)
}

impl DeclarePoint {
    pub fn parse(i: &mut &str) -> WResult<Self> {
        ("point", ws, Label::parse)
            .map(|(_, _, label)| Self { label })
            .parse_next(i)
    }
}

impl Horizontal {
    pub fn parse(i: &mut &str) -> WResult<Self> {
        let _ = "horizontal".parse_next(i)?;
        ignore_ws(i);
        let _ = '('.parse_next(i)?;
        ignore_ws(i);
        let p0 = Label::parse(i)?;
        let _ = ','.parse_next(i)?;
        ignore_ws(i);
        let p1 = Label::parse(i)?;
        let _ = ')'.parse_next(i)?;
        Ok(Self { label: (p0, p1) })
    }
}

impl Vertical {
    pub fn parse(i: &mut &str) -> WResult<Self> {
        let _ = "vertical".parse_next(i)?;
        ignore_ws(i);
        let _ = '('.parse_next(i)?;
        ignore_ws(i);
        let p0 = Label::parse(i)?;
        let _ = ','.parse_next(i)?;
        ignore_ws(i);
        let p1 = Label::parse(i)?;
        let _ = ')'.parse_next(i)?;
        Ok(Self { label: (p0, p1) })
    }
}

impl Instruction {
    fn parse(i: &mut &str) -> WResult<Self> {
        ignore_ws(i);
        alt((
            DeclarePoint::parse.map(Instruction::DeclarePoint),
            FixPointComponent::parse.map(Instruction::FixPointComponent),
            Horizontal::parse.map(Instruction::Horizontal),
            Vertical::parse.map(Instruction::Vertical),
        ))
        .parse_next(i)
    }
}

fn ws(i: &mut &str) -> WResult<()> {
    space0.parse_next(i).map(|_| ())
}

fn ignore_ws(i: &mut &str) {
    let _ = ws.parse_next(i);
}

impl Component {
    fn parse(i: &mut &str) -> WResult<Self> {
        alt(('x'.map(|_| Self::X), 'y'.map(|_| Self::Y))).parse_next(i)
    }
}

impl FixPointComponent {
    fn parse(i: &mut &str) -> WResult<FixPointComponent> {
        (
            Label::parse,
            '.',
            Component::parse,
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
}

impl Label {
    fn parse(i: &mut &str) -> WResult<Label> {
        alphanumeric1
            .map(|s: &str| Label(s.to_owned()))
            .parse_next(i)
    }
}

impl Point {
    pub fn parse(input: &mut &str) -> WResult<Point> {
        delimited(
            '(',
            (parse_number, ',', space0, parse_number).map(|(x, _comma, _space, y)| Point { x, y }),
            ')',
        )
        .parse_next(input)
    }
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
