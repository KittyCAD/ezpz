use crate::{Id, IdGenerator, textual::Point};

const VARS_PER_POINT: usize = 2;
const VARS_PER_CIRCLE: usize = 3;
const VARS_PER_ARC: usize = 6;

/// Stores variables for different constrainable geometry.
#[derive(Default, Clone)]
pub struct GeometryVariables {
    /// List of variables, each with an ID and a value.
    // Layout of this vec:
    // - All variables for points are stored first,
    //   then all variables for circles.
    // - For a point, its variables are stored `[x, y]`.
    // - For a circle, its variables are stored `[center_x, center_y, radius]`.
    // So for example, storing two points and a circle would be
    // `[point0_x, point0_y, point1_x, point1_y, circle_x, circle_y, circle_radius]`
    variables: Vec<(Id, f64)>,
    num_points: usize,
    num_circles: usize,
    num_arcs: usize,
}

impl GeometryVariables {
    /// How many variables are stored?
    pub fn len(&self) -> usize {
        self.variables.len()
    }

    pub fn variables(&self) -> Vec<(Id, f64)> {
        self.variables.clone()
    }

    /// Add a single variable.
    fn push_scalar(&mut self, id_generator: &mut IdGenerator, guess: f64) {
        self.variables.push((id_generator.next_id(), guess));
    }

    /// Add variables for a 2D point.
    /// Must be called before `push_circle`.
    pub fn push_point(&mut self, id_generator: &mut IdGenerator, x: f64, y: f64) {
        if self.num_circles > 0 {
            panic!("You must add points before circles");
        }
        if self.num_arcs > 0 {
            panic!("You must add points before arcs");
        }
        self.num_points += 1;
        self.push_scalar(id_generator, x);
        self.push_scalar(id_generator, y);
    }

    /// Add variables for a circle.
    /// Once you call this, you cannot push normal 2D point anymore.
    pub fn push_circle(
        &mut self,
        id_generator: &mut IdGenerator,
        center_x: f64,
        center_y: f64,
        radius: f64,
    ) {
        if self.num_arcs > 0 {
            panic!("You must add circles before arcs");
        }
        self.num_circles += 1;
        self.variables.push((id_generator.next_id(), center_x));
        self.variables.push((id_generator.next_id(), center_y));
        self.variables.push((id_generator.next_id(), radius));
    }

    /// Add variables for a arc.
    /// Once you call this, you cannot push 2D points or circles anymore.
    pub fn push_arc(&mut self, id_generator: &mut IdGenerator, p: Point, q: Point, center: Point) {
        self.num_arcs += 1;
        let c = center;
        self.variables.push((id_generator.next_id(), p.x));
        self.variables.push((id_generator.next_id(), p.y));
        self.variables.push((id_generator.next_id(), q.x));
        self.variables.push((id_generator.next_id(), q.y));
        self.variables.push((id_generator.next_id(), c.x));
        self.variables.push((id_generator.next_id(), c.y));
    }

    /// Look up the variables for a given 2D point.
    pub fn get_point_ids(&self, point_id: usize) -> PointVars {
        let x = self.variables[VARS_PER_POINT * point_id].0;
        let y = self.variables[VARS_PER_POINT * point_id + 1].0;
        PointVars { x, y }
    }

    /// Look up the variables for a given circle.
    pub fn get_circle_ids(&self, circle_id: usize) -> CircleVars {
        let start_of_circles = VARS_PER_POINT * self.num_points;
        let x = self.variables[start_of_circles + VARS_PER_CIRCLE * circle_id].0;
        let y = self.variables[start_of_circles + VARS_PER_CIRCLE * circle_id + 1].0;
        let radius = self.variables[start_of_circles + VARS_PER_CIRCLE * circle_id + 2].0;
        CircleVars {
            center: PointVars { x, y },
            radius,
        }
    }

    /// Look up the variables for a given arc.
    pub fn get_arc_ids(&self, arc_id: usize) -> ArcVars {
        let start_of_arcs = VARS_PER_POINT * self.num_points;
        let px = self.variables[start_of_arcs + VARS_PER_ARC * arc_id].0;
        let py = self.variables[start_of_arcs + VARS_PER_ARC * arc_id + 1].0;
        let p = PointVars { x: px, y: py };
        let qx = self.variables[start_of_arcs + VARS_PER_ARC * arc_id + 2].0;
        let qy = self.variables[start_of_arcs + VARS_PER_ARC * arc_id + 3].0;
        let q = PointVars { x: qx, y: qy };
        let cx = self.variables[start_of_arcs + VARS_PER_ARC * arc_id + 4].0;
        let cy = self.variables[start_of_arcs + VARS_PER_ARC * arc_id + 5].0;
        let center = PointVars { x: cx, y: cy };
        ArcVars { p, q, center }
    }
}

pub struct PointVars {
    pub x: Id,
    pub y: Id,
}

pub struct CircleVars {
    pub center: PointVars,
    pub radius: Id,
}

pub struct ArcVars {
    pub p: PointVars,
    pub q: PointVars,
    pub center: PointVars,
}
