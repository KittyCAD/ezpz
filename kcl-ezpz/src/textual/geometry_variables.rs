use crate::{Id, IdGenerator};

const VARS_PER_POINT: usize = 2;
const VARS_PER_CIRCLE: usize = 3;

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
        self.num_circles += 1;
        self.variables.push((id_generator.next_id(), center_x));
        self.variables.push((id_generator.next_id(), center_y));
        self.variables.push((id_generator.next_id(), radius));
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
}

pub struct PointVars {
    pub x: Id,
    pub y: Id,
}
pub struct CircleVars {
    pub center: PointVars,
    pub radius: Id,
}
