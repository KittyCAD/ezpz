#[derive(Clone, Copy, PartialEq, PartialOrd)]
pub(crate) struct V {
    pub x: f64,
    pub y: f64,
}

#[allow(dead_code)]
impl V {
    #[inline(always)]
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    #[inline(always)]
    pub fn magnitude(self) -> f64 {
        libm::hypot(self.x, self.y)
    }

    #[inline(always)]
    pub fn magnitude_squared(self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    #[inline(always)]
    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y
    }

    #[inline(always)]
    pub fn euclidean_distance(self, rhs: Self) -> f64 {
        let d = self - rhs;
        d.magnitude()
    }

    /// <https://stackoverflow.com/questions/243945/calculating-a-2d-vectors-cross-product>
    #[inline(always)]
    pub fn cross_2d(self, rhs: Self) -> f64 {
        self.x * rhs.y - self.y * rhs.x
    }

    #[inline(always)]
    pub fn perp_ccw(self) -> Self {
        Self {
            x: -self.y,
            y: self.x,
        }
    }

    #[inline(always)]
    pub fn perp_cw(self) -> Self {
        Self {
            x: self.y,
            y: -self.x,
        }
    }

    /// Project one vector onto another.
    pub fn project(self, b: Self) -> Self {
        b * (self.dot(b) / b.dot(b))
    }

    /// Rejection is the perpendicular component of one vector w.r.t. another
    pub fn reject(self, b: Self) -> Self {
        self - self.project(b)
    }

    pub fn reflect(self, b: Self) -> Self {
        self - (self.reject(b) * 2.0)
    }

    /// Returns the signed angle between this vector and another. Result is in [-pi, pi].
    pub fn signed_angle(self, b: Self) -> f64 {
        libm::atan2(self.cross_2d(b), self.dot(b))
    }
}

impl std::ops::Sub<Self> for V {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl std::ops::Mul<f64> for V {
    type Output = Self;

    fn mul(self, scale: f64) -> Self::Output {
        Self {
            x: self.x * scale,
            y: self.y * scale,
        }
    }
}

impl std::ops::Div<f64> for V {
    type Output = Self;

    fn div(self, scale: f64) -> Self::Output {
        Self {
            x: self.x / scale,
            y: self.y / scale,
        }
    }
}

impl std::ops::Add for V {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

#[derive(Clone, Copy, PartialEq, PartialOrd)]
pub(crate) struct Rotation2 {
    col0: V,
}

impl Rotation2 {
    #[inline(always)]
    pub fn from_angle_radians(angle: f64) -> Self {
        let (sin, cos) = libm::sincos(angle);
        Self::from_sincos(sin, cos)
    }

    #[inline(always)]
    pub fn from_sincos(sin: f64, cos: f64) -> Self {
        Self {
            col0: V::new(cos, sin),
        }
    }

    #[inline(always)]
    pub fn apply(self, v: V) -> V {
        V {
            x: (self.col0.x * v.x) - (self.col0.y * v.y),
            y: (self.col0.y * v.x) + (self.col0.x * v.y),
        }
    }

    #[inline(always)]
    pub fn inverse(self) -> Self {
        Self {
            col0: V::new(self.col0.x, -self.col0.y),
        }
    }
}
