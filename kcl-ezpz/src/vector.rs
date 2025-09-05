#[derive(Clone, Copy, PartialEq, PartialOrd)]
pub(crate) struct V {
    pub x: f64,
    pub y: f64,
}

impl V {
    #[inline(always)]
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    #[inline(always)]
    pub fn magnitude(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }

    #[inline(always)]
    pub fn magnitude_squared(&self) -> f64 {
        self.x.powi(2) + self.y.powi(2)
    }

    #[inline(always)]
    pub fn dot(&self, rhs: &Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y
    }

    #[inline(always)]
    pub fn euclidean_distance(self, rhs: Self) -> f64 {
        let d = self - rhs;
        d.magnitude()
    }

    /// <https://stackoverflow.com/questions/243945/calculating-a-2d-vectors-cross-product>
    #[inline(always)]
    pub fn cross_2d(&self, rhs: &Self) -> f64 {
        self.x * rhs.y - self.y * rhs.x
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
