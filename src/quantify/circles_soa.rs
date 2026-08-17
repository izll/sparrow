use jagua_rs::geometry::primitives::Circle;

/// Number of lanes the SoA buffers are padded to.
/// Padding to a fixed multiple lets the (auto)vectorized overlap-proxy loops run without a scalar remainder loop.
pub const SOA_LANES: usize = 8;

/// Collection of circles, but with a memory layout that's more suitable for SIMD operations:
/// SoA (Structure of Arrays) instead of AoS (Array of Structures).
///
/// The buffers are always padded to a multiple of [`SOA_LANES`] with "null" circles (radius 0 at the origin).
/// These contribute exactly `0.0` to the overlap proxy (their contribution is multiplied by `min(r1, 0) == 0`),
/// so callers can safely iterate over the full (padded) length.
#[derive(Debug, Clone)]
#[repr(align(32))]
pub struct CirclesSoA {
    pub x: Vec<f32>,
    pub y: Vec<f32>,
    pub r: Vec<f32>,
    /// Number of "real" circles (excluding padding)
    pub n: usize,
}

impl Default for CirclesSoA {
    fn default() -> Self {
        Self::new()
    }
}

impl CirclesSoA {
    pub fn new() -> Self {
        Self {
            x: Vec::new(),
            y: Vec::new(),
            r: Vec::new(),
            n: 0,
        }
    }

    pub fn from_circles(circles: &[Circle]) -> Self {
        let mut soa = Self::new();
        soa.load(circles);
        soa
    }

    /// Loads the circles into the SoA buffers (reusing the existing allocations), padding to a multiple of [`SOA_LANES`].
    pub fn load(&mut self, circles: &[Circle]) -> &mut Self {
        let n = circles.len();
        let padded_len = n.div_ceil(SOA_LANES) * SOA_LANES;

        // (re)size the buffers, the tail beyond `n` acts as padding with null circles
        self.x.clear();
        self.y.clear();
        self.r.clear();
        self.x.resize(padded_len, 0.0);
        self.y.resize(padded_len, 0.0);
        self.r.resize(padded_len, 0.0);

        for (i, c) in circles.iter().enumerate() {
            self.x[i] = c.center.0;
            self.y[i] = c.center.1;
            self.r[i] = c.radius;
        }
        self.n = n;

        debug_assert!(self.x.len().is_multiple_of(SOA_LANES));
        self
    }

    /// Length of the (padded) buffers
    #[inline(always)]
    pub fn padded_len(&self) -> usize {
        self.x.len()
    }
}
