/// State returned by a deadband check.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DeadbandState {
    /// Value is within acceptable bounds.
    Normal,
    /// Value is approaching the boundary (for deadbands that support approaching).
    Approaching,
    /// Value has exceeded the deadband.
    Exceeded,
}

/// Heading deadband — monitors whether a heading is within tolerance of a target.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HeadingDeadband {
    pub target: f64,
    pub tolerance_degrees: f64,
}

impl HeadingDeadband {
    /// Create a new heading deadband.
    ///
    /// `tolerance_degrees` is the half-width of the deadband (e.g. 5.0 means ±5°).
    /// The `approaching` threshold is at 1.5× tolerance (configurable via `check_approaching`).
    pub fn new(target: f64, tolerance_degrees: f64) -> Self {
        HeadingDeadband {
            target,
            tolerance_degrees: tolerance_degrees.abs(),
        }
    }

    /// Check a heading value against the deadband.
    pub fn check(&self, value: f64) -> DeadbandState {
        let diff = shortest_angular_distance(self.target, value);
        if diff.abs() <= self.tolerance_degrees {
            DeadbandState::Normal
        } else {
            DeadbandState::Exceeded
        }
    }

    /// Check with an approaching zone at `approach_multiple` × tolerance.
    pub fn check_approaching(&self, value: f64, approach_multiple: f64) -> DeadbandState {
        let diff = shortest_angular_distance(self.target, value);
        let abs_diff = diff.abs();
        if abs_diff <= self.tolerance_degrees {
            DeadbandState::Normal
        } else if abs_diff <= self.tolerance_degrees * approach_multiple {
            DeadbandState::Approaching
        } else {
            DeadbandState::Exceeded
        }
    }
}

/// Depth deadband — percentage-based tolerance for depth.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DepthDeadband {
    pub target: f64,
    pub tolerance_pct: f64,
}

impl DepthDeadband {
    /// `target` is the nominal depth in metres.
    /// `tolerance_pct` is the allowable deviation as a fraction (e.g. 0.1 = ±10%).
    pub fn new(target: f64, tolerance_pct: f64) -> Self {
        DepthDeadband {
            target,
            tolerance_pct: tolerance_pct.abs(),
        }
    }

    /// Check a depth measurement against the deadband.
    pub fn check(&self, value: f64) -> DeadbandState {
        if self.target.abs() < 1e-12 {
            // If target is zero, use absolute tolerance based on target magnitude
            return if value.abs() <= 0.1 {
                DeadbandState::Normal
            } else {
                DeadbandState::Exceeded
            };
        }
        let deviation = (value - self.target).abs() / self.target;
        if deviation <= self.tolerance_pct {
            DeadbandState::Normal
        } else {
            DeadbandState::Exceeded
        }
    }

    /// Check with an approaching zone.
    pub fn check_approaching(&self, value: f64, approach_multiple: f64) -> DeadbandState {
        if self.target.abs() < 1e-12 {
            return self.check(value);
        }
        let deviation = (value - self.target).abs() / self.target;
        if deviation <= self.tolerance_pct {
            DeadbandState::Normal
        } else if deviation <= self.tolerance_pct * approach_multiple {
            DeadbandState::Approaching
        } else {
            DeadbandState::Exceeded
        }
    }
}

/// Speed deadband — absolute tolerance in knots.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpeedDeadband {
    pub target: f64,
    pub tolerance_knots: f64,
}

impl SpeedDeadband {
    /// `target` is the nominal speed in knots.
    /// `tolerance_knots` is the absolute allowable deviation.
    pub fn new(target: f64, tolerance_knots: f64) -> Self {
        SpeedDeadband {
            target,
            tolerance_knots: tolerance_knots.abs(),
        }
    }

    /// Check a speed measurement against the deadband.
    pub fn check(&self, value: f64) -> DeadbandState {
        let deviation = (value - self.target).abs();
        if deviation <= self.tolerance_knots {
            DeadbandState::Normal
        } else {
            DeadbandState::Exceeded
        }
    }

    /// Check with an approaching zone.
    pub fn check_approaching(&self, value: f64, approach_multiple: f64) -> DeadbandState {
        let deviation = (value - self.target).abs();
        if deviation <= self.tolerance_knots {
            DeadbandState::Normal
        } else if deviation <= self.tolerance_knots * approach_multiple {
            DeadbandState::Approaching
        } else {
            DeadbandState::Exceeded
        }
    }

    /// One-sided check: only flag if value is below target minus tolerance.
    pub fn check_low(&self, value: f64) -> DeadbandState {
        let diff = self.target - value;
        if diff <= self.tolerance_knots {
            DeadbandState::Normal
        } else {
            DeadbandState::Exceeded
        }
    }

    /// One-sided check: only flag if value is above target plus tolerance.
    pub fn check_high(&self, value: f64) -> DeadbandState {
        let diff = value - self.target;
        if diff <= self.tolerance_knots {
            DeadbandState::Normal
        } else {
            DeadbandState::Exceeded
        }
    }
}

/// Compute the shortest angular distance between two headings in degrees.
fn shortest_angular_distance(a: f64, b: f64) -> f64 {
    let mut diff = b - a;
    if diff > 180.0 {
        diff -= 360.0;
    } else if diff <= -180.0 {
        diff += 360.0;
    }
    diff
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- Heading deadband ----
    #[test]
    fn test_heading_normal() {
        let db = HeadingDeadband::new(90.0, 5.0);
        assert_eq!(db.check(90.0), DeadbandState::Normal);
        assert_eq!(db.check(93.0), DeadbandState::Normal);
        assert_eq!(db.check(85.0), DeadbandState::Normal);
    }

    #[test]
    fn test_heading_exceeded() {
        let db = HeadingDeadband::new(90.0, 5.0);
        assert_eq!(db.check(96.0), DeadbandState::Exceeded);
        assert_eq!(db.check(84.0), DeadbandState::Exceeded);
    }

    #[test]
    fn test_heading_wraparound_normal() {
        let db = HeadingDeadband::new(5.0, 10.0);
        assert_eq!(db.check(0.0), DeadbandState::Normal);
        assert_eq!(db.check(359.0), DeadbandState::Normal); // 6° difference, within ±10°
    }

    #[test]
    fn test_heading_wraparound_exceeded() {
        let db = HeadingDeadband::new(5.0, 5.0);
        assert_eq!(db.check(355.0), DeadbandState::Exceeded);
    }

    #[test]
    fn test_heading_approaching() {
        let db = HeadingDeadband::new(90.0, 5.0);
        assert_eq!(db.check_approaching(90.0, 2.0), DeadbandState::Normal);
        assert_eq!(db.check_approaching(96.0, 2.0), DeadbandState::Approaching);
        assert_eq!(db.check_approaching(97.0, 2.0), DeadbandState::Approaching);
        assert_eq!(db.check_approaching(101.0, 2.0), DeadbandState::Exceeded);
    }

    // ---- Depth deadband ----
    #[test]
    fn test_depth_normal() {
        let db = DepthDeadband::new(10.0, 0.1);
        assert_eq!(db.check(10.0), DeadbandState::Normal);
        assert_eq!(db.check(11.0), DeadbandState::Normal);
        assert_eq!(db.check(9.0), DeadbandState::Normal);
    }

    #[test]
    fn test_depth_exceeded() {
        let db = DepthDeadband::new(10.0, 0.1);
        assert_eq!(db.check(12.0), DeadbandState::Exceeded);
        assert_eq!(db.check(8.0), DeadbandState::Exceeded);
    }

    #[test]
    fn test_depth_target_zero() {
        let db = DepthDeadband::new(0.0, 0.1);
        assert_eq!(db.check(0.0), DeadbandState::Normal);
        assert_eq!(db.check(0.05), DeadbandState::Normal);
        assert_eq!(db.check(0.2), DeadbandState::Exceeded);
    }

    #[test]
    fn test_depth_approaching() {
        let db = DepthDeadband::new(10.0, 0.1);
        assert_eq!(db.check_approaching(12.0, 1.5), DeadbandState::Exceeded); // 20% > 15%
        assert_eq!(db.check_approaching(11.5, 1.5), DeadbandState::Approaching); // 15%, right on boundary
        assert_eq!(db.check_approaching(11.0, 1.5), DeadbandState::Normal); // 10%
    }

    // ---- Speed deadband ----
    #[test]
    fn test_speed_normal() {
        let db = SpeedDeadband::new(12.0, 1.0);
        assert_eq!(db.check(12.0), DeadbandState::Normal);
        assert_eq!(db.check(12.5), DeadbandState::Normal);
        assert_eq!(db.check(11.0), DeadbandState::Normal);
    }

    #[test]
    fn test_speed_exceeded() {
        let db = SpeedDeadband::new(12.0, 1.0);
        assert_eq!(db.check(13.1), DeadbandState::Exceeded);
        assert_eq!(db.check(10.9), DeadbandState::Exceeded);
    }

    #[test]
    fn test_speed_approaching() {
        let db = SpeedDeadband::new(12.0, 1.0);
        assert_eq!(db.check_approaching(12.0, 2.0), DeadbandState::Normal);
        assert_eq!(db.check_approaching(13.5, 2.0), DeadbandState::Approaching); // 1.5kts → 1.5x tolerance
        assert_eq!(db.check_approaching(14.1, 2.0), DeadbandState::Exceeded);
    }

    #[test]
    fn test_speed_one_sided_low() {
        let db = SpeedDeadband::new(12.0, 1.0);
        assert_eq!(db.check_low(11.0), DeadbandState::Normal);
        assert_eq!(db.check_low(10.0), DeadbandState::Exceeded);
        assert_eq!(db.check_low(13.0), DeadbandState::Normal); // high is fine for low check
    }

    #[test]
    fn test_speed_one_sided_high() {
        let db = SpeedDeadband::new(12.0, 1.0);
        assert_eq!(db.check_high(13.0), DeadbandState::Normal);
        assert_eq!(db.check_high(14.0), DeadbandState::Exceeded);
        assert_eq!(db.check_high(10.0), DeadbandState::Normal); // low is fine for high check
    }

    // ---- Shortest angular distance ----
    #[test]
    fn test_shortest_angular_distance_basic() {
        assert!((shortest_angular_distance(0.0, 90.0) - 90.0).abs() < 1e-9);
        assert!((shortest_angular_distance(90.0, 0.0) - (-90.0)).abs() < 1e-9);
    }

    #[test]
    fn test_shortest_angular_distance_wraparound() {
        assert!((shortest_angular_distance(350.0, 10.0) - 20.0).abs() < 1e-9);
        assert!((shortest_angular_distance(10.0, 350.0) - (-20.0)).abs() < 1e-9);
    }
}
