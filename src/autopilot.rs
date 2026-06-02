use thiserror::Error;

/// Configuration and state for a PID-based autopilot.
#[derive(Debug, Clone)]
pub struct Autopilot {
    /// Target heading in degrees (0–360).
    target_heading: f64,
    /// Most recent measured heading in degrees.
    current_heading: f64,
    /// Proportional gain.
    kp: f64,
    /// Integral gain.
    ki: f64,
    /// Derivative gain.
    kd: f64,
    /// Accumulated integral term (anti-windup protected).
    integral: f64,
    /// Previous error for derivative calculation.
    last_error: f64,
    /// Maximum absolute rudder command (degrees).
    max_correction: f64,
    /// Heading tolerance in degrees; within this the vessel is "on course".
    on_course_tolerance: f64,
    /// Maximum integral contribution to prevent windup.
    integral_limit: f64,
    /// Whether this is the first update (no valid last_error).
    first_update: bool,
}

/// Output from a single autopilot update cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AutopilotOutput {
    /// Rudder command in degrees. Positive = starboard, negative = port.
    pub rudder_command: f64,
    /// Signed heading error (target - current), normalised to [-180, 180).
    pub heading_error: f64,
    /// Whether the vessel is within the on-course deadband.
    pub is_on_course: bool,
}

/// Errors that can occur during autopilot operation.
#[derive(Error, Debug, Clone, PartialEq)]
pub enum AutopilotError {
    #[error("invalid heading: {0}")]
    InvalidHeading(f64),
    #[error("dt must be positive, got {0}")]
    InvalidDt(f64),
}

impl Autopilot {
    /// Create a new autopilot with the given PID gains and limits.
    ///
    /// # Arguments
    /// * `kp` - Proportional gain
    /// * `ki` - Integral gain
    /// * `kd` - Derivative gain
    /// * `max_correction` - Maximum absolute rudder angle in degrees
    /// * `on_course_tolerance` - Deadband in degrees for "on course" detection
    pub fn new(kp: f64, ki: f64, kd: f64, max_correction: f64, on_course_tolerance: f64) -> Self {
        Autopilot {
            target_heading: 0.0,
            current_heading: 0.0,
            kp,
            ki,
            kd,
            integral: 0.0,
            last_error: 0.0,
            max_correction: max_correction.abs(),
            on_course_tolerance: on_course_tolerance.abs(),
            integral_limit: 50.0,
            first_update: true,
        }
    }

    /// Set the target heading. Automatically normalises to [0, 360).
    pub fn set_heading(&mut self, target: f64) -> Result<(), AutopilotError> {
        if !target.is_finite() {
            return Err(AutopilotError::InvalidHeading(target));
        }
        self.target_heading = target.rem_euclid(360.0);
        Ok(())
    }

    /// Get the current target heading.
    pub fn target_heading(&self) -> f64 {
        self.target_heading
    }

    /// Get the most recent heading.
    pub fn current_heading(&self) -> f64 {
        self.current_heading
    }

    /// Set the maximum integral contribution.
    pub fn set_integral_limit(&mut self, limit: f64) {
        self.integral_limit = limit.abs();
    }

    /// Compute the shortest angular difference from `from` to `to` in degrees,
    /// normalised to (-180, 180].
    fn heading_error(target: f64, current: f64) -> f64 {
        let mut diff = target - current;
        if diff > 180.0 {
            diff -= 360.0;
        } else if diff <= -180.0 {
            diff += 360.0;
        }
        diff
    }

    /// Update the autopilot with a new heading measurement.
    ///
    /// # Arguments
    /// * `current_heading` - Measured heading in degrees.
    /// * `dt` - Time delta in seconds since the last update.
    ///
    /// Returns the rudder command and heading error.
    pub fn update(&mut self, current_heading: f64, dt: f64) -> Result<AutopilotOutput, AutopilotError> {
        if !current_heading.is_finite() {
            return Err(AutopilotError::InvalidHeading(current_heading));
        }
        if dt <= 0.0 {
            return Err(AutopilotError::InvalidDt(dt));
        }

        self.current_heading = current_heading.rem_euclid(360.0);
        let error = Self::heading_error(self.target_heading, self.current_heading);

        // P term
        let p_term = self.kp * error;

        // I term with anti-windup
        self.integral += error * dt;
        self.integral = self.integral.clamp(-self.integral_limit, self.integral_limit);
        let i_term = self.ki * self.integral;

        // D term
        let d_term = if self.first_update {
            self.first_update = false;
            self.last_error = error;
            0.0
        } else {
            let derivative = (error - self.last_error) / dt;
            self.last_error = error;
            self.kd * derivative
        };

        let mut rudder = p_term + i_term + d_term;

        // Apply rudder limit
        rudder = rudder.clamp(-self.max_correction, self.max_correction);

        // Clamp integral when output is saturated (conditional integration / back-calculation
        // would be more sophisticated, but simple clamping is fine for Layer 0)
        if rudder.abs() >= self.max_correction {
            self.integral -= error * dt; // undo the integration this cycle
            self.integral = self.integral.clamp(-self.integral_limit, self.integral_limit);
        }

        Ok(AutopilotOutput {
            rudder_command: rudder,
            heading_error: error,
            is_on_course: error.abs() <= self.on_course_tolerance,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    #[test]
    fn test_set_heading_normalises() {
        let mut ap = Autopilot::new(1.0, 0.0, 0.0, 30.0, 1.0);
        ap.set_heading(450.0).unwrap();
        assert!((ap.target_heading() - 90.0).abs() < TOL);
    }

    #[test]
    fn test_set_heading_negative() {
        let mut ap = Autopilot::new(1.0, 0.0, 0.0, 30.0, 1.0);
        ap.set_heading(-90.0).unwrap();
        assert!((ap.target_heading() - 270.0).abs() < TOL);
    }

    #[test]
    fn test_set_heading_invalid() {
        let mut ap = Autopilot::new(1.0, 0.0, 0.0, 30.0, 1.0);
        let err = ap.set_heading(f64::NAN).unwrap_err();
        assert!(matches!(err, AutopilotError::InvalidHeading(v) if v.is_nan()));
    }

    #[test]
    fn test_p_only_converges() {
        let mut ap = Autopilot::new(5.0, 0.0, 0.0, 30.0, 1.0);
        ap.set_heading(90.0).unwrap();

        let mut heading = 0.0;
        for _ in 0..500 {
            let out = ap.update(heading, 0.1).unwrap();
            heading += out.rudder_command * 0.02;
        }
        assert!((heading - 90.0).abs() < 0.5);
    }

    #[test]
    fn test_pid_converges_faster() {
        let mut ap = Autopilot::new(5.0, 0.1, 1.0, 30.0, 1.0);
        ap.set_heading(90.0).unwrap();

        let mut heading = 0.0;
        for _ in 0..300 {
            let out = ap.update(heading, 0.1).unwrap();
            heading += out.rudder_command * 0.02;
        }
        assert!((heading - 90.0).abs() < 0.5);
    }

    #[test]
    fn test_on_course_deadband() {
        let mut ap = Autopilot::new(1.0, 0.0, 0.0, 30.0, 2.0);
        ap.set_heading(90.0).unwrap();
        // Exactly on course
        let out = ap.update(90.0, 1.0).unwrap();
        assert!(out.is_on_course);
        assert!(out.rudder_command.abs() < TOL);

        // Slightly off but within deadband
        let out = ap.update(91.5, 1.0).unwrap();
        assert!(out.is_on_course);

        // Outside deadband
        let out = ap.update(93.0, 1.0).unwrap();
        assert!(!out.is_on_course);
    }

    #[test]
    fn test_rudder_limit() {
        let mut ap = Autopilot::new(10.0, 0.0, 0.0, 15.0, 1.0);
        ap.set_heading(180.0).unwrap();
        let out = ap.update(0.0, 1.0).unwrap();
        // Error is ±180, P term would be 1800, clamped to 15
        assert!((out.rudder_command.abs() - 15.0).abs() < TOL);
    }

    #[test]
    fn test_heading_error_shortest_path() {
        // Target 350°, current 10° → error should be -20° (turn left 20° not right 340°)
        let mut ap = Autopilot::new(1.0, 0.0, 0.0, 30.0, 1.0);
        ap.set_heading(350.0).unwrap();
        let out = ap.update(10.0, 1.0).unwrap();
        assert!((out.heading_error - (-20.0)).abs() < TOL);
        assert!(out.rudder_command < 0.0); // port turn
    }

    #[test]
    fn test_heading_error_cross_zero() {
        let mut ap = Autopilot::new(1.0, 0.0, 0.0, 30.0, 1.0);
        ap.set_heading(10.0).unwrap();
        let out = ap.update(350.0, 1.0).unwrap();
        assert!((out.heading_error - 20.0).abs() < TOL);
    }

    #[test]
    fn test_integral_anti_windup() {
        let mut ap = Autopilot::new(0.1, 1.0, 0.0, 10.0, 1.0);
        ap.set_heading(90.0).unwrap();
        ap.set_integral_limit(20.0);

        // Hold at 0° for a long time so integral builds up
        for _ in 0..100 {
            ap.update(0.0, 1.0).unwrap();
        }
        // Integral should be clamped
        assert!(ap.integral.abs() <= 20.0 + TOL);
    }

    #[test]
    fn test_invalid_dt() {
        let mut ap = Autopilot::new(1.0, 0.0, 0.0, 30.0, 1.0);
        ap.set_heading(90.0).unwrap();
        assert_eq!(
            ap.update(0.0, -1.0).unwrap_err(),
            AutopilotError::InvalidDt(-1.0)
        );
        assert_eq!(
            ap.update(0.0, 0.0).unwrap_err(),
            AutopilotError::InvalidDt(0.0)
        );
    }

    #[test]
    fn test_invalid_heading_update() {
        let mut ap = Autopilot::new(1.0, 0.0, 0.0, 30.0, 1.0);
        ap.set_heading(90.0).unwrap();
        assert_eq!(
            ap.update(f64::INFINITY, 1.0).unwrap_err(),
            AutopilotError::InvalidHeading(f64::INFINITY)
        );
    }

    #[test]
    fn test_output_struct() {
        let mut ap = Autopilot::new(1.0, 0.0, 0.0, 30.0, 1.0);
        ap.set_heading(90.0).unwrap();
        let out = ap.update(0.0, 1.0).unwrap();
        assert!(!out.is_on_course);
        assert!((out.heading_error - 90.0).abs() < TOL);
        assert!((out.rudder_command - 30.0).abs() < TOL); // clamped
    }

    #[test]
    fn test_convergence_tight_tolerance() {
        let mut ap = Autopilot::new(5.0, 0.05, 0.5, 30.0, 0.5);
        ap.set_heading(180.0).unwrap();
        let mut heading = 0.0f64;
        for _ in 0..400 {
            let out = ap.update(heading, 0.1).unwrap();
            heading += out.rudder_command * 0.02;
        }
        assert!((heading - 180.0).abs() < 0.5);
    }
}
