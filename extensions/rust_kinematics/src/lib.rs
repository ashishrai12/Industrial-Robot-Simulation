/// Forward and Inverse Kinematics for a 2-link planar robot arm.
/// Rust implementation focusing on safety and performance.

pub struct RobotArm {
    pub l1: f64,
    pub l2: f64,
}

impl RobotArm {
    pub fn new(l1: f64, l2: f64) -> Self {
        RobotArm { l1, l2 }
    }

    /// Calculates the (x, y) position of the end-effector.
    pub fn forward_kinematics(&self, theta1: f64, theta2: f64) -> (f64, f64) {
        let x = self.l1 * theta1.cos() + self.l2 * (theta1 + theta2).cos();
        let y = self.l1 * theta1.sin() + self.l2 * (theta1 + theta2).sin();
        (x, y)
    }

    /// Calculates theta1 and theta2 required to reach (target_x, target_y).
    /// Returns None if the target is unreachable.
    pub fn inverse_kinematics(&self, x: f64, y: f64) -> Option<(f64, f64)> {
        let d_sq = x * x + y * y;
        let cos_theta2 = (d_sq - self.l1 * self.l1 - self.l2 * self.l2) / (2.0 * self.l1 * self.l2);

        if cos_theta2.abs() > 1.0 {
            return None;
        }

        let theta2 = cos_theta2.acos();
        let theta1 = y.atan2(x) - (self.l2 * theta2.sin()).atan2(self.l1 + self.l2 * theta2.cos());

        Some((theta1, theta2))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_forward() {
        let arm = RobotArm::new(1.0, 1.0);
        let (x, y) = arm.forward_kinematics(0.0, 0.0);
        assert!((x - 2.0).abs() < 1e-10);
        assert!((y - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_inverse() {
        let arm = RobotArm::new(1.0, 1.0);
        if let Some((t1, t2)) = arm.inverse_kinematics(2.0, 0.0) {
            assert!(t1.abs() < 1e-10);
            assert!(t2.abs() < 1e-10);
        } else {
            panic!("Should be reachable");
        }
    }
}
