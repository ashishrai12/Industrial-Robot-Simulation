/**
 * Forward and Inverse Kinematics for a 2-link planar robot arm.
 * JavaScript implementation for potential web-based visualizations or edge computing.
 */
class PlanarKinematics2D {
    /**
     * @param {number} l1 - Length of the first link
     * @param {number} l2 - Length of the second link
     */
    constructor(l1, l2) {
        this.l1 = l1;
        this.l2 = l2;
    }

    /**
     * Calculates the (x, y) position of the end effector.
     * @param {number} theta1 - Angle of the first joint in radians
     * @param {number} theta2 - Angle of the second joint in radians
     * @returns {{x: number, y: number}} The coordinates of the end effector.
     */
    forward(theta1, theta2) {
        const x = this.l1 * Math.cos(theta1) + this.l2 * Math.cos(theta1 + theta2);
        const y = this.l1 * Math.sin(theta1) + this.l2 * Math.sin(theta1 + theta2);
        return { x, y };
    }

    /**
     * Calculates theta1 and theta2 required to reach (targetX, targetY).
     * @param {number} targetX - Target X coordinate
     * @param {number} targetY - Target Y coordinate
     * @param {boolean} elbowUp - Whether to use the elbow-up configuration
     * @returns {{theta1: number, theta2: number} | null} The required angles, or null if unreachable.
     */
    inverse(targetX, targetY, elbowUp = true) {
        const dSq = targetX ** 2 + targetY ** 2;

        // Law of cosines for theta2
        const cosTheta2 = (dSq - this.l1 ** 2 - this.l2 ** 2) / (2 * this.l1 * this.l2);

        if (Math.abs(cosTheta2) > 1.0) {
            return null; // Target out of reach
        }

        let theta2 = Math.acos(cosTheta2);
        if (!elbowUp) {
            theta2 = -theta2;
        }

        const theta1 = Math.atan2(targetY, targetX) - Math.atan2(this.l2 * Math.sin(theta2), this.l1 + this.l2 * Math.cos(theta2));

        return { theta1, theta2 };
    }
}

// Example usage and rudimentary tests
if (require.main === module) {
    const kin = new PlanarKinematics2D(1.0, 1.0);
    console.log("Testing JavaScript Kinematics...");

    // Test Forward Kinematics
    const { x, y } = kin.forward(0, 0);
    console.log(`FK(0, 0) -> x: ${x.toFixed(2)}, y: ${y.toFixed(2)}`);

    // Test Inverse Kinematics
    const angles = kin.inverse(2, 0);
    if (angles) {
        console.log(`IK(2, 0) -> theta1: ${angles.theta1.toFixed(4)}, theta2: ${angles.theta2.toFixed(4)}`);
    } else {
        console.log("IK failed: Unreachable point");
    }
}

module.exports = PlanarKinematics2D;
