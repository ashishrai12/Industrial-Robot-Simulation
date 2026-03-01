module Kinematics

"""
Forward and Inverse Kinematics for a 2-link planar robot arm.
Implementation in Julia for high-performance numerical computing.
"""

struct RobotArm
    l1::Float64
    l2::Float64
end

"""
    forward_kinematics(arm, theta1, theta2)

Calculate the (x, y) coordinates of the end-effector.
"""
function forward_kinematics(arm::RobotArm, theta1::Float64, theta2::Float64)
    x = arm.l1 * cos(theta1) + arm.l2 * cos(theta1 + theta2)
    y = arm.l1 * sin(theta1) + arm.l2 * sin(theta1 + theta2)
    return (x, y)
end

"""
    inverse_kinematics(arm, target_x, target_y)

Calculate theta1 and theta2 required to reach (target_x, target_y).
Returns a Tuple of angles or nothing if unreachable.
"""
function inverse_kinematics(arm::RobotArm, x::Float64, y::Float64)
    d_sq = x^2 + y^2
    cos_theta2 = (d_sq - arm.l1^2 - arm.l2^2) / (2 * arm.l1 * arm.l2)
    
    if abs(cos_theta2) > 1.0
        return nothing
    end
    
    theta2 = acos(cos_theta2)
    theta1 = atan(y, x) - atan(arm.l2 * sin(theta2), arm.l1 + arm.l2 * cos(theta2))
    
    return (theta1, theta2)
end

export RobotArm, forward_kinematics, inverse_kinematics

end

# Example usage/test
using .Kinematics
arm = RobotArm(1.0, 1.0)
println("Testing Julia Kinematics...")
pos = forward_kinematics(arm, 0.0, 0.0)
println("FK(0, 0) -> ", pos)
angles = inverse_kinematics(arm, 2.0, 0.0)
println("IK(2, 0) -> ", angles)
