
import { Pose, Velocity } from './types'
import { normalizeAngle } from './utils'

/**
 * Differential drive kinematics
 * Two-wheeled robot where each wheel can spin independently
 */
export function differentialDriveUpdate(
    pose: Pose,
    leftVel: number,
    rightVel: number,
    wheelBase: number,
    dt: number
): Pose {
    // Calculate linear and angular velocity from wheel speeds
    const v = (leftVel + rightVel) / 2
    const omega = (rightVel - leftVel) / wheelBase

    // Update pose using kinematic equations
    if (Math.abs(omega) < 1e-6) {
        // Straight line motion
        return {
            x: pose.x + v * Math.cos(pose.theta) * dt,
            y: pose.y + v * Math.sin(pose.theta) * dt,
            theta: pose.theta
        }
    } else {
        // Arc motion
        const R = v / omega
        const newTheta = pose.theta + omega * dt
        return {
            x: pose.x + R * (Math.sin(newTheta) - Math.sin(pose.theta)),
            y: pose.y + R * (Math.cos(pose.theta) - Math.cos(newTheta)),
            theta: normalizeAngle(newTheta)
        }
    }
}

/**
 * Omnidirectional kinematics
 * Can move in any direction without rotating first
 */
export function omniUpdate(
    pose: Pose,
    vx: number,
    vy: number,
    omega: number,
    dt: number
): Pose {
    // Transform local velocity to global frame
    const cos_t = Math.cos(pose.theta)
    const sin_t = Math.sin(pose.theta)

    const vx_global = cos_t * vx - sin_t * vy
    const vy_global = sin_t * vx + cos_t * vy

    return {
        x: pose.x + vx_global * dt,
        y: pose.y + vy_global * dt,
        theta: normalizeAngle(pose.theta + omega * dt)
    }
}

/**
 * Ackermann steering kinematics
 * Car-like steering with front wheel steering angle
 */
export function ackermannUpdate(
    pose: Pose,
    velocity: number,
    steeringAngle: number,
    wheelBase: number,
    dt: number
): Pose {
    // Limit steering angle to realistic range
    const maxSteering = Math.PI / 4 // 45 degrees
    steeringAngle = Math.max(-maxSteering, Math.min(maxSteering, steeringAngle))

    if (Math.abs(steeringAngle) < 1e-6) {
        // Straight motion
        return {
            x: pose.x + velocity * Math.cos(pose.theta) * dt,
            y: pose.y + velocity * Math.sin(pose.theta) * dt,
            theta: pose.theta
        }
    } else {
        // Curved motion
        const turnRadius = wheelBase / Math.tan(steeringAngle)
        const omega = velocity / turnRadius
        const newTheta = pose.theta + omega * dt

        return {
            x: pose.x + turnRadius * (Math.sin(newTheta) - Math.sin(pose.theta)),
            y: pose.y + turnRadius * (Math.cos(pose.theta) - Math.cos(newTheta)),
            theta: normalizeAngle(newTheta)
        }
    }
}

/**
 * Quadrotor kinematics update
 * Simulates 2D projection of quadrotor flight with attitude-based control
 * Uses velocity commands: vx, vy for translation, omega for yaw rotation
 * The quadrotor can move in any direction instantaneously (holonomic)
 */
export function quadrotorUpdate(
    pose: Pose,
    vx: number,      // Forward velocity (body frame)
    vy: number,      // Lateral velocity (body frame)  
    omega: number,   // Yaw rate
    dt: number
): Pose {
    // Convert body-frame velocities to world-frame
    const cosTheta = Math.cos(pose.theta)
    const sinTheta = Math.sin(pose.theta)

    // World-frame velocities
    const vxWorld = vx * cosTheta - vy * sinTheta
    const vyWorld = vx * sinTheta + vy * cosTheta

    // Update position and orientation
    return {
        x: pose.x + vxWorld * dt,
        y: pose.y + vyWorld * dt,
        theta: normalizeAngle(pose.theta + omega * dt)
    }
}

/**
 * Apply odometry noise model (α-model)
 * Based on probabilistic robotics odometry motion model
 */
export function applyKinematicsNoise(
    prevPose: Pose,
    newPose: Pose,
    alpha: [number, number, number, number]
): Pose {
    const [a1, a2, a3, a4] = alpha

    // Compute odometry motion
    const dx = newPose.x - prevPose.x
    const dy = newPose.y - prevPose.y
    const dRot1 = Math.atan2(dy, dx) - prevPose.theta
    const dTrans = Math.sqrt(dx * dx + dy * dy)
    const dRot2 = newPose.theta - prevPose.theta - dRot1

    // Add noise using gaussianRandom (needs import or move here)
    // Importing from './utils' to avoid circular dependency
    const dRot1Hat = dRot1 + gaussianRandom() * Math.sqrt(a1 * dRot1 * dRot1 + a2 * dTrans * dTrans)
    const dTransHat = dTrans + gaussianRandom() * Math.sqrt(a3 * dTrans * dTrans + a4 * (dRot1 * dRot1 + dRot2 * dRot2))
    const dRot2Hat = dRot2 + gaussianRandom() * Math.sqrt(a1 * dRot2 * dRot2 + a2 * dTrans * dTrans)

    // Compute noisy pose
    return {
        x: prevPose.x + dTransHat * Math.cos(prevPose.theta + dRot1Hat),
        y: prevPose.y + dTransHat * Math.sin(prevPose.theta + dRot1Hat),
        theta: normalizeAngle(prevPose.theta + dRot1Hat + dRot2Hat)
    }
}

function gaussianRandom(): number {
    let u = 0, v = 0
    while (u === 0) u = Math.random() // Converting [0,1) to (0,1)
    while (v === 0) v = Math.random()
    return Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v)
}
