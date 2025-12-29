/**
 * @module dynamics/physics
 * @description Quadrotor Rigid Body Dynamics Simulation
 * 
 * Implements Newton-Euler equations for 3D quadrotor physics:
 * - Linear dynamics: m * a = F_thrust + F_drag + F_gravity
 * - Angular dynamics: I * omega_dot = tau - omega × (I * omega)
 */

import type { QuadrotorState3D, QuadrotorParams, Vector3Array, Quaternion } from './types';
import type { ControlCommand } from '../control/types';

/**
 * Main physics update function
 * Integrates quadrotor dynamics over one timestep
 * 
 * @param state - Current quadrotor state
 * @param control - Control command (thrust + torque)
 * @param params - Quadrotor physical parameters
 * @param dt - Time step (seconds)
 * @returns Updated quadrotor state
 */
export function quadrotorPhysicsUpdate(
    state: QuadrotorState3D,
    control: ControlCommand,
    params: QuadrotorParams,
    dt: number
): QuadrotorState3D {
    const { position, velocity, quaternion, angularVelocity } = state;
    const { mass, inertia, dragCoeff, gravity } = params;

    // ========== Linear Dynamics ==========

    // Thrust force in body frame is [0, 0, thrust]
    const thrustBodyFrame: Vector3Array = [0, 0, control.thrust];

    // Rotate thrust to world frame
    const thrustWorldFrame = rotateVectorByQuaternion(thrustBodyFrame, quaternion);

    // Drag force (linear approximation: F_drag = -k * v)
    const dragForce: Vector3Array = [
        -dragCoeff[0] * velocity[0],
        -dragCoeff[1] * velocity[1],
        -dragCoeff[2] * velocity[2],
    ];

    // Gravity force
    const gravityForce: Vector3Array = [0, 0, -mass * gravity];

    // Total force
    const totalForce: Vector3Array = [
        thrustWorldFrame[0] + dragForce[0] + gravityForce[0],
        thrustWorldFrame[1] + dragForce[1] + gravityForce[1],
        thrustWorldFrame[2] + dragForce[2] + gravityForce[2],
    ];

    // Linear acceleration: a = F / m
    const linearAcceleration: Vector3Array = [
        totalForce[0] / mass,
        totalForce[1] / mass,
        totalForce[2] / mass,
    ];

    // ========== Angular Dynamics ==========

    // Euler's rotation equation: I * omega_dot = tau - omega × (I * omega)
    const Iomega: Vector3Array = [
        inertia[0] * angularVelocity[0],
        inertia[1] * angularVelocity[1],
        inertia[2] * angularVelocity[2],
    ];

    const omegaCrossIomega = cross(angularVelocity, Iomega);

    const angularAcceleration: Vector3Array = [
        (control.torque[0] - omegaCrossIomega[0]) / inertia[0],
        (control.torque[1] - omegaCrossIomega[1]) / inertia[1],
        (control.torque[2] - omegaCrossIomega[2]) / inertia[2],
    ];

    // ========== Integration (Semi-implicit Euler) ==========

    // Update velocity first (uses current acceleration)
    const newVelocity: Vector3Array = [
        velocity[0] + linearAcceleration[0] * dt,
        velocity[1] + linearAcceleration[1] * dt,
        velocity[2] + linearAcceleration[2] * dt,
    ];

    // Update position (uses new velocity)
    const newPosition: Vector3Array = [
        position[0] + newVelocity[0] * dt,
        position[1] + newVelocity[1] * dt,
        position[2] + newVelocity[2] * dt,
    ];

    // Update angular velocity
    const newAngularVelocity: Vector3Array = [
        angularVelocity[0] + angularAcceleration[0] * dt,
        angularVelocity[1] + angularAcceleration[1] * dt,
        angularVelocity[2] + angularAcceleration[2] * dt,
    ];

    // Update quaternion (uses new angular velocity)
    const newQuaternion = quaternionIntegration(quaternion, newAngularVelocity, dt);

    return {
        position: newPosition,
        velocity: newVelocity,
        quaternion: newQuaternion,
        angularVelocity: newAngularVelocity,
    };
}

/**
 * Rotate a vector by a quaternion
 * v' = q * v * q^(-1)
 * 
 * @param v - Vector to rotate
 * @param q - Rotation quaternion [w, x, y, z]
 * @returns Rotated vector
 */
export function rotateVectorByQuaternion(v: Vector3Array, q: Quaternion): Vector3Array {
    const [w, qx, qy, qz] = q;
    const [vx, vy, vz] = v;

    // Compute q * v * q^(-1) using expanded formula for efficiency
    // This avoids creating temporary quaternions

    // First compute q * [0, v]
    const t0 = -qx * vx - qy * vy - qz * vz;
    const t1 = w * vx + qy * vz - qz * vy;
    const t2 = w * vy + qz * vx - qx * vz;
    const t3 = w * vz + qx * vy - qy * vx;

    // Then compute (q * v) * q^(-1) = (q * v) * [w, -qx, -qy, -qz]
    return [
        t1 * w - t0 * qx - t2 * qz + t3 * qy,
        t2 * w - t0 * qy - t3 * qx + t1 * qz,
        t3 * w - t0 * qz - t1 * qy + t2 * qx,
    ];
}

/**
 * Quaternion time integration using exponential map
 * q(t+dt) = q(t) * exp(0.5 * omega * dt)
 * 
 * For small angles: exp(0.5 * omega * dt) ≈ [1, 0.5*omega*dt]
 * 
 * @param q - Current quaternion
 * @param omega - Angular velocity in body frame [rad/s]
 * @param dt - Time step
 * @returns Integrated quaternion (normalized)
 */
export function quaternionIntegration(
    q: Quaternion,
    omega: Vector3Array,
    dt: number
): Quaternion {
    const halfDt = 0.5 * dt;

    // Angular velocity magnitude
    const omegaMag = Math.sqrt(omega[0] ** 2 + omega[1] ** 2 + omega[2] ** 2);

    let deltaQ: Quaternion;

    if (omegaMag > 1e-10) {
        // Use exact exponential map for larger rotations
        const halfAngle = 0.5 * omegaMag * dt;
        const sinHalfAngle = Math.sin(halfAngle);
        const cosHalfAngle = Math.cos(halfAngle);
        const axis: Vector3Array = [
            omega[0] / omegaMag,
            omega[1] / omegaMag,
            omega[2] / omegaMag,
        ];
        deltaQ = [
            cosHalfAngle,
            axis[0] * sinHalfAngle,
            axis[1] * sinHalfAngle,
            axis[2] * sinHalfAngle,
        ];
    } else {
        // Small angle approximation
        deltaQ = [
            1,
            omega[0] * halfDt,
            omega[1] * halfDt,
            omega[2] * halfDt,
        ];
    }

    // q_new = q * deltaQ
    const newQ = quaternionMultiply(q, deltaQ);

    // Normalize to prevent drift
    return normalizeQuaternion(newQ);
}

/**
 * Hamilton product of two quaternions
 */
export function quaternionMultiply(q1: Quaternion, q2: Quaternion): Quaternion {
    const [w1, x1, y1, z1] = q1;
    const [w2, x2, y2, z2] = q2;

    return [
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ];
}

/**
 * Normalize quaternion to unit length
 */
export function normalizeQuaternion(q: Quaternion): Quaternion {
    const norm = Math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2);
    if (norm < 1e-10) {
        return [1, 0, 0, 0]; // Return identity if degenerate
    }
    return [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm];
}

/**
 * Quaternion conjugate (for unit quaternion, this equals inverse)
 */
export function quaternionConjugate(q: Quaternion): Quaternion {
    return [q[0], -q[1], -q[2], -q[3]];
}

/**
 * Convert quaternion to rotation matrix (3x3)
 */
export function quaternionToRotationMatrix(q: Quaternion): number[][] {
    const [w, x, y, z] = q;

    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ];
}

/**
 * Cross product of two 3D vectors
 */
export function cross(a: Vector3Array, b: Vector3Array): Vector3Array {
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ];
}

/**
 * Dot product of two 3D vectors
 */
export function dot(a: Vector3Array, b: Vector3Array): number {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/**
 * Vector addition
 */
export function addVectors(a: Vector3Array, b: Vector3Array): Vector3Array {
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

/**
 * Vector subtraction
 */
export function subtractVectors(a: Vector3Array, b: Vector3Array): Vector3Array {
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

/**
 * Scalar multiplication
 */
export function scaleVector(v: Vector3Array, s: number): Vector3Array {
    return [v[0] * s, v[1] * s, v[2] * s];
}

/**
 * Vector magnitude
 */
export function magnitude(v: Vector3Array): number {
    return Math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2);
}

/**
 * Create identity quaternion
 */
export function identityQuaternion(): Quaternion {
    return [1, 0, 0, 0];
}

/**
 * Create initial hover state at a given position
 */
export function createHoverState(position: Vector3Array): QuadrotorState3D {
    return {
        position,
        velocity: [0, 0, 0],
        quaternion: identityQuaternion(),
        angularVelocity: [0, 0, 0],
    };
}

/**
 * Compute hover thrust for a given mass and gravity
 */
export function computeHoverThrust(mass: number, gravity: number): number {
    return mass * gravity;
}
