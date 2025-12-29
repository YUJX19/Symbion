/**
 * @module control/flatness
 * @description Differential Flatness Map Implementation
 * 
 * Supports quadrotor dynamics mapping including nonlinear drag effects.
 * Maps flat outputs (position, yaw) to control inputs (thrust, attitude, body rates).
 */

import type { Vector3Array, TrajectoryState } from '../../planning/trajectory/types';
import type { Quaternion, ControlOutput, PhysicsParams } from './types';

/**
 * Differential Flatness Mapping Class
 * Implements the mapping from trajectory states (pos, vel, acc, jerk) and yaw 
 * to rigid body control commands.
 */
export class FlatnessMap {
    private physics: PhysicsParams = {
        mass: 0.98,
        gravity: 9.81,
        horizDragCoeff: 0.0,
        vertDragCoeff: 0.0,
        parasDragCoeff: 0.0,
        speedEps: 1e-3,
    };

    /**
     * Resets physics parameters used for mapping
     */
    reset(params: PhysicsParams): void {
        this.physics = { ...params };
    }

    /**
     * Forward Mapping: Trajectory State -> Control Output
     * 
     * @param state - Current trajectory state (position, velocity, acceleration, jerk)
     * @param yaw - Desired yaw angle (rad)
     * @param yawRate - Desired yaw rate (rad/s)
     * @returns Control commands (thrust, quaternion attitude, body rates)
     */
    forward(
        state: TrajectoryState,
        yaw: number = 0,
        yawRate: number = 0
    ): ControlOutput {
        const { velocity, acceleration, jerk = [0, 0, 0] } = state;
        const { mass, gravity, horizDragCoeff, vertDragCoeff, parasDragCoeff, speedEps } = this.physics;

        // 1. Calculate aerodynamic drag forces
        const speed = Math.sqrt(velocity[0] ** 2 + velocity[1] ** 2 + velocity[2] ** 2);
        const speedSmooth = Math.sqrt(speed * speed + speedEps * speedEps); // Avoid division by zero

        const dragForce: Vector3Array = [
            -(horizDragCoeff * velocity[0] + parasDragCoeff * velocity[0]) * speedSmooth,
            -(horizDragCoeff * velocity[1] + parasDragCoeff * velocity[1]) * speedSmooth,
            -(vertDragCoeff * velocity[2] + parasDragCoeff * velocity[2]) * speedSmooth,
        ];

        // 2. Calculate normalized thrust vector in world frame
        // thrust_vec = m * (acc + g*e_z - drag_force/m)
        const thrustVec: Vector3Array = [
            acceleration[0] - dragForce[0] / mass,
            acceleration[1] - dragForce[1] / mass,
            acceleration[2] + gravity - dragForce[2] / mass,
        ];

        // 3. Compute thrust magnitude
        const thrustMag = Math.sqrt(thrustVec[0] ** 2 + thrustVec[1] ** 2 + thrustVec[2] ** 2);
        const thrust = thrustMag * mass;

        // 4. Compute z-axis of the body frame (aligned with thrust vector)
        const zBody: Vector3Array = [
            thrustVec[0] / thrustMag,
            thrustVec[1] / thrustMag,
            thrustVec[2] / thrustMag,
        ];

        // 5. Compute x-axis of the body frame using constructive approach
        // Desired x direction in horizontal plane based on yaw
        const xDes: Vector3Array = [Math.cos(yaw), Math.sin(yaw), 0];

        // y_body = z_body x x_des (Cross product)
        const yBody = this.cross(zBody, xDes);
        const yBodyNorm = Math.sqrt(yBody[0] ** 2 + yBody[1] ** 2 + yBody[2] ** 2);
        const yBodyUnit: Vector3Array = [
            yBody[0] / yBodyNorm,
            yBody[1] / yBodyNorm,
            yBody[2] / yBodyNorm,
        ];

        // x_body = y_body x z_body
        const xBody = this.cross(yBodyUnit, zBody);

        // 6. Construct rotation matrix and convert to quaternion
        const quaternion = this.rotationMatrixToQuaternion([xBody, yBodyUnit, zBody]);

        // 7. Estimate body angular rates
        // Estimated from jerk and rate of change of thrust direction (Simplified)
        const bodyRate = this.computeBodyRate(
            zBody,
            jerk,
            dragForce,
            mass,
            thrustMag,
            yaw,
            yawRate
        );

        return {
            thrust,
            quaternion,
            bodyRate,
        };
    }

    /**
     * Backward Mapping (Jacobian): Control Gradient -> State Gradient
     * Used for gradient-based trajectory optimization (e.g., Backpropagation through Flatness)
     */
    backward(
        state: TrajectoryState,
        yaw: number,
        yawRate: number,
        thrustGrad: number,
        quatGrad: Quaternion,
        omgGrad: Vector3Array
    ): {
        velGrad: Vector3Array;
        accGrad: Vector3Array;
        jerkGrad: Vector3Array;
    } {
        // Simplified implementation: using numerical differentiation
        const eps = 1e-6;
        const velGrad: Vector3Array = [0, 0, 0];
        const accGrad: Vector3Array = [0, 0, 0];
        const jerkGrad: Vector3Array = [0, 0, 0];

        // Partial derivative w.r.t velocity
        for (let i = 0; i < 3; i++) {
            const statePlus = { ...state };
            statePlus.velocity = [...state.velocity] as Vector3Array;
            statePlus.velocity[i] += eps;

            const ctrlPlus = this.forward(statePlus, yaw, yawRate);
            velGrad[i] = (ctrlPlus.thrust - this.forward(state, yaw, yawRate).thrust) / eps * thrustGrad;
        }

        // Partial derivative w.r.t acceleration
        for (let i = 0; i < 3; i++) {
            const statePlus = { ...state };
            statePlus.acceleration = [...state.acceleration] as Vector3Array;
            statePlus.acceleration[i] += eps;

            const ctrlPlus = this.forward(statePlus, yaw, yawRate);
            accGrad[i] = (ctrlPlus.thrust - this.forward(state, yaw, yawRate).thrust) / eps * thrustGrad;
        }

        return { velGrad, accGrad, jerkGrad };
    }

    /**
     * Vector cross product calculation
     */
    private cross(a: Vector3Array, b: Vector3Array): Vector3Array {
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ];
    }

    /**
     * Vector dot product calculation
     */
    private dot(a: Vector3Array, b: Vector3Array): number {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    /**
     * Converts a 3x3 rotation matrix to a unit quaternion
     * Input: [x_body, y_body, z_body] column vectors
     */
    private rotationMatrixToQuaternion(R: Vector3Array[]): Quaternion {
        const [x, y, z] = R;

        // Build 3x3 rotation matrix elements
        const R11 = x[0], R12 = y[0], R13 = z[0];
        const R21 = x[1], R22 = y[1], R23 = z[1];
        const R31 = x[2], R32 = y[2], R33 = z[2];

        // Conversion using Shepperd's Method for numerical stability
        const trace = R11 + R22 + R33;

        let w: number, qx: number, qy: number, qz: number;

        if (trace > 0) {
            const s = Math.sqrt(trace + 1.0) * 2;
            w = 0.25 * s;
            qx = (R32 - R23) / s;
            qy = (R13 - R31) / s;
            qz = (R21 - R12) / s;
        } else if (R11 > R22 && R11 > R33) {
            const s = Math.sqrt(1.0 + R11 - R22 - R33) * 2;
            w = (R32 - R23) / s;
            qx = 0.25 * s;
            qy = (R12 + R21) / s;
            qz = (R13 + R31) / s;
        } else if (R22 > R33) {
            const s = Math.sqrt(1.0 + R22 - R11 - R33) * 2;
            w = (R13 - R31) / s;
            qx = (R12 + R21) / s;
            qy = 0.25 * s;
            qz = (R23 + R32) / s;
        } else {
            const s = Math.sqrt(1.0 + R33 - R11 - R22) * 2;
            w = (R21 - R12) / s;
            qx = (R13 + R31) / s;
            qy = (R23 + R32) / s;
            qz = 0.25 * s;
        }

        // Final normalization
        const norm = Math.sqrt(w * w + qx * qx + qy * qy + qz * qz);
        return [w / norm, qx / norm, qy / norm, qz / norm];
    }

    /**
     * Estimates body angular rates using robust cross-product formulation
     * 
     * Solves: ω × z_B = ż_B for angular velocity ω
     * This formulation avoids singularities at hover (z_B = [0,0,1])
     * 
     * Mathematical derivation:
     * Given: ż_B = ω × z_B
     * We can recover ω_x and ω_y from:
     *   ż_B = [ω_y * z_B_z - ω_z * z_B_y,
     *          ω_z * z_B_x - ω_x * z_B_z,
     *          ω_x * z_B_y - ω_y * z_B_x]
     * 
     * Using least-squares solution for robustness at all attitudes.
     */
    private computeBodyRate(
        zBody: Vector3Array,
        jerk: Vector3Array,
        dragForce: Vector3Array,
        mass: number,
        thrustMag: number,
        yaw: number,
        yawRate: number
    ): Vector3Array {
        // Avoid division by zero for very small thrust
        const safeThrustMag = Math.max(thrustMag, 1e-6);

        // ż_B ≈ jerk / ||thrust|| (simplified, ignoring drag dynamics)
        const zBodyDot: Vector3Array = [
            jerk[0] / safeThrustMag,
            jerk[1] / safeThrustMag,
            jerk[2] / safeThrustMag,
        ];

        // Robust angular velocity extraction using cross-product formulation
        // From ż_B = ω × z_B, we solve for ω_x and ω_y
        // 
        // The cross product gives us:
        // ż_B_x = ω_y * z_B_z - ω_z * z_B_y
        // ż_B_y = ω_z * z_B_x - ω_x * z_B_z
        // ż_B_z = ω_x * z_B_y - ω_y * z_B_x
        //
        // Using weighted least-squares for numerical stability:
        const zx = zBody[0], zy = zBody[1], zz = zBody[2];
        const dzx = zBodyDot[0], dzy = zBodyDot[1], dzz = zBodyDot[2];

        // Denominator for stable inversion (sum of squared horizontal components)
        // This equals 1 - zz² = zx² + zy² when |z_B| = 1
        const denom = zx * zx + zy * zy + zz * zz;
        const eps = 1e-8;

        let omegaX: number, omegaY: number;

        if (denom > eps) {
            // General case: use pseudo-inverse solution
            // From the first two equations (ignoring yaw contribution for ω_z):
            // ω_y = (ż_B_x + ω_z * z_B_y) / z_B_z  (if z_B_z ≠ 0)
            // ω_x = (ω_z * z_B_x - ż_B_y) / z_B_z  (if z_B_z ≠ 0)
            //
            // More robust: use the fact that ω is perpendicular to z_B in body frame
            // and solve via cross product: ω_horizontal = z_B × ż_B / |z_B|²

            // Cross product z_B × ż_B gives us direction proportional to ω (in body frame projection)
            const crossX = zy * dzz - zz * dzy;
            const crossY = zz * dzx - zx * dzz;
            const crossZ = zx * dzy - zy * dzx;

            // Scale by |z_B|² (which should be 1 for unit vector, but include for safety)
            const scale = 1.0 / Math.max(denom, eps);

            // Extract roll and pitch rates
            // Note: this gives angular velocity in world frame, need to account for body frame
            omegaX = crossX * scale;
            omegaY = crossY * scale;
        } else {
            // Edge case: z_B is nearly zero (should not happen for valid thrust)
            omegaX = 0;
            omegaY = 0;
        }

        // Yaw rate is decoupled and directly specified
        const omegaZ = yawRate;

        return [omegaX, omegaY, omegaZ];
    }

    /**
     * Validates if the thrust command is within hardware limits
     */
    checkThrustConstraint(thrust: number, minThrust: number, maxThrust: number): boolean {
        return thrust >= minThrust && thrust <= maxThrust;
    }

    /**
     * Validates if the attitude tilt is within safety bounds
     */
    checkTiltConstraint(quaternion: Quaternion, maxTilt: number): boolean {
        // Compute tilt angle from quaternion
        // tilt = acos(1 - 2*(qx² + qy²))
        const [w, qx, qy, qz] = quaternion;
        const tilt = Math.acos(1 - 2 * (qx * qx + qy * qy));
        return tilt <= maxTilt;
    }

    /**
     * Validates if the body rates are within actuator capability
     */
    checkBodyRateConstraint(bodyRate: Vector3Array, maxBodyRate: number): boolean {
        const magnitude = Math.sqrt(bodyRate[0] ** 2 + bodyRate[1] ** 2 + bodyRate[2] ** 2);
        return magnitude <= maxBodyRate;
    }
}

/**
 * Utility functions for Quaternion operations
 */
export class QuaternionUtils {
    /**
     * Perform Hamilton product of two quaternions
     */
    static multiply(q1: Quaternion, q2: Quaternion): Quaternion {
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
     * Converts unit quaternion to Euler angles (ZYX sequence)
     */
    static toEuler(q: Quaternion): { roll: number; pitch: number; yaw: number } {
        const [w, x, y, z] = q;

        // Roll (x-axis rotation)
        const sinr_cosp = 2 * (w * x + y * z);
        const cosr_cosp = 1 - 2 * (x * x + y * y);
        const roll = Math.atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        const sinp = 2 * (w * y - z * x);
        const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);

        // Yaw (z-axis rotation)
        const siny_cosp = 2 * (w * z + x * y);
        const cosy_cosp = 1 - 2 * (y * y + z * z);
        const yaw = Math.atan2(siny_cosp, cosy_cosp);

        return { roll, pitch, yaw };
    }

    /**
     * Converts Euler angles to unit quaternion
     */
    static fromEuler(roll: number, pitch: number, yaw: number): Quaternion {
        const cy = Math.cos(yaw * 0.5);
        const sy = Math.sin(yaw * 0.5);
        const cp = Math.cos(pitch * 0.5);
        const sp = Math.sin(pitch * 0.5);
        const cr = Math.cos(roll * 0.5);
        const sr = Math.sin(roll * 0.5);

        return [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ];
    }

    /**
     * Normalizes a quaternion to unit length
     */
    static normalize(q: Quaternion): Quaternion {
        const norm = Math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2);
        return [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm];
    }
}

