/**
 * @module control/controller
 * @description Cascade PID Flight Controller for Quadrotor
 * 
 * Implements a 4-layer cascade control structure:
 * Position -> Velocity -> Attitude -> Angular Rate
 * 
 * Based on geometric control theory with differential flatness feedforward.
 */

import type { Vector3Array, TrajectoryState } from '../../planning/trajectory/types';
import type {
    Quaternion,
    ControlCommand,
    ControllerParams,
    PIDGains3D
} from './types';
import type { QuadrotorState3D, QuadrotorParams } from '../dynamics/types';
import { FlatnessMap, QuaternionUtils } from './flatness';

/**
 * PID Controller State (for integral and derivative terms)
 */
interface PIDState {
    integral: Vector3Array;
    prevError: Vector3Array;
}

/**
 * Default controller parameters (tuned for ~1kg quadrotor)
 * Higher Z-axis gains for good vertical tracking
 */
export const DEFAULT_CONTROLLER_PARAMS: ControllerParams = {
    positionPID: {
        Kp: [2.0, 2.0, 5.0],  // Increased Z Kp for faster altitude response
        Ki: [0.1, 0.1, 0.5],
        Kd: [1.0, 1.0, 2.0],  // Increased Z Kd for damping
    },
    velocityPID: {
        Kp: [5.0, 5.0, 10.0], // Increased Z Kp
        Ki: [1.0, 1.0, 3.0],  // Increased Z Ki
        Kd: [0.5, 0.5, 1.0],  // Increased Z Kd
    },
    attitudePID: {
        Kp: [6.0, 6.0, 3.0],
    },
    angularRatePID: {
        Kp: [3.0, 3.0, 1.5],
        Ki: [0.5, 0.5, 0.3],
        Kd: [0.1, 0.1, 0.05],
    },
};

/**
 * Controller parameters optimized for aggressive tracking
 * 
 * Key changes from DEFAULT_CONTROLLER_PARAMS:
 * - Reduced integral gains (Ki) to prevent integral saturation during aggressive tracking
 * - Lower integralLimit (2.0 vs 5.0) for tighter anti-windup protection
 * - Slightly higher Kp for faster response without overshoot
 */
export const AGGRESSIVE_TRACKING_PARAMS: ControllerParams = {
    positionPID: {
        Kp: [2.5, 2.5, 6.0],   // Slightly higher for faster response
        Ki: [0.05, 0.05, 0.2], // Reduced from [0.1, 0.1, 0.5] - prevents position integral saturation
        Kd: [1.2, 1.2, 2.5],   // Slightly higher for better damping
    },
    velocityPID: {
        Kp: [6.0, 6.0, 12.0],  // Higher for snappier velocity tracking
        Ki: [0.3, 0.3, 0.8],   // Reduced from [1.0, 1.0, 3.0] - key anti-saturation change
        Kd: [0.6, 0.6, 1.2],   // Slightly higher for damping
    },
    attitudePID: {
        Kp: [8.0, 8.0, 4.0],   // Higher for faster attitude response
    },
    angularRatePID: {
        Kp: [4.0, 4.0, 2.0],   // Higher for faster rate response
        Ki: [0.2, 0.2, 0.1],   // Reduced from [0.5, 0.5, 0.3] - prevents rate integral buildup
        Kd: [0.15, 0.15, 0.08],
    },
};

/** Recommended integral limit for AGGRESSIVE_TRACKING_PARAMS */
export const AGGRESSIVE_TRACKING_INTEGRAL_LIMIT = 2.0;

/**
 * Default quadrotor physical parameters
 */
export const DEFAULT_QUADROTOR_PARAMS: QuadrotorParams = {
    mass: 0.98,
    inertia: [0.0082, 0.0082, 0.0148], // kg·m² for a small quadrotor
    dragCoeff: [0.1, 0.1, 0.15],
    gravity: 9.81,
    thrustMin: 2.0,
    thrustMax: 30.0,  // Increased from 20 to allow stronger altitude corrections
    torqueMax: [0.5, 0.5, 0.2],
};

/**
 * Cascade PID Flight Controller
 * 
 * Control flow:
 * 1. Position error -> Position PID -> Desired velocity
 * 2. Velocity error -> Velocity PID -> Desired acceleration
 * 3. Desired acceleration -> Flatness Map -> Desired attitude (feedforward)
 * 4. Attitude error -> Attitude P -> Desired angular rate
 * 5. Angular rate error -> Rate PID -> Control torque
 */
export class CascadeController {
    private params: ControllerParams;
    private quadParams: QuadrotorParams;
    private flatnessMap: FlatnessMap;

    // PID states for integral windup prevention
    private positionPID: PIDState;
    private velocityPID: PIDState;
    private angularRatePID: PIDState;

    // Configuration
    private integralLimit: number = 5.0;
    private useIntegralWindupProtection: boolean = true;

    constructor(
        params: ControllerParams = DEFAULT_CONTROLLER_PARAMS,
        quadParams: QuadrotorParams = DEFAULT_QUADROTOR_PARAMS
    ) {
        this.params = params;
        this.quadParams = quadParams;
        this.flatnessMap = new FlatnessMap();

        // Initialize PID states
        this.positionPID = { integral: [0, 0, 0], prevError: [0, 0, 0] };
        this.velocityPID = { integral: [0, 0, 0], prevError: [0, 0, 0] };
        this.angularRatePID = { integral: [0, 0, 0], prevError: [0, 0, 0] };
    }

    /**
     * Reset all PID integral and derivative states
     */
    reset(): void {
        this.positionPID = { integral: [0, 0, 0], prevError: [0, 0, 0] };
        this.velocityPID = { integral: [0, 0, 0], prevError: [0, 0, 0] };
        this.angularRatePID = { integral: [0, 0, 0], prevError: [0, 0, 0] };
    }

    /**
     * Main control update function
     * 
     * @param currentState - Current quadrotor state (position, velocity, attitude, angular velocity)
     * @param desiredState - Desired trajectory state from MINCO planner
     * @param yaw - Desired yaw angle (rad)
     * @param yawRate - Desired yaw rate (rad/s)
     * @param dt - Time step (seconds)
     * @returns Control command (thrust + body torque)
     */
    update(
        currentState: QuadrotorState3D,
        desiredState: TrajectoryState,
        yaw: number,
        yawRate: number,
        dt: number
    ): ControlCommand {
        // ========== Layer 1: Position Control ==========
        const posError = this.computeError(
            desiredState.position,
            currentState.position
        );
        const desiredVelocity = this.computePID(
            posError,
            this.params.positionPID,
            this.positionPID,
            dt
        );

        // Add feedforward velocity from trajectory
        const totalDesiredVelocity: Vector3Array = [
            desiredVelocity[0] + desiredState.velocity[0],
            desiredVelocity[1] + desiredState.velocity[1],
            desiredVelocity[2] + desiredState.velocity[2],
        ];

        // ========== Layer 2: Velocity Control ==========
        const velError = this.computeError(
            totalDesiredVelocity,
            currentState.velocity
        );
        const desiredAcceleration = this.computePID(
            velError,
            this.params.velocityPID,
            this.velocityPID,
            dt
        );

        // Add feedforward acceleration and gravity compensation
        const rawDesiredAcceleration: Vector3Array = [
            desiredAcceleration[0] + desiredState.acceleration[0],
            desiredAcceleration[1] + desiredState.acceleration[1],
            desiredAcceleration[2] + desiredState.acceleration[2],
        ];

        // ========== CRITICAL: Clamp total acceleration to prevent thrust overload ==========
        // Without this, large position/velocity errors can cause acceleration demands
        // that exceed physical limits, causing thrust saturation and instability.
        // Max acceleration ~4-6 m/s² is reasonable for a ~1kg quadrotor
        const MAX_HORIZONTAL_ACCEL = 4.0;  // m/s² in X/Y
        const MAX_VERTICAL_ACCEL = 6.0;    // m/s² in Z (more headroom for altitude control)

        const totalDesiredAcceleration: Vector3Array = [
            Math.max(-MAX_HORIZONTAL_ACCEL, Math.min(MAX_HORIZONTAL_ACCEL, rawDesiredAcceleration[0])),
            Math.max(-MAX_HORIZONTAL_ACCEL, Math.min(MAX_HORIZONTAL_ACCEL, rawDesiredAcceleration[1])),
            Math.max(-MAX_VERTICAL_ACCEL, Math.min(MAX_VERTICAL_ACCEL, rawDesiredAcceleration[2])),
        ];

        // ========== Compute Thrust and Desired Attitude via Flatness ==========
        const flatnessInput: TrajectoryState = {
            position: desiredState.position,
            velocity: currentState.velocity,
            acceleration: totalDesiredAcceleration,
            jerk: desiredState.jerk || [0, 0, 0],
        };

        const feedforwardControl = this.flatnessMap.forward(flatnessInput, yaw, yawRate);
        const desiredQuaternion = feedforwardControl.quaternion;
        const thrust = feedforwardControl.thrust;

        // ========== Layer 3: Attitude Control (P only) ==========
        const attError = this.computeQuaternionError(desiredQuaternion, currentState.quaternion);
        const desiredAngularRate: Vector3Array = [
            this.params.attitudePID.Kp[0] * attError[0] + feedforwardControl.bodyRate[0],
            this.params.attitudePID.Kp[1] * attError[1] + feedforwardControl.bodyRate[1],
            this.params.attitudePID.Kp[2] * attError[2] + feedforwardControl.bodyRate[2],
        ];

        // ========== Layer 4: Angular Rate Control ==========
        const rateError = this.computeError(
            desiredAngularRate,
            currentState.angularVelocity
        );
        const torque = this.computePID(
            rateError,
            this.params.angularRatePID,
            this.angularRatePID,
            dt
        );

        // Clamp outputs to physical limits
        const clampedThrust = Math.max(
            this.quadParams.thrustMin,
            Math.min(this.quadParams.thrustMax, thrust)
        );

        const clampedTorque: Vector3Array = [
            Math.max(-this.quadParams.torqueMax[0], Math.min(this.quadParams.torqueMax[0], torque[0])),
            Math.max(-this.quadParams.torqueMax[1], Math.min(this.quadParams.torqueMax[1], torque[1])),
            Math.max(-this.quadParams.torqueMax[2], Math.min(this.quadParams.torqueMax[2], torque[2])),
        ];

        return {
            thrust: clampedThrust,
            torque: clampedTorque,
        };
    }

    /**
     * Compute 3D error vector (desired - current)
     */
    private computeError(desired: Vector3Array, current: Vector3Array): Vector3Array {
        return [
            desired[0] - current[0],
            desired[1] - current[1],
            desired[2] - current[2],
        ];
    }

    /**
     * Generic PID controller for 3 axes
     */
    private computePID(
        error: Vector3Array,
        gains: PIDGains3D,
        state: PIDState,
        dt: number
    ): Vector3Array {
        const output: Vector3Array = [0, 0, 0];

        for (let i = 0; i < 3; i++) {
            // Proportional term
            const pTerm = gains.Kp[i] * error[i];

            // Integral term with anti-windup
            state.integral[i] += error[i] * dt;
            if (this.useIntegralWindupProtection) {
                state.integral[i] = Math.max(
                    -this.integralLimit,
                    Math.min(this.integralLimit, state.integral[i])
                );
            }
            const iTerm = gains.Ki[i] * state.integral[i];

            // Derivative term (with filtering via difference)
            const derivative = (error[i] - state.prevError[i]) / dt;
            const dTerm = gains.Kd[i] * derivative;
            state.prevError[i] = error[i];

            output[i] = pTerm + iTerm + dTerm;
        }

        return output;
    }

    /**
     * Compute attitude error from quaternion difference
     * Returns the rotation error in body frame as [roll_error, pitch_error, yaw_error]
     * 
     * Using q_error = q_desired * q_current^(-1)
     * Then extract the vector part scaled by 2 for small angle approximation
     */
    private computeQuaternionError(
        qDesired: Quaternion,
        qCurrent: Quaternion
    ): Vector3Array {
        // q_current^(-1) = conjugate for unit quaternion
        const qCurrentInv: Quaternion = [
            qCurrent[0],
            -qCurrent[1],
            -qCurrent[2],
            -qCurrent[3],
        ];

        // q_error = q_desired * q_current^(-1)
        const qError = QuaternionUtils.multiply(qDesired, qCurrentInv);

        // Use sign of w to ensure shortest path rotation
        const sign = qError[0] >= 0 ? 1 : -1;

        // Return 2 * vector part (small angle approximation for rotation vector)
        return [
            2 * sign * qError[1],
            2 * sign * qError[2],
            2 * sign * qError[3],
        ];
    }

    /**
     * Set new controller gains
     */
    setParams(params: Partial<ControllerParams>): void {
        this.params = { ...this.params, ...params };
    }

    /**
     * Set new quadrotor physical parameters
     */
    setQuadParams(params: Partial<QuadrotorParams>): void {
        this.quadParams = { ...this.quadParams, ...params };
    }

    /**
     * Get current controller parameters
     */
    getParams(): ControllerParams {
        return { ...this.params };
    }

    /**
     * Set integral windup limit
     * Lower values provide tighter anti-windup protection but may slow response
     * @param limit Maximum absolute value for integral terms (default: 5.0)
     */
    setIntegralLimit(limit: number): void {
        this.integralLimit = limit;
    }

    /**
     * Get current integral limit
     */
    getIntegralLimit(): number {
        return this.integralLimit;
    }
}

/**
 * Simple hover controller for testing
 * Takes current state and desired position, outputs control command
 */
export function simpleHoverController(
    currentState: QuadrotorState3D,
    targetPosition: Vector3Array,
    quadParams: QuadrotorParams = DEFAULT_QUADROTOR_PARAMS,
    Kp: number = 2.0,
    Kd: number = 1.0
): ControlCommand {
    // Position error
    const posError: Vector3Array = [
        targetPosition[0] - currentState.position[0],
        targetPosition[1] - currentState.position[1],
        targetPosition[2] - currentState.position[2],
    ];

    // Velocity (derivative of position error)
    const velError: Vector3Array = [
        -currentState.velocity[0],
        -currentState.velocity[1],
        -currentState.velocity[2],
    ];

    // PD control for desired acceleration
    const desiredAcc: Vector3Array = [
        Kp * posError[0] + Kd * velError[0],
        Kp * posError[1] + Kd * velError[1],
        Kp * posError[2] + Kd * velError[2] + quadParams.gravity,
    ];

    // Thrust = mass * ||acc||
    const thrust = quadParams.mass * Math.sqrt(
        desiredAcc[0] ** 2 + desiredAcc[1] ** 2 + desiredAcc[2] ** 2
    );

    // Simple angular rate damping for stability
    const torque: Vector3Array = [
        -0.5 * currentState.angularVelocity[0],
        -0.5 * currentState.angularVelocity[1],
        -0.2 * currentState.angularVelocity[2],
    ];

    return {
        thrust: Math.max(quadParams.thrustMin, Math.min(quadParams.thrustMax, thrust)),
        torque,
    };
}
