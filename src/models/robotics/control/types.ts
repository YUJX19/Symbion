/**
 * @module control/types
 * @description Type definitions for Control algorithms
 */

import type { Vector3Array } from '../../planning/trajectory/types';

/**
 * Quaternion representation [w, x, y, z]
 */
export type Quaternion = [number, number, number, number];

/**
 * Control Output structure (thrust, attitude, body rates)
 */
export interface ControlOutput {
    /** Collective thrust (N) */
    thrust: number;
    /** Desired attitude quaternion */
    quaternion: Quaternion;
    /** Desired body angular velocity (rad/s) */
    bodyRate: Vector3Array;
}

/**
 * Physical parameters for the dynamic model
 */
export interface PhysicsParams {
    /** Mass of the robot (kg) */
    mass: number;
    /** Gravitational acceleration (m/s^2) */
    gravity: number;
    /** Horizontal drag coefficient */
    horizDragCoeff: number;
    /** Vertical drag coefficient */
    vertDragCoeff: number;
    /** Parasitic drag coefficient */
    parasDragCoeff: number;
    /** Speed smoothing factor to avoid singularities */
    speedEps: number;
}

/**
 * Safety and Actuator Constraint Boundaries
 */
export interface ConstraintBounds {
    /** Maximum allowed velocity (m/s) */
    maxVelocity: number;
    /** Maximum allowed body angular rate (rad/s) */
    maxBodyRate: number;
    /** Maximum allowed tilt angle from vertical (rad) */
    maxTiltAngle: number;
    /** Minimum required thrust (N) */
    minThrust: number;
    /** Maximum available thrust (N) */
    maxThrust: number;
}

/**
 * PID Gains for a single axis controller
 */
export interface PIDGains {
    Kp: number;
    Ki: number;
    Kd: number;
}

/**
 * PID Gains for 3-axis controller (X, Y, Z)
 */
export interface PIDGains3D {
    Kp: Vector3Array;
    Ki: Vector3Array;
    Kd: Vector3Array;
}

/**
 * Controller parameters for cascade flight controller
 */
export interface ControllerParams {
    /** Position PID gains */
    positionPID: PIDGains3D;
    /** Velocity PID gains */
    velocityPID: PIDGains3D;
    /** Attitude P gains (roll, pitch, yaw) */
    attitudePID: { Kp: Vector3Array };
    /** Angular rate PID gains (roll rate, pitch rate, yaw rate) */
    angularRatePID: PIDGains3D;
}

/**
 * Control command output (thrust + torque)
 */
export interface ControlCommand {
    /** Total thrust force (N) */
    thrust: number;
    /** Body-frame torque [τx, τy, τz] (N·m) */
    torque: Vector3Array;
}

