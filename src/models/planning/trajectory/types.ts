/**
 * Trajectory-related type definitions
 */

export interface Vector3 {
    x: number;
    y: number;
    z: number;
}

export type Vector3Array = [number, number, number];

/**
 * 3x3 matrix (for rotation, state, etc.)
 */
export type Matrix3x3 = [
    [number, number, number],
    [number, number, number],
    [number, number, number]
];

/**
 * Trajectory Piece - polynomial representation
 */
export interface TrajectoryPiece {
    duration: number; // Duration in seconds
    coefficients: Matrix3x3[]; // Polynomial coefficients [x, y, z] × degree
    degree: number; // Polynomial degree (5 for s=3)
}

/**
 * Complete trajectory
 */
export interface Trajectory {
    pieces: TrajectoryPiece[];
    totalDuration: number;
}

/**
 * Trajectory state (position, velocity, acceleration, etc.)
 */
export interface TrajectoryState {
    position: Vector3Array;
    velocity: Vector3Array;
    acceleration: Vector3Array;
    jerk?: Vector3Array;
}

/**
 * MINCO configuration
 */
export interface MincoConfig {
    order: 2 | 3 | 4; // s=2,3,4 (minimize jerk, snap, crackle)
    pieceNum: number; // Number of trajectory pieces
    headState: Matrix3x3; // Initial state [pos, vel, acc]
    tailState: Matrix3x3; // Final state [pos, vel, acc]
}

/**
 * Single polynomial piece for trajectory representation
 * coeffs[dim][i] represents coefficient c_i for dimension dim
 * The polynomial is: p(t) = c[0] * t^(n-1) + c[1] * t^(n-2) + ... + c[n-1]
 */
export interface PolynomialPiece {
    duration: number;
    coeffs: [number[], number[], number[]]; // [x_coeffs, y_coeffs, z_coeffs]
}

/**
 * Complete polynomial trajectory (sequence of polynomial pieces)
 */
export interface PiecewiseTrajectory {
    pieces: PolynomialPiece[];
}

