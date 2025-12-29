/**
 * @module planning/types
 * @description Type definitions for Spatial Planning and SFC algorithms
 */

import type { Vector3Array } from '../trajectory/types';

// Re-export common types from drone module (these don't conflict)
export type { Vector2, Pose, Velocity, Robot, BehaviorConfig } from '../../robotics/drone/types';
// Note: Obstacle and WorldConfig should be imported directly from sensing/types

/**
 * 3D Path Node for trajectory planning
 */
export interface PathNode {
    x: number;
    y: number;
    z: number;
    velocity?: Vector3Array;
}

/**
 * H-representation of a convex polyhedron (used for Safe Flight Corridors)
 * Each row [h0, h1, h2, h3] represents the constraint: h0*x + h1*y + h2*z + h3 <= 0
 */
export type HPolyhedron = number[][]; // Nx4 Matrix

/**
 * V-representation of a convex polyhedron (Vertex representation)
 */
export type VPolyhedron = Vector3Array[];

/**
 * Safe Flight Corridor (SFC)
 * A sequential collection of convex polyhedra connecting waypoints
 */
export interface SafeFlightCorridor {
    polyhedra: HPolyhedron[];
    waypoints: Vector3Array[];
}

/**
 * Fast Iterative Region Inflation (FIRI) Configuration
 */
export interface FIRIConfig {
    iterations: number;
    /** Convergence threshold */
    epsilon: number;
}

/**
 * Ellipsoid representation for FIRI inflation
 */
export interface Ellipsoid {
    /** Center point in 3D space */
    center: Vector3Array;
    /** 3x3 rotation matrix */
    rotation: [[number, number, number], [number, number, number], [number, number, number]];
    /** Principal semi-axes lengths */
    radii: Vector3Array;
}

/**
 * Trajectory Configuration
 */
export interface TrajectoryConfig {
    spatialWeight: number;
    smoothWeight: number;
    timeWeight: number;
    integralSteps: number;
}

/**
 * Planning/Optimization Result
 */
export interface PlanningResult<T = unknown> {
    success: boolean;
    trajectory: T;
    cost: number;
    iterations: number;
}


