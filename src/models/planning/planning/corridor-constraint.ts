/**
 * @module planning/corridor-constraint
 * @description Corridor constraint penalty functions for trajectory optimization.
 * 
 * This module implements the corridor constraint penalty as used in Elastic-Tracker:
 * - Check if trajectory points violate corridor boundaries
 * - Compute penalty cost and gradient for L-BFGS optimization
 * 
 * Reference: 
 * - ZJU-FAST-Lab/Elastic-Tracker (grad_cost_p_corridor in traj_opt.cc)
 * - Zhepei Wang et al., "Geometrically Constrained Trajectory Optimization for Multicopters"
 */

import type { Vector3Array, PiecewiseTrajectory } from '../trajectory/types';
import type { HPolyhedron } from './types';
import { evaluateTrajectory } from './minco';

// ==================== Types ====================

export interface CorridorConstraintResult {
    /** Whether any constraint is violated */
    violated: boolean;
    /** Total penalty cost */
    cost: number;
    /** Gradient w.r.t. position [gx, gy, gz] */
    gradient: Vector3Array;
}

export interface TrajectoryCorridorCost {
    /** Total corridor violation cost */
    cost: number;
    /** Sum of gradients at all sample points */
    totalGradientNorm: number;
    /** Number of violated sample points */
    violatedPoints: number;
    /** Total sample points checked */
    totalPoints: number;
}

// ==================== Configuration ====================

export interface CorridorConstraintConfig {
    /** Corridor penalty weight (default: 1000) */
    rhoP: number;
    /** Clearance distance from corridor boundary (default: 0.1m) */
    clearanceD: number;
    /** Number of samples per trajectory segment (default: 10) */
    samplesPerSegment: number;
}

const DEFAULT_CONFIG: CorridorConstraintConfig = {
    rhoP: 1000.0,
    clearanceD: 0.1,
    samplesPerSegment: 10,
};

// ==================== Core Functions ====================

/**
 * Compute corridor constraint penalty and gradient for a single point.
 * 
 * Implements the penetration penalty from Elastic-Tracker:
 * - For each half-plane in HPolyhedron, compute penetration distance
 * - If penetration > 0, add cubic penalty: rhoP * pen^3
 * - Gradient is: rhoP * 3 * pen^2 * normal_vector
 * 
 * @param p Point to check [x, y, z]
 * @param corridor HPolyhedron representing the safe corridor
 * @param config Constraint configuration
 * @returns Whether constraint is violated, penalty cost, and gradient
 * 
 * HPolyhedron format: [[a, b, c, d], ...] where a*x + b*y + c*z + d <= 0
 */
export function gradCostPCorridor(
    p: Vector3Array,
    corridor: HPolyhedron,
    config: Partial<CorridorConstraintConfig> = {}
): CorridorConstraintResult {
    const cfg = { ...DEFAULT_CONFIG, ...config };

    let violated = false;
    let cost = 0;
    const gradient: Vector3Array = [0, 0, 0];

    for (const plane of corridor) {
        const [a, b, c, d] = plane;

        // Compute penetration: a*x + b*y + c*z + d + clearance * ||n||
        // If penetration > 0, the point is outside the corridor
        const normSq = a * a + b * b + c * c;
        const norm = Math.sqrt(normSq);

        if (norm < 1e-10) continue;

        // Normalize the plane equation
        const na = a / norm;
        const nb = b / norm;
        const nc = c / norm;
        const nd = d / norm;

        // Penetration = n · p + nd + clearance
        // (positive means outside corridor)
        const penetration = na * p[0] + nb * p[1] + nc * p[2] + nd + cfg.clearanceD;

        if (penetration > 0) {
            violated = true;
            const pen2 = penetration * penetration;
            const pen3 = pen2 * penetration;

            // Cubic penalty for smooth gradient
            cost += cfg.rhoP * pen3;

            // Gradient: 3 * pen^2 * normal
            gradient[0] += cfg.rhoP * 3 * pen2 * na;
            gradient[1] += cfg.rhoP * 3 * pen2 * nb;
            gradient[2] += cfg.rhoP * 3 * pen2 * nc;
        }
    }

    return { violated, cost, gradient };
}

/**
 * Check if a point is inside a corridor (no penetration).
 * 
 * @param p Point to check [x, y, z]
 * @param corridor HPolyhedron representing the safe corridor
 * @returns true if point is inside the corridor
 */
export function isPointInCorridor(
    p: Vector3Array,
    corridor: HPolyhedron
): boolean {
    for (const plane of corridor) {
        const [a, b, c, d] = plane;
        // a*x + b*y + c*z + d <= 0 for point to be inside
        if (a * p[0] + b * p[1] + c * p[2] + d > 1e-6) {
            return false;
        }
    }
    return true;
}

/**
 * Compute total corridor constraint cost for an entire trajectory.
 * 
 * Samples the trajectory uniformly and accumulates corridor violation penalties.
 * This function can be used as a cost term in L-BFGS optimization.
 * 
 * @param trajectory The piecewise trajectory to check
 * @param corridors Array of HPolyhedrons (one per waypoint segment)
 * @param config Constraint configuration
 * @returns Total cost and statistics
 */
export function computeTrajectoryCorridorCost(
    trajectory: PiecewiseTrajectory,
    corridors: HPolyhedron[],
    config: Partial<CorridorConstraintConfig> = {}
): TrajectoryCorridorCost {
    const cfg = { ...DEFAULT_CONFIG, ...config };

    if (!trajectory.pieces || trajectory.pieces.length === 0) {
        return { cost: 0, totalGradientNorm: 0, violatedPoints: 0, totalPoints: 0 };
    }

    let totalCost = 0;
    let totalGradNorm = 0;
    let violatedCount = 0;
    let sampleCount = 0;

    // Sample each segment
    for (let segIdx = 0; segIdx < trajectory.pieces.length; segIdx++) {
        const segment = trajectory.pieces[segIdx];
        const corridor = corridors[Math.min(segIdx, corridors.length - 1)];

        if (!corridor || corridor.length === 0) continue;

        const dt = segment.duration / cfg.samplesPerSegment;

        for (let i = 0; i <= cfg.samplesPerSegment; i++) {
            const t = i * dt;

            // Evaluate trajectory at time t within this segment
            const state = evaluateTrajectory(trajectory, segIdx * segment.duration + t);
            const pos = state.position;

            const result = gradCostPCorridor(pos, corridor, cfg);

            sampleCount++;
            if (result.violated) {
                violatedCount++;
                totalCost += result.cost;
                totalGradNorm += Math.sqrt(
                    result.gradient[0] ** 2 +
                    result.gradient[1] ** 2 +
                    result.gradient[2] ** 2
                );
            }
        }
    }

    return {
        cost: totalCost,
        totalGradientNorm: totalGradNorm,
        violatedPoints: violatedCount,
        totalPoints: sampleCount,
    };
}

/**
 * Create a corridor constraint function for use with ConstrainedLBFGS.
 * 
 * This returns a constraint function that can be passed to ConstrainedLBFGS.optimizeConstrained().
 * The constraint returns a violation value (> 0 means violated).
 * 
 * @param getTrajectory Function to generate trajectory from optimization variables
 * @param corridors Array of HPolyhedrons for each segment
 * @param config Constraint configuration
 * @returns Constraint function compatible with ConstrainedLBFGS
 */
export function createCorridorConstraint(
    getTrajectory: (x: number[]) => PiecewiseTrajectory,
    corridors: HPolyhedron[],
    config: Partial<CorridorConstraintConfig> = {}
): (x: number[]) => { value: number; gradient: number[] } {
    const cfg = { ...DEFAULT_CONFIG, ...config };

    return (x: number[]) => {
        const trajectory = getTrajectory(x);
        const result = computeTrajectoryCorridorCost(trajectory, corridors, cfg);

        // For ConstrainedLBFGS, constraint value should be <= 0 when satisfied
        // We return the cost as the violation value
        // Gradient is approximated via finite differences (simplified here)
        return {
            value: result.cost,
            gradient: new Array(x.length).fill(0), // Placeholder - real implementation needs FD
        };
    };
}

// ==================== Exports ====================

export {
    gradCostPCorridor as corridorPenalty,
    isPointInCorridor as pointInCorridor,
    computeTrajectoryCorridorCost as trajectoryCost,
};
