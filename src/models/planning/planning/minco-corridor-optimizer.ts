/**
 * @module planning/minco-corridor-optimizer
 * @description L-BFGS based trajectory optimizer with corridor constraints.
 * 
 * Combines time allocation optimization with corridor constraint enforcement
 * following the Elastic-Tracker approach:
 * - Optimizes segment durations (time allocation)
 * - Penalizes trajectory points that violate corridor constraints
 * - Uses cubic penalty for smooth gradients
 * 
 * Reference: Zhepei Wang et al., "Geometrically Constrained Trajectory Optimization
 * for Multicopters", IEEE Transactions on Robotics, 2022.
 */

import { lbfgsOptimize, LBFGSStatus, lbfgsStatusMessage } from '../../numeric/optimization/lbfgs';
import { MinimumSnapTrajectory, evaluateTrajectory } from './minco';
import { recordLBFGSIteration, recordMINCOData } from './optimization-history';
import type { Vector3Array, PiecewiseTrajectory } from '../trajectory/types';
import type { HPolyhedron } from './types';

// ==================== Configuration ====================

export interface CorridorOptimizationConfig {
    /** Weight for total time penalty (default: 1.0) */
    timeWeight: number;
    /** Weight for energy (smoothness) cost (default: 1.0) */
    energyWeight: number;
    /** Weight for corridor constraint penalty (default: 100.0) */
    corridorWeight: number;
    /** Minimum duration per segment in seconds (default: 0.1) */
    minDuration: number;
    /** Maximum L-BFGS iterations (default: 50) */
    maxIterations: number;
    /** Convergence tolerance (default: 1e-4) */
    tolerance: number;
    /** Finite difference epsilon (default: 1e-4) */
    fdEpsilon: number;
    /** Number of samples per segment for constraint checking (default: 8) */
    samplesPerSegment: number;
    /** Clearance distance for corridor constraint (default: 0.1m) */
    clearanceD: number;
    /** Enable console logging (default: false) */
    verbose: boolean;
    /** Robot ID for history recording (default: 'robot-0') */
    robotId: string;
}

const defaultConfig: CorridorOptimizationConfig = {
    timeWeight: 1.0,
    energyWeight: 1.0,
    corridorWeight: 100.0,  // Strong penalty for corridor violations
    minDuration: 0.1,
    maxIterations: 50,
    tolerance: 1e-4,
    fdEpsilon: 1e-4,
    samplesPerSegment: 8,
    clearanceD: 0.1,
    verbose: false,
    robotId: 'robot-0',
};

// ==================== Internal Helpers ====================

/**
 * Convert unconstrained tau to positive durations T = e^tau
 */
function tauToT(tau: number[], minDuration: number): number[] {
    return tau.map(t => Math.exp(t) + minDuration);
}

/**
 * Convert positive durations T to unconstrained tau = ln(T - minDuration)
 */
function tToTau(T: number[], minDuration: number): number[] {
    return T.map(t => Math.log(Math.max(t - minDuration, 1e-10)));
}

/**
 * Compute corridor constraint penalty for a single point.
 * 
 * Implements the penetration penalty from Elastic-Tracker:
 * - For each half-plane, compute penetration distance
 * - If penetration > 0, add cubic penalty: rhoP * pen^3
 * 
 * @param p Point to check [x, y, z]
 * @param corridor HPolyhedron representing the safe corridor
 * @param clearanceD Additional clearance distance
 * @returns { violated, cost } where cost is the penalty
 */
function computeCorridorPenalty(
    p: Vector3Array,
    corridor: HPolyhedron,
    clearanceD: number
): { violated: boolean; cost: number } {
    let violated = false;
    let cost = 0;

    for (const plane of corridor) {
        const [a, b, c, d] = plane;

        // Compute penetration: a*x + b*y + c*z + d + clearance
        const normSq = a * a + b * b + c * c;
        const norm = Math.sqrt(normSq);

        if (norm < 1e-10) continue;

        // Normalize the plane equation
        const na = a / norm;
        const nb = b / norm;
        const nc = c / norm;
        const nd = d / norm;

        // Penetration = n · p + nd + clearance (positive means outside corridor)
        const penetration = na * p[0] + nb * p[1] + nc * p[2] + nd + clearanceD;

        if (penetration > 0) {
            violated = true;
            const pen3 = penetration * penetration * penetration;
            cost += pen3;  // Cubic penalty for smooth gradient
        }
    }

    return { violated, cost };
}

/**
 * Compute the MINCO trajectory for given durations and return total cost.
 * Includes both energy cost and corridor constraint penalty.
 */
function computeTotalCost(
    waypoints: Vector3Array[],
    durations: number[],
    headState: { position: Vector3Array; velocity: Vector3Array; acceleration: Vector3Array },
    tailState: { position: Vector3Array; velocity: Vector3Array; acceleration: Vector3Array },
    corridors: HPolyhedron[],
    cfg: CorridorOptimizationConfig
): { totalCost: number; energy: number; corridorCost: number; trajectory: MinimumSnapTrajectory; violatedPoints: number } {
    const numPieces = waypoints.length - 1;
    const traj = new MinimumSnapTrajectory();

    traj.setConditions(headState, tailState, numPieces);
    const intermediateWaypoints = waypoints.slice(1, -1);
    traj.setParameters(intermediateWaypoints, durations);

    const energy = traj.getEnergy();
    const totalT = durations.reduce((a, b) => a + b, 0);

    // Compute corridor constraint cost by sampling trajectory
    let corridorCost = 0;
    let violatedPoints = 0;

    const trajectory = traj.getTrajectory();

    if (corridors && corridors.length > 0 && trajectory.pieces && trajectory.pieces.length > 0) {
        let elapsedTime = 0;

        for (let segIdx = 0; segIdx < trajectory.pieces.length; segIdx++) {
            const segment = trajectory.pieces[segIdx];
            const corridor = corridors[Math.min(segIdx, corridors.length - 1)];

            if (!corridor || corridor.length === 0) {
                elapsedTime += segment.duration;
                continue;
            }

            const dt = segment.duration / cfg.samplesPerSegment;

            for (let j = 0; j <= cfg.samplesPerSegment; j++) {
                const t = elapsedTime + j * dt;
                const state = evaluateTrajectory(trajectory, t);
                const pos = state.position;

                const { violated, cost } = computeCorridorPenalty(pos, corridor, cfg.clearanceD);

                if (violated) {
                    violatedPoints++;
                    corridorCost += cost * dt;  // Weight by time step (trapezoidal integration)
                }
            }

            elapsedTime += segment.duration;
        }
    }

    const totalCost = cfg.energyWeight * energy +
        cfg.timeWeight * totalT +
        cfg.corridorWeight * corridorCost;

    return {
        totalCost,
        energy,
        corridorCost,
        trajectory: traj,
        violatedPoints,
    };
}

// ==================== Main Optimizer ====================

export interface CorridorOptimizationResult {
    /** Optimized segment durations */
    durations: number[];
    /** Optimized trajectory */
    trajectory: PiecewiseTrajectory;
    /** Final total cost */
    cost: number;
    /** Initial cost value */
    initialCost: number;
    /** Number of L-BFGS iterations */
    iterations: number;
    /** Optimization status */
    status: LBFGSStatus;
    /** Status message */
    message: string;
    /** Total optimized time */
    totalTime: number;
    /** Final corridor violation cost */
    corridorCost: number;
    /** Number of violated sample points */
    violatedPoints: number;
}

/**
 * Optimize time allocation with corridor constraints using L-BFGS.
 * 
 * This combines:
 * 1. Time allocation optimization (minimize jerk energy)
 * 2. Corridor constraint enforcement (penalize violations)
 * 
 * @param waypoints Array of 3D waypoints
 * @param initialTotalTime Initial total trajectory time hint
 * @param corridors Safe flight corridors (HPolyhedrons)
 * @param config Optimization configuration
 * @param headState Initial state
 * @param tailState Final state
 * @returns Optimization result with constrained trajectory
 */
export function optimizeWithCorridors(
    waypoints: Vector3Array[],
    initialTotalTime: number,
    corridors: HPolyhedron[],
    config?: Partial<CorridorOptimizationConfig>,
    headState?: { position: Vector3Array; velocity: Vector3Array; acceleration: Vector3Array },
    tailState?: { position: Vector3Array; velocity: Vector3Array; acceleration: Vector3Array }
): CorridorOptimizationResult {
    const cfg = { ...defaultConfig, ...config };
    const numPieces = waypoints.length - 1;

    if (numPieces < 1) {
        throw new Error('At least 2 waypoints required for trajectory optimization');
    }

    // Default boundary conditions
    const head = headState || {
        position: waypoints[0],
        velocity: [0, 0, 0] as Vector3Array,
        acceleration: [0, 0, 0] as Vector3Array,
    };
    const tail = tailState || {
        position: waypoints[waypoints.length - 1],
        velocity: [0, 0, 0] as Vector3Array,
        acceleration: [0, 0, 0] as Vector3Array,
    };

    // Compute initial duration allocation based on distances
    const distances: number[] = [];
    let totalDist = 0;
    for (let i = 0; i < numPieces; i++) {
        const dx = waypoints[i + 1][0] - waypoints[i][0];
        const dy = waypoints[i + 1][1] - waypoints[i][1];
        const dz = waypoints[i + 1][2] - waypoints[i][2];
        const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        distances.push(Math.max(dist, 0.01));
        totalDist += dist;
    }

    // Initial durations proportional to distance
    const initialDurations = distances.map(d =>
        Math.max((d / totalDist) * initialTotalTime, cfg.minDuration * 2)
    );

    // Convert to unconstrained space
    const tau = tToTau(initialDurations, cfg.minDuration);

    // Compute initial cost
    const initialResult = computeTotalCost(waypoints, initialDurations, head, tail, corridors, cfg);

    if (cfg.verbose) {
        console.log(`[MINCO-Corridor] Initial: Energy=${initialResult.energy.toFixed(4)}, ` +
            `CorridorCost=${initialResult.corridorCost.toFixed(4)}, ` +
            `Violated=${initialResult.violatedPoints}, ` +
            `TotalCost=${initialResult.totalCost.toFixed(4)}`);
    }

    // Cost function for L-BFGS
    const costFunction = (x: number[]) => {
        const T = tauToT(x, cfg.minDuration);
        const result = computeTotalCost(waypoints, T, head, tail, corridors, cfg);

        // Compute gradient via finite differences
        const gradient: number[] = [];
        for (let i = 0; i < x.length; i++) {
            const xPlus = [...x];
            const xMinus = [...x];
            xPlus[i] += cfg.fdEpsilon;
            xMinus[i] -= cfg.fdEpsilon;

            const TPlus = tauToT(xPlus, cfg.minDuration);
            const TMinus = tauToT(xMinus, cfg.minDuration);

            const costPlus = computeTotalCost(waypoints, TPlus, head, tail, corridors, cfg).totalCost;
            const costMinus = computeTotalCost(waypoints, TMinus, head, tail, corridors, cfg).totalCost;

            gradient.push((costPlus - costMinus) / (2 * cfg.fdEpsilon));
        }

        return { cost: result.totalCost, gradient };
    };

    // Progress callback
    let iterCount = 0;
    let lastCorridorCost = initialResult.corridorCost;
    let lastViolated = initialResult.violatedPoints;

    const progressCallback = (x: number[], g: number[], fx: number, step: number, k: number) => {
        iterCount = k;

        // Periodically compute corridor cost for logging
        if (cfg.verbose && k % 5 === 0) {
            const T = tauToT(x, cfg.minDuration);
            const result = computeTotalCost(waypoints, T, head, tail, corridors, cfg);
            lastCorridorCost = result.corridorCost;
            lastViolated = result.violatedPoints;
            const totalT = T.reduce((a, b) => a + b, 0);
            console.log(`[MINCO-Corridor] Iter ${k}: Cost=${fx.toFixed(4)}, ` +
                `CorridorViolations=${lastViolated}, Time=${totalT.toFixed(2)}s`);
        }

        recordLBFGSIteration(
            cfg.robotId,
            k,
            fx,
            Math.sqrt(g.reduce((sum, gi) => sum + gi * gi, 0)),
            step
        );

        return 0;
    };

    // Run L-BFGS
    const result = lbfgsOptimize(
        tau,
        costFunction,
        {
            memSize: 6,
            gEpsilon: cfg.tolerance,
            maxIterations: cfg.maxIterations,
        },
        undefined,
        progressCallback
    );

    // Extract optimized durations and final trajectory
    const optimizedDurations = tauToT(result.x, cfg.minDuration);
    const optimizedTotal = optimizedDurations.reduce((a, b) => a + b, 0);
    const finalResult = computeTotalCost(waypoints, optimizedDurations, head, tail, corridors, cfg);

    // Record MINCO data
    recordMINCOData(cfg.robotId, numPieces, optimizedDurations, optimizedTotal, {
        position: finalResult.corridorCost,
        velocity: 0,
        acceleration: 0,
        total: result.cost,
    });

    if (cfg.verbose) {
        console.log(`[MINCO-Corridor] Done: ${lbfgsStatusMessage(result.status)}`);
        console.log(`[MINCO-Corridor] Final: Energy=${finalResult.energy.toFixed(4)}, ` +
            `CorridorCost=${finalResult.corridorCost.toFixed(4)}, ` +
            `Violated=${finalResult.violatedPoints}, Time=${optimizedTotal.toFixed(2)}s`);
        console.log(`[MINCO-Corridor] Improvement: ${((1 - result.cost / initialResult.totalCost) * 100).toFixed(1)}%`);
    }

    return {
        durations: optimizedDurations,
        trajectory: finalResult.trajectory.getTrajectory(),
        cost: result.cost,
        initialCost: initialResult.totalCost,
        iterations: result.iterations,
        status: result.status,
        message: lbfgsStatusMessage(result.status),
        totalTime: optimizedTotal,
        corridorCost: finalResult.corridorCost,
        violatedPoints: finalResult.violatedPoints,
    };
}

/**
 * Generate an optimized minimum-snap trajectory with corridor constraints.
 * This is the main entry point for constrained trajectory generation.
 */
export function generateCorridorConstrainedTrajectory(
    waypoints: Vector3Array[],
    totalTimeHint: number,
    corridors: HPolyhedron[],
    options?: {
        initialVelocity?: Vector3Array;
        finalVelocity?: Vector3Array;
        initialAcceleration?: Vector3Array;
        finalAcceleration?: Vector3Array;
    },
    optimizationConfig?: Partial<CorridorOptimizationConfig>
): { trajectory: PiecewiseTrajectory; optimizationResult: CorridorOptimizationResult } {
    const opts = options || {};

    const headState = {
        position: waypoints[0],
        velocity: opts.initialVelocity || [0, 0, 0] as Vector3Array,
        acceleration: opts.initialAcceleration || [0, 0, 0] as Vector3Array,
    };

    const tailState = {
        position: waypoints[waypoints.length - 1],
        velocity: opts.finalVelocity || [0, 0, 0] as Vector3Array,
        acceleration: opts.finalAcceleration || [0, 0, 0] as Vector3Array,
    };

    const result = optimizeWithCorridors(
        waypoints,
        totalTimeHint,
        corridors,
        optimizationConfig,
        headState,
        tailState
    );

    return {
        trajectory: result.trajectory,
        optimizationResult: result,
    };
}
