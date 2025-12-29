/**
 * @module planning/minco-time-optimizer
 * @description L-BFGS based time allocation optimizer for MINCO trajectories.
 * 
 * Optimizes segment durations to minimize control effort (snap energy)
 * while considering time cost. Uses numerical gradients via finite differences.
 * 
 * Reference: Zhepei Wang et al., "Geometrically Constrained Trajectory Optimization
 * for Multicopters", IEEE Transactions on Robotics, 2022.
 */

import { lbfgsOptimize, LBFGSStatus, lbfgsStatusMessage } from '../../numeric/optimization/lbfgs';
import { MinimumSnapTrajectory } from './minco';
import { recordLBFGSIteration, recordMINCOData } from './optimization-history';
import type { Vector3Array, PiecewiseTrajectory } from '../trajectory/types';

// ==================== Configuration ====================

export interface TimeOptimizationConfig {
    /** Weight for total time penalty (default: 1.0) */
    timeWeight: number;
    /** Weight for energy (smoothness) cost (default: 1.0) */
    energyWeight: number;
    /** Minimum duration per segment in seconds (default: 0.1) */
    minDuration: number;
    /** Maximum L-BFGS iterations (default: 50) */
    maxIterations: number;
    /** Convergence tolerance (default: 1e-4) */
    tolerance: number;
    /** Finite difference epsilon (default: 1e-4) */
    fdEpsilon: number;
    /** Enable console logging (default: false) */
    verbose: boolean;
    /** Robot ID for history recording (default: 'robot-0') */
    robotId: string;
}

const defaultConfig: TimeOptimizationConfig = {
    timeWeight: 1.0,
    energyWeight: 1.0,
    minDuration: 0.1,
    maxIterations: 50,
    tolerance: 1e-4,
    fdEpsilon: 1e-4,
    verbose: false,
    robotId: 'robot-0',
};

// ==================== Internal Helpers ====================

/**
 * Convert unconstrained tau to positive durations T = e^tau
 * This ensures all durations are positive.
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
 * Compute the MINCO trajectory for given durations and return energy + time cost.
 */
function computeMincoEnergy(
    waypoints: Vector3Array[],
    durations: number[],
    headState: { position: Vector3Array; velocity: Vector3Array; acceleration: Vector3Array },
    tailState: { position: Vector3Array; velocity: Vector3Array; acceleration: Vector3Array }
): { energy: number; trajectory: MinimumSnapTrajectory } {
    const numPieces = waypoints.length - 1;
    const traj = new MinimumSnapTrajectory();

    traj.setConditions(headState, tailState, numPieces);

    // Intermediate waypoints (excluding start and end)
    const intermediateWaypoints = waypoints.slice(1, -1);
    traj.setParameters(intermediateWaypoints, durations);

    return {
        energy: traj.getEnergy(),
        trajectory: traj,
    };
}

// ==================== Main Optimizer ====================

export interface TimeOptimizationResult {
    /** Optimized segment durations */
    durations: number[];
    /** Optimized trajectory */
    trajectory: PiecewiseTrajectory;
    /** Final cost value */
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
}

/**
 * Optimize time allocation for a minimum-snap trajectory using L-BFGS.
 * 
 * @param waypoints Array of 3D waypoints (at least 2 required)
 * @param initialTotalTime Initial total trajectory time (used for heuristic init)
 * @param config Optional optimization configuration
 * @param headState Initial state (position, velocity, acceleration)
 * @param tailState Final state
 * @returns Optimization result with optimized durations and trajectory
 * 
 * @example
 * ```typescript
 * const result = optimizeTimeAllocation(
 *   [[0, 0, 0], [5, 5, 2], [10, 0, 0]],
 *   5.0, // total time hint
 *   { verbose: true }
 * );
 * console.log(`Optimized in ${result.iterations} iterations`);
 * ```
 */
export function optimizeTimeAllocation(
    waypoints: Vector3Array[],
    initialTotalTime: number,
    config?: Partial<TimeOptimizationConfig>,
    headState?: { position: Vector3Array; velocity: Vector3Array; acceleration: Vector3Array },
    tailState?: { position: Vector3Array; velocity: Vector3Array; acceleration: Vector3Array }
): TimeOptimizationResult {
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
        distances.push(Math.max(dist, 0.01)); // Avoid zero distance
        totalDist += dist;
    }

    // Initial durations proportional to distance
    const initialDurations = distances.map(d =>
        Math.max((d / totalDist) * initialTotalTime, cfg.minDuration * 2)
    );

    // Convert to unconstrained space
    const tau = tToTau(initialDurations, cfg.minDuration);

    // Compute initial cost
    const initialResult = computeMincoEnergy(waypoints, initialDurations, head, tail);
    const initialEnergy = initialResult.energy;
    const initialTotalT = initialDurations.reduce((a, b) => a + b, 0);
    const initialCost = cfg.energyWeight * initialEnergy + cfg.timeWeight * initialTotalT;

    if (cfg.verbose) {
        console.log(`[MINCO-Opt] Initial: Energy=${initialEnergy.toFixed(4)}, Time=${initialTotalT.toFixed(2)}s, Cost=${initialCost.toFixed(4)}`);
    }

    // Cost function for L-BFGS
    const costFunction = (x: number[]) => {
        const T = tauToT(x, cfg.minDuration);
        const { energy } = computeMincoEnergy(waypoints, T, head, tail);
        const totalT = T.reduce((a, b) => a + b, 0);
        const cost = cfg.energyWeight * energy + cfg.timeWeight * totalT;

        // Compute gradient via finite differences
        const gradient: number[] = [];
        for (let i = 0; i < x.length; i++) {
            const xPlus = [...x];
            const xMinus = [...x];
            xPlus[i] += cfg.fdEpsilon;
            xMinus[i] -= cfg.fdEpsilon;

            const TPlus = tauToT(xPlus, cfg.minDuration);
            const TMinus = tauToT(xMinus, cfg.minDuration);

            const { energy: ePlus } = computeMincoEnergy(waypoints, TPlus, head, tail);
            const { energy: eMinus } = computeMincoEnergy(waypoints, TMinus, head, tail);

            const totalTPlus = TPlus.reduce((a, b) => a + b, 0);
            const totalTMinus = TMinus.reduce((a, b) => a + b, 0);

            const costPlus = cfg.energyWeight * ePlus + cfg.timeWeight * totalTPlus;
            const costMinus = cfg.energyWeight * eMinus + cfg.timeWeight * totalTMinus;

            gradient.push((costPlus - costMinus) / (2 * cfg.fdEpsilon));
        }

        return { cost, gradient };
    };

    // Progress callback
    let iterCount = 0;
    const progressCallback = (x: number[], g: number[], fx: number, step: number, k: number) => {
        iterCount = k;

        // Record for visualization
        recordLBFGSIteration(
            cfg.robotId,
            k,
            fx,
            Math.sqrt(g.reduce((sum, gi) => sum + gi * gi, 0)),
            step
        );

        if (cfg.verbose && k % 5 === 0) {
            const T = tauToT(x, cfg.minDuration);
            const totalT = T.reduce((a, b) => a + b, 0);
            console.log(`[MINCO-Opt] Iter ${k}: Cost=${fx.toFixed(4)}, TotalTime=${totalT.toFixed(2)}s`);
        }

        return 0; // Continue optimization
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

    // Extract optimized durations
    const optimizedDurations = tauToT(result.x, cfg.minDuration);
    const optimizedTotal = optimizedDurations.reduce((a, b) => a + b, 0);

    // Generate final trajectory
    const finalResult = computeMincoEnergy(waypoints, optimizedDurations, head, tail);
    const finalEnergy = finalResult.energy;

    // Record final MINCO data
    recordMINCOData(cfg.robotId, numPieces, optimizedDurations, optimizedTotal, {
        position: 0, // Not tracking position cost here
        velocity: 0,
        acceleration: 0,
        total: result.cost,
    });

    if (cfg.verbose) {
        console.log(`[MINCO-Opt] Done: ${lbfgsStatusMessage(result.status)}`);
        console.log(`[MINCO-Opt] Final: Energy=${finalEnergy.toFixed(4)}, Time=${optimizedTotal.toFixed(2)}s, Cost=${result.cost.toFixed(4)}`);
        console.log(`[MINCO-Opt] Improvement: ${((1 - result.cost / initialCost) * 100).toFixed(1)}%`);
    }

    return {
        durations: optimizedDurations,
        trajectory: finalResult.trajectory.getTrajectory(),
        cost: result.cost,
        initialCost,
        iterations: result.iterations,
        status: result.status,
        message: lbfgsStatusMessage(result.status),
        totalTime: optimizedTotal,
    };
}

/**
 * Generate an optimized minimum-snap trajectory through waypoints.
 * This is the main entry point that combines time optimization with trajectory generation.
 * 
 * @param waypoints Array of 3D waypoints
 * @param totalTimeHint Approximate total trajectory time
 * @param options Trajectory options (velocities, accelerations at boundaries)
 * @param optimizationConfig L-BFGS optimization settings
 * @returns Optimized piecewise trajectory
 */
export function generateOptimizedTrajectory(
    waypoints: Vector3Array[],
    totalTimeHint: number,
    options?: {
        initialVelocity?: Vector3Array;
        finalVelocity?: Vector3Array;
        initialAcceleration?: Vector3Array;
        finalAcceleration?: Vector3Array;
    },
    optimizationConfig?: Partial<TimeOptimizationConfig>
): { trajectory: PiecewiseTrajectory; optimizationResult: TimeOptimizationResult } {
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

    const result = optimizeTimeAllocation(
        waypoints,
        totalTimeHint,
        optimizationConfig,
        headState,
        tailState
    );

    return {
        trajectory: result.trajectory,
        optimizationResult: result,
    };
}
