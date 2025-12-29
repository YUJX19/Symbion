/**
 * @module planning/portal-constraint-optimizer
 * @description MINCO + L-BFGS trajectory optimizer with Portal constraints for gate passing.
 * 
 * Portal Constraint for each gate i:
 * 1. Near-plane: |n_i^T (p(t_i) - c_i)| <= epsilon  (trajectory passes within epsilon of gate plane)
 * 2. In-frame: projected to gate coords, |u| <= w/2 - delta, |v| <= h/2 - delta
 * 
 * This is more suitable for "flying through gates" than Safe Flight Corridors.
 */

import { lbfgsOptimize, LBFGSStatus, lbfgsStatusMessage } from '../../numeric/optimization/lbfgs';
import { MinimumSnapTrajectory, evaluateTrajectory } from './minco';
import { recordLBFGSIteration, recordMINCOData } from './optimization-history';
import type { Vector3Array, PiecewiseTrajectory } from '../trajectory/types';
import { cross3, dot3, normalize3, sub3, type Vec3 } from '../../numeric/math/linear-algebra';

// ==================== Portal Definition ====================

/**
 * Portal (Gate) definition
 */
export interface Portal {
    /** Center of the gate in 3D space */
    center: Vector3Array;
    /** Unit normal vector pointing in the direction the drone should pass */
    normal: Vector3Array;
    /** Unit vector pointing "right" in the gate frame (horizontal) */
    right: Vector3Array;
    /** Unit vector pointing "up" in the gate frame (vertical) */
    up: Vector3Array;
    /** Gate width */
    width: number;
    /** Gate height */
    height: number;
    /** Target passage time (initially estimated, optimized) */
    passageTime: number;
}

/**
 * Configuration for Portal constraint optimization
 */
export interface PortalOptimizationConfig {
    /** Weight for total time penalty (default: 1.0) */
    timeWeight: number;
    /** Weight for energy (smoothness) cost (default: 1.0) */
    energyWeight: number;
    /** Weight for portal plane constraint penalty (default: 100.0) */
    planeWeight: number;
    /** Weight for portal frame constraint penalty (default: 100.0) */
    frameWeight: number;
    /** Maximum distance from gate plane (epsilon) (default: 0.2m) */
    planeEpsilon: number;
    /** Safety margin from gate edges (delta) (default: 0.1m) */
    frameMargin: number;
    /** Minimum duration per segment (default: 0.1s) */
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

const defaultConfig: PortalOptimizationConfig = {
    timeWeight: 1.0,
    energyWeight: 1.0,
    planeWeight: 100.0,
    frameWeight: 100.0,
    planeEpsilon: 0.2,
    frameMargin: 0.1,
    minDuration: 0.1,
    maxIterations: 50,
    tolerance: 1e-4,
    fdEpsilon: 1e-4,
    verbose: false,
    robotId: 'robot-0',
};

// ==================== Helper Functions (using shared math library) ====================

// Convenience wrappers for Vector3Array compatibility
const dot = (a: Vector3Array, b: Vector3Array): number => dot3(a as Vec3, b as Vec3);
const sub = (a: Vector3Array, b: Vector3Array): Vector3Array => sub3(a as Vec3, b as Vec3);
const normalize = (v: Vector3Array): Vector3Array => normalize3(v as Vec3);
const cross = (a: Vector3Array, b: Vector3Array): Vector3Array => cross3(a as Vec3, b as Vec3);


/** Convert unconstrained tau to positive durations T = e^tau + minDuration */
function tauToT(tau: number[], minDuration: number): number[] {
    return tau.map(t => Math.exp(t) + minDuration);
}

/** Convert positive durations T to unconstrained tau = ln(T - minDuration) */
function tToTau(T: number[], minDuration: number): number[] {
    return T.map(t => Math.log(Math.max(t - minDuration, 1e-10)));
}

// ==================== Portal Constraint Computation ====================

/**
 * Compute portal constraint penalty for a single trajectory point.
 * 
 * Portal Constraint:
 * 1. Plane: |n^T (p - c)| <= epsilon
 * 2. Frame: |u| <= w/2 - delta, |v| <= h/2 - delta
 * 
 * Uses cubic penalty for smooth gradients.
 */
function computePortalPenalty(
    position: Vector3Array,
    portal: Portal,
    config: PortalOptimizationConfig
): { planeCost: number; frameCost: number; planeViolated: boolean; frameViolated: boolean } {
    const offset = sub(position, portal.center);

    // 1. Plane constraint: |n^T (p - c)| <= epsilon
    const planeDistance = Math.abs(dot(portal.normal, offset));
    const planeViolation = Math.max(0, planeDistance - config.planeEpsilon);
    const planeCost = planeViolation * planeViolation * planeViolation; // Cubic penalty

    // 2. Frame constraint: project onto gate plane
    const u = dot(portal.right, offset); // Horizontal position in gate frame
    const v = dot(portal.up, offset);    // Vertical position in gate frame

    const halfWidth = portal.width / 2 - config.frameMargin;
    const halfHeight = portal.height / 2 - config.frameMargin;

    const uViolation = Math.max(0, Math.abs(u) - halfWidth);
    const vViolation = Math.max(0, Math.abs(v) - halfHeight);

    const frameCost = (uViolation * uViolation * uViolation) + (vViolation * vViolation * vViolation);

    return {
        planeCost,
        frameCost,
        planeViolated: planeViolation > 0,
        frameViolated: uViolation > 0 || vViolation > 0
    };
}

/**
 * Compute total trajectory cost including portal constraints.
 */
function computeTotalCost(
    waypoints: Vector3Array[],
    durations: number[],
    headState: { position: Vector3Array; velocity: Vector3Array; acceleration: Vector3Array },
    tailState: { position: Vector3Array; velocity: Vector3Array; acceleration: Vector3Array },
    portals: Portal[],
    config: PortalOptimizationConfig
): {
    totalCost: number;
    energy: number;
    timeCost: number;
    planeCost: number;
    frameCost: number;
    planeViolations: number;
    frameViolations: number;
    trajectory: MinimumSnapTrajectory;
} {
    const numPieces = waypoints.length - 1;
    const traj = new MinimumSnapTrajectory();

    traj.setConditions(headState, tailState, numPieces);
    const intermediateWaypoints = waypoints.slice(1, -1);
    traj.setParameters(intermediateWaypoints, durations);

    const energy = traj.getEnergy();
    const totalT = durations.reduce((a, b) => a + b, 0);
    const timeCost = totalT;

    // Compute portal constraint costs
    let planeCostTotal = 0;
    let frameCostTotal = 0;
    let planeViolations = 0;
    let frameViolations = 0;

    const trajectory = traj.getTrajectory();

    // For each portal, evaluate the trajectory at the expected passage time
    let elapsedTime = 0;
    for (let i = 0; i < portals.length && i < numPieces; i++) {
        const portal = portals[i];

        // Sample around the segment midpoint (when drone should pass through gate)
        const segmentDuration = durations[i];
        const passageTime = elapsedTime + segmentDuration / 2;

        // Sample multiple points around passage time for robustness
        const samples = 5;
        for (let s = 0; s < samples; s++) {
            const t = elapsedTime + (s / (samples - 1)) * segmentDuration;
            const state = evaluateTrajectory(trajectory, t);

            // Weight more heavily near passage time
            const weight = 1 + 2 * Math.exp(-((t - passageTime) ** 2) / (0.5 * segmentDuration ** 2));

            const penalty = computePortalPenalty(state.position, portal, config);

            planeCostTotal += penalty.planeCost * weight;
            frameCostTotal += penalty.frameCost * weight;

            if (penalty.planeViolated) planeViolations++;
            if (penalty.frameViolated) frameViolations++;
        }

        elapsedTime += segmentDuration;
    }

    const totalCost =
        config.energyWeight * energy +
        config.timeWeight * timeCost +
        config.planeWeight * planeCostTotal +
        config.frameWeight * frameCostTotal;

    return {
        totalCost,
        energy,
        timeCost,
        planeCost: planeCostTotal,
        frameCost: frameCostTotal,
        planeViolations,
        frameViolations,
        trajectory: traj,
    };
}

// ==================== Result Interface ====================

export interface PortalOptimizationResult {
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
    /** Final energy cost */
    energy: number;
    /** Final plane constraint cost */
    planeCost: number;
    /** Final frame constraint cost */
    frameCost: number;
    /** Number of plane violations */
    planeViolations: number;
    /** Number of frame violations */
    frameViolations: number;
}

// ==================== Main Optimizer ====================

/**
 * Optimize trajectory with Portal constraints using L-BFGS.
 * 
 * @param waypoints Array of 3D waypoints (gate centers)
 * @param portals Array of Portal definitions
 * @param initialTotalTime Initial total trajectory time hint
 * @param config Optimization configuration
 * @param headState Initial state
 * @param tailState Final state
 * @returns Optimization result with trajectory satisfying portal constraints
 */
export function optimizeWithPortals(
    waypoints: Vector3Array[],
    portals: Portal[],
    initialTotalTime: number,
    config?: Partial<PortalOptimizationConfig>,
    headState?: { position: Vector3Array; velocity: Vector3Array; acceleration: Vector3Array },
    tailState?: { position: Vector3Array; velocity: Vector3Array; acceleration: Vector3Array }
): PortalOptimizationResult {
    const cfg = { ...defaultConfig, ...config };
    const numPieces = waypoints.length - 1;

    if (numPieces < 1) {
        throw new Error('At least 2 waypoints required for trajectory optimization');
    }

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

    // Initial durations proportional to distance
    const distances: number[] = [];
    let totalDist = 0;
    for (let i = 0; i < numPieces; i++) {
        const d = sub(waypoints[i + 1], waypoints[i]);
        const dist = Math.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2);
        distances.push(Math.max(dist, 0.01));
        totalDist += dist;
    }

    const initialDurations = distances.map(d =>
        Math.max((d / totalDist) * initialTotalTime, cfg.minDuration * 2)
    );

    const tau = tToTau(initialDurations, cfg.minDuration);

    // Initial cost
    const initialResult = computeTotalCost(waypoints, initialDurations, head, tail, portals, cfg);

    if (cfg.verbose) {
        console.log(`[Portal] Initial: Energy=${initialResult.energy.toFixed(4)}, ` +
            `PlaneCost=${initialResult.planeCost.toFixed(4)}, ` +
            `FrameCost=${initialResult.frameCost.toFixed(4)}, ` +
            `TotalCost=${initialResult.totalCost.toFixed(4)}`);
    }

    // Cost function for L-BFGS
    const costFunction = (x: number[]) => {
        const T = tauToT(x, cfg.minDuration);
        const result = computeTotalCost(waypoints, T, head, tail, portals, cfg);

        // Gradient via finite differences
        const gradient: number[] = [];
        for (let i = 0; i < x.length; i++) {
            const xPlus = [...x];
            const xMinus = [...x];
            xPlus[i] += cfg.fdEpsilon;
            xMinus[i] -= cfg.fdEpsilon;

            const costPlus = computeTotalCost(waypoints, tauToT(xPlus, cfg.minDuration), head, tail, portals, cfg).totalCost;
            const costMinus = computeTotalCost(waypoints, tauToT(xMinus, cfg.minDuration), head, tail, portals, cfg).totalCost;

            gradient.push((costPlus - costMinus) / (2 * cfg.fdEpsilon));
        }

        return { cost: result.totalCost, gradient };
    };

    // Progress callback
    let iterCount = 0;
    const progressCallback = (x: number[], g: number[], fx: number, step: number, k: number) => {
        iterCount = k;

        if (cfg.verbose && k % 5 === 0) {
            const T = tauToT(x, cfg.minDuration);
            const result = computeTotalCost(waypoints, T, head, tail, portals, cfg);
            console.log(`[Portal] Iter ${k}: Cost=${fx.toFixed(4)}, ` +
                `Plane=${result.planeCost.toFixed(4)}, Frame=${result.frameCost.toFixed(4)}`);
        }

        recordLBFGSIteration(cfg.robotId, k, fx, Math.sqrt(g.reduce((s, gi) => s + gi * gi, 0)), step);
        return 0;
    };

    // Run L-BFGS
    const result = lbfgsOptimize(
        tau,
        costFunction,
        { memSize: 6, gEpsilon: cfg.tolerance, maxIterations: cfg.maxIterations },
        undefined,
        progressCallback
    );

    // Extract final result
    const optimizedDurations = tauToT(result.x, cfg.minDuration);
    const optimizedTotal = optimizedDurations.reduce((a, b) => a + b, 0);
    const finalResult = computeTotalCost(waypoints, optimizedDurations, head, tail, portals, cfg);

    // Record MINCO data
    recordMINCOData(cfg.robotId, numPieces, optimizedDurations, optimizedTotal, {
        position: finalResult.planeCost + finalResult.frameCost,
        velocity: 0,
        acceleration: 0,
        total: result.cost,
    });

    if (cfg.verbose) {
        console.log(`[Portal] Done: ${lbfgsStatusMessage(result.status)}`);
        console.log(`[Portal] Final: Energy=${finalResult.energy.toFixed(4)}, ` +
            `PlaneCost=${finalResult.planeCost.toFixed(4)}, FrameCost=${finalResult.frameCost.toFixed(4)}, ` +
            `Time=${optimizedTotal.toFixed(2)}s`);
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
        energy: finalResult.energy,
        planeCost: finalResult.planeCost,
        frameCost: finalResult.frameCost,
        planeViolations: finalResult.planeViolations,
        frameViolations: finalResult.frameViolations,
    };
}

// ==================== Helper: Create Portal from Gate Parameters ====================

/**
 * Create a Portal from gate position and orientation
 */
export function createPortal(
    center: Vector3Array,
    direction: Vector3Array, // Direction the drone is flying
    width: number,
    height: number,
    passageTime: number = 0
): Portal {
    // Normal is perpendicular to the gate plane (same as flight direction)
    const normal = normalize(direction);

    // Compute right and up vectors
    // Default up is [0, 0, 1] in MINCO coords (z is up)
    const worldUp: Vector3Array = [0, 0, 1];

    // Right vector = normal × worldUp
    let right = cross(normal, worldUp);
    const rightLen = Math.sqrt(right[0] ** 2 + right[1] ** 2 + right[2] ** 2);

    if (rightLen < 0.1) {
        // Normal is nearly vertical, use different reference
        const worldForward: Vector3Array = [0, 1, 0];
        right = cross(normal, worldForward);
    }
    right = normalize(right);

    // Up vector = right × normal
    const up = normalize(cross(right, normal));

    return {
        center,
        normal,
        right,
        up,
        width,
        height,
        passageTime,
    };
}

/**
 * Main entry point for Portal-constrained trajectory generation
 */
export function generatePortalConstrainedTrajectory(
    waypoints: Vector3Array[],
    portals: Portal[],
    totalTimeHint: number,
    options?: {
        initialVelocity?: Vector3Array;
        finalVelocity?: Vector3Array;
        initialAcceleration?: Vector3Array;
        finalAcceleration?: Vector3Array;
    },
    optimizationConfig?: Partial<PortalOptimizationConfig>
): { trajectory: PiecewiseTrajectory; optimizationResult: PortalOptimizationResult } {
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

    const result = optimizeWithPortals(
        waypoints,
        portals,
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
