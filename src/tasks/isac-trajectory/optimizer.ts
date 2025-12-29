/**
 * @module tasks/isac-trajectory/task1/optimizer
 * @description ISAC Trajectory Optimizer (Paper Alg. 1)
 * 
 * **Key Improvements over original implementation:**
 * 1. **Central Finite Difference** - More stable gradient estimation
 * 2. **Gradient Cache** - Reduces redundant state evaluations
 * 3. **Two-Stage Sampling** - Coarse for optimization, fine for validation
 * 4. **Time-normalized costs** - Stable weights across sampling rates
 * 
 * **Symbion Integration:**
 * - Uses MINCO for analytical jerk gradient
 * - Uses L-BFGS from numeric module
 * - Returns structured result for Core framework
 */

import type { Vector3Array, PiecewiseTrajectory } from '../../models/planning/trajectory/types';
import { MinimumJerkTrajectory, evaluateTrajectory } from '../../models/planning/planning/minco';
import { lbfgsOptimize, LBFGSStatus, lbfgsStatusMessage } from '../../models/numeric/optimization/lbfgs';

import type { Task1Config, SphereObstacle } from './config';
import type { OptimizationState, TrajectorySample, OptimizerReport } from './types';
import { computeCostBreakdown } from './objective';
import { sampleTrajectory } from './utils';

// ==========================================
// Gradient Cache
// ==========================================

interface CacheEntry {
    total: number;
    jerk: number;
}

/**
 * Round vector to cache key
 * Reduces cache misses from floating-point noise
 */
function roundKey(x: number[], rounding: number): string {
    const inv = 1 / rounding;
    return x.map(v => Math.round(v * inv) / inv).join(',');
}

/**
 * Adaptive step size for finite difference
 * Uses larger steps for larger values (relative scaling)
 */
function adaptiveStep(xi: number, cfg: Task1Config): number {
    return Math.max(cfg.optimizer.fdBaseStep, Math.abs(xi) * cfg.optimizer.fdRelativeStep);
}

// ==========================================
// Optimized Trajectory Result Class
// ==========================================

/**
 * Wrapper for optimized trajectory with evaluation utilities
 */
export class OptimizedTrajectory {
    private traj: PiecewiseTrajectory;
    private waypoints: Vector3Array[];
    private totalTime: number;

    constructor(traj: PiecewiseTrajectory, waypoints: Vector3Array[], totalTime: number) {
        this.traj = traj;
        this.waypoints = waypoints;
        this.totalTime = totalTime;
    }

    /** Evaluate trajectory at time t via differential flatness */
    evaluate(t: number) {
        return evaluateTrajectory(this.traj, t);
    }

    getTrajectory(): PiecewiseTrajectory { return this.traj; }
    getWaypoints(): Vector3Array[] { return this.waypoints; }
    getTotalTime(): number { return this.totalTime; }

    /** Sample trajectory at given interval */
    sample(dt: number): TrajectorySample[] {
        return sampleTrajectory(this.traj, dt);
    }
}

// ==========================================
// Optimizer Configuration
// ==========================================

export interface OptimizerOptions {
    startPos: Vector3Array;
    endPos: Vector3Array;
    targetPath: Vector3Array[];
    obstacles: SphereObstacle[];
    totalTime: number;
    numWaypoints?: number;
    config: Task1Config;
}

// ==========================================
// Main Optimizer Class
// ==========================================

/**
 * ISAC Trajectory Optimizer (Paper Alg. 1)
 * 
 * **Core Module Value:**
 * - Integrates MINCO for smooth trajectory representation
 * - Uses L-BFGS from numeric module for efficient optimization
 * - Produces structured results for Core framework
 * 
 * **Key Features:**
 * - Hybrid gradients: Analytical (MINCO) + Numerical (Central FD)
 * - Two-stage sampling: Fast optimization + Strict validation
 * - Gradient caching for performance
 */
export class IsacTrajectoryOptimizer {
    private startPos: Vector3Array;
    private endPos: Vector3Array;
    private targetPath: Vector3Array[];
    private obstacles: SphereObstacle[];
    private totalTime: number;
    private numWaypoints: number;
    private cfg: Task1Config;

    constructor(options: OptimizerOptions) {
        this.startPos = options.startPos;
        this.endPos = options.endPos;
        this.targetPath = options.targetPath;
        this.obstacles = options.obstacles;
        this.totalTime = options.totalTime;
        this.numWaypoints = options.numWaypoints ?? options.config.numWaypoints;
        this.cfg = options.config;
    }

    // ==========================================
    // Waypoint Utilities
    // ==========================================

    /**
     * Build initial waypoints (linear interpolation)
     */
    buildInitialWaypoints(): Vector3Array[] {
        const wps: Vector3Array[] = [this.startPos];
        for (let i = 1; i < this.numWaypoints - 1; i++) {
            const t = i / (this.numWaypoints - 1);
            wps.push([
                this.startPos[0] * (1 - t) + this.endPos[0] * t,
                this.startPos[1] * (1 - t) + this.endPos[1] * t,
                this.startPos[2] * (1 - t) + this.endPos[2] * t
            ]);
        }
        wps.push(this.endPos);
        return wps;
    }

    /**
     * Convert waypoints to optimization vector (intermediate only)
     */
    private waypointsToVector(wps: Vector3Array[]): number[] {
        const x: number[] = [];
        for (let i = 1; i < wps.length - 1; i++) {
            x.push(wps[i][0], wps[i][1], wps[i][2]);
        }
        return x;
    }

    /**
     * Convert optimization vector back to waypoints
     */
    private vectorToWaypoints(x: number[]): Vector3Array[] {
        const wps: Vector3Array[] = [this.startPos];
        for (let i = 0; i < x.length; i += 3) {
            wps.push([x[i], x[i + 1], x[i + 2]]);
        }
        wps.push(this.endPos);
        return wps;
    }

    // ==========================================
    // State Building
    // ==========================================

    /**
     * Build optimization state from waypoints
     * 
     * @param waypoints Waypoint positions
     * @param dt Sampling interval (for two-stage sampling)
     */
    buildState(waypoints: Vector3Array[], dt: number): OptimizationState {
        const numPieces = waypoints.length - 1;
        const durations = new Array(numPieces).fill(this.totalTime / numPieces);

        // Create MINCO trajectory
        const minco = new MinimumJerkTrajectory();
        minco.setConditions(
            { position: this.startPos, velocity: [0, 0, 0] },
            { position: this.endPos, velocity: [0, 0, 0] },
            numPieces
        );
        minco.setParameters(waypoints.slice(1, -1), durations);
        const traj = minco.getTrajectory();
        const samples = sampleTrajectory(traj, dt);

        return {
            traj,
            minco,
            samples,
            waypoints,
            durations,
            targetPath: this.targetPath,
            obstacles: this.obstacles,
            totalTime: this.totalTime,
            dt,
        };
    }

    // ==========================================
    // Main Optimization
    // ==========================================

    /**
     * Run L-BFGS optimization (Paper Alg. 1)
     * 
     * **Process:**
     * 1. Build initial state with validation sampling
     * 2. Run L-BFGS with optimization sampling (coarse, fast)
     * 3. Validate final result with validation sampling (fine, strict)
     * 
     * @returns Optimizer report with cost history and cache stats
     */
    optimize(): {
        trajectory: OptimizedTrajectory;
        initialState: OptimizationState;
        finalState: OptimizationState;
        report: OptimizerReport;
    } {
        const dtOptimize = this.cfg.sampling.dtOptimize;
        const dtValidate = this.cfg.sampling.dtValidate;

        // Initial waypoints
        const initialWaypoints = this.buildInitialWaypoints();
        const x0 = this.waypointsToVector(initialWaypoints);

        // Initial state (with validation sampling for accurate initial metrics)
        const initialState = this.buildState(initialWaypoints, dtValidate);

        // Gradient cache
        const cache = new Map<string, CacheEntry>();
        let cacheHits = 0;
        let cacheMisses = 0;

        // Cost history
        const costHistory: number[] = [];
        let iterCount = 0;

        // ===== Evaluate Penalty (with cache) =====
        const evaluatePenalty = (x: number[]): CacheEntry => {
            const key = this.cfg.optimizer.enableCache
                ? roundKey(x, this.cfg.optimizer.cacheRounding)
                : '';

            if (this.cfg.optimizer.enableCache && cache.has(key)) {
                cacheHits++;
                return cache.get(key)!;
            }

            cacheMisses++;
            const state = this.buildState(this.vectorToWaypoints(x), dtOptimize);
            const cb = computeCostBreakdown(state, this.cfg, dtOptimize);
            const entry = { total: cb.total, jerk: cb.jerk };

            if (this.cfg.optimizer.enableCache) {
                cache.set(key, entry);
            }
            return entry;
        };

        // ===== Objective Function with Gradient =====
        const evaluate = (x: number[]): { cost: number; gradient: number[] } => {
            const waypoints = this.vectorToWaypoints(x);
            const state = this.buildState(waypoints, dtOptimize);

            // Compute cost breakdown
            const costBreak = computeCostBreakdown(state, this.cfg, dtOptimize);
            const cost = costBreak.total;

            // === Gradient Computation ===
            // Hybrid: MINCO analytical + Central FD for penalties
            const gradient = new Array(x.length).fill(0);

            // 1. Get MINCO energy gradient (analytical)
            const gdJerkByCoeffs = state.minco.getEnergyPartialGradByCoeffs();
            const gdJerkByTimes = state.minco.getEnergyPartialGradByTimes();

            // 2. Propagate to waypoint space
            const { gradByPoints } = state.minco.propagateGrad(gdJerkByCoeffs, gdJerkByTimes);

            // 3. Add analytical jerk gradient (intermediate waypoints only)
            for (let i = 0; i < gradByPoints.length && i * 3 < x.length; i++) {
                gradient[i * 3] += this.cfg.weights.smooth * gradByPoints[i][0];
                gradient[i * 3 + 1] += this.cfg.weights.smooth * gradByPoints[i][1];
                gradient[i * 3 + 2] += this.cfg.weights.smooth * gradByPoints[i][2];
            }

            // 4. Central FD for penalty terms (with cache)
            for (let i = 0; i < x.length; i++) {
                const eps = adaptiveStep(x[i], this.cfg);

                const xPlus = x.slice();
                xPlus[i] += eps;
                const xMinus = x.slice();
                xMinus[i] -= eps;

                const plus = evaluatePenalty(xPlus);
                const minus = evaluatePenalty(xMinus);

                // Penalty gradient (exclude jerk which is analytical)
                const penPlus = plus.total - this.cfg.weights.smooth * plus.jerk;
                const penMinus = minus.total - this.cfg.weights.smooth * minus.jerk;

                // Central difference: (f(x+eps) - f(x-eps)) / (2*eps)
                gradient[i] += (penPlus - penMinus) / (2 * eps);
            }

            // Track progress
            if (iterCount % 5 === 0) {
                costHistory.push(cost);
            }
            iterCount++;

            return { cost, gradient };
        };

        // ===== Run L-BFGS =====
        const lbfgsResult = lbfgsOptimize(x0, evaluate, {
            maxIterations: this.cfg.optimizer.maxIterations,
            gEpsilon: this.cfg.optimizer.gEpsilon,
        });

        // ===== Build Final State (with validation sampling) =====
        const finalWaypoints = this.vectorToWaypoints(lbfgsResult.x);
        const finalState = this.buildState(finalWaypoints, dtValidate);

        // Build trajectory wrapper
        const trajectory = new OptimizedTrajectory(
            finalState.traj,
            finalWaypoints,
            this.totalTime
        );

        // Build report
        const report: OptimizerReport = {
            status: lbfgsResult.status === LBFGSStatus.CONVERGENCE ||
                lbfgsResult.status === LBFGSStatus.STOP
                ? 'converged' : 'max_iterations',
            iterations: lbfgsResult.iterations,
            costHistory,
            cacheStats: this.cfg.optimizer.enableCache ? {
                hits: cacheHits,
                misses: cacheMisses,
                hitRate: cacheHits / (cacheHits + cacheMisses) || 0,
            } : undefined,
        };

        return { trajectory, initialState, finalState, report };
    }
}

// ==========================================
// Logging Utilities
// ==========================================

/**
 * Print optimization progress to console
 */
export function logOptimizationStart(cfg: Task1Config, initialCost: number): void {
    console.log('============================================================');
    console.log('     ISAC TRAJECTORY OPTIMIZATION (Paper Alg. 1)');
    console.log('============================================================');
    console.log('');
    console.log(`Task: ${cfg.taskName}`);
    console.log(`Seed: ${cfg.seed}`);
    console.log(`Max Iterations: ${cfg.optimizer.maxIterations}`);
    console.log(`Sampling: optimize=${cfg.sampling.dtOptimize}s, validate=${cfg.sampling.dtValidate}s`);
    console.log(`Gradient: Central FD, cache=${cfg.optimizer.enableCache}`);
    console.log('');
    console.log(`Initial Cost: ${initialCost.toFixed(4)}`);
    console.log('');
}

/**
 * Print optimization result to console
 */
export function logOptimizationEnd(report: OptimizerReport, finalCost: number): void {
    console.log('');
    console.log(`Status: ${report.status.toUpperCase()}`);
    console.log(`Iterations: ${report.iterations}`);
    console.log(`Final Cost: ${finalCost.toFixed(4)}`);

    if (report.cacheStats) {
        console.log(`Cache: ${report.cacheStats.hits} hits, ${report.cacheStats.misses} misses (${(report.cacheStats.hitRate * 100).toFixed(1)}% hit rate)`);
    }
    console.log('');
}
