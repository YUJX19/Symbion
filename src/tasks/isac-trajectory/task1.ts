/**
 * @module src/tasks/isac-trajectory/task1
 * @description ISAC Trajectory Baseline - Paper-Accurate Implementation
 * 
 * Implements the paper's optimization formulation (Eq. 26-50):
 * - J_o: Real jerk energy from MINCO (Eq. 26)
 * - UAV-UAV LoS/Path Loss model (Eq. 27-30)
 * - Communication-aware objective (throughput + URLLC)
 * - Hybrid analytical gradients (MINCO + penalty FD)
 * 
 * Key Symbion Integrations:
 * - core/objective: MetricSpec for metric composition
 * - core/constraint: ConstraintSpec for constraint handling
 * - numeric/lbfgs: L-BFGS optimization
 * - planning/minco: MINCO trajectory with getEnergy() and propagateGrad()
 */

import type { Vector3Array, PiecewiseTrajectory } from '../../models/planning/trajectory/types';
import { MinimumJerkTrajectory, evaluateTrajectory } from '../../models/planning/planning/minco';
import { lbfgsOptimize, LBFGSStatus, lbfgsStatusMessage } from '../../models/numeric/optimization/lbfgs';

// Core Module Integration
import { type MetricSpec, asMetricSpec } from '../../core/objective';
import { type ConstraintSpec, type ConstraintReport, leConstraint, geConstraint, evaluateConstraints } from '../../core/constraint';

// UAV-UAV Communication Model (Paper Eq. 27-30)
import {
    calculateLosProbabilityUAV,
    calculateExpectedPathLoss,
    calculateSINR,
    calculateCapacity,
    ITU_PARAMS
} from './uav-comm-model';

// Local Utils
import { sampleTrajectory, distance3D, getTargetAtTime, TrajectorySample } from './utils';

// ==========================================
// 1. System Parameters (Paper Table 2/3)
// ==========================================

const PARAMS = {
    // RF (from paper)
    FC_GHZ: 2.9,
    BANDWIDTH_HZ: 10e6,
    TX_POWER_DBM: 20,
    NOISE_POWER_DBM: -97, // -174 + 10*log10(10e6) + 7

    // Dynamics
    MAX_VELOCITY: 15.0,      // v_m (m/s)
    MAX_ACCELERATION: 10.0,  // a_m (m/s^2)
    SAFETY_MARGIN: 2.0,      // Obstacle safety margin (m)

    // Communication Constraints (Eq. 37-38)
    MIN_COMM_DISTANCE: 10.0, // d_l (m)
    MAX_COMM_DISTANCE: 80.0, // d_u (m)
    LOS_CONE_ANGLE: Math.PI / 6, // θ_k (rad)

    // Optimization Weights
    W_SMOOTH: 1.0,           // Weight for J_o (jerk energy)
    W_OBSTACLE: 10000.0,     // Weight for obstacle penalty (P1)
    W_DYNAMICS: 500.0,       // Weight for velocity/acceleration (P1)
    W_DISTANCE: 200.0,       // Weight for distance window (H1)
    W_THROUGHPUT: 0.1,       // Weight for throughput maximization
    W_URLLC: 5000.0,         // Weight for URLLC reliability penalty
    URLLC_LOS_THRESHOLD: 0.8, // Minimum acceptable LoS probability for URLLC

    // Two-stage Sampling (optimization vs validation)
    DT_OPTIMIZE: 0.4,        // Coarse sampling for optimization (faster)
    DT_VALIDATE: 0.1,        // Fine sampling for validation (stricter)

    // Central Finite Difference Parameters
    FD_BASE_STEP: 1e-2,      // Base step size (1 cm for meter-scale)
    FD_RELATIVE_STEP: 1e-4,  // Relative step for adaptive FD
    ENABLE_CACHE: true,      // Enable gradient cache
    CACHE_ROUNDING: 1e-3,    // Cache key rounding precision (1 mm)
};

// ==========================================
// 2. State Definition
// ==========================================

interface Obstacle {
    center: Vector3Array;
    radius: number;
}

interface OptimizationState {
    traj: PiecewiseTrajectory;
    minco: MinimumJerkTrajectory;
    samples: TrajectorySample[];
    waypoints: Vector3Array[];
    durations: number[];
    targetPath: Vector3Array[];
    obstacles: Obstacle[];
    totalTime: number;
}

// ==========================================
// 3. Cost Decomposition (Paper Eq. 35-50)
// ==========================================

interface CostBreakdown {
    jerk: number;       // J_o = ∫||r^(3)(t)||² dt
    obstacle: number;   // P1 obstacle part
    dynamics: number;   // P1 velocity/acceleration part
    distance: number;   // H1 distance window
    throughput: number; // Negative (maximization)
    urllc: number;      // URLLC reliability penalty
    total: number;
}

/**
 * Compute cost breakdown following paper formulation
 * 
 * **Time Normalization:** All sample-based terms are multiplied by dt
 * to form proper time integrals, making weights stable across sampling rates.
 * 
 * @param state Current optimization state
 * @param dt Sampling interval for time normalization
 */
function computeCostBreakdown(state: OptimizationState, dt: number): CostBreakdown {
    const { minco, samples, targetPath, obstacles, totalTime } = state;

    // ===== J_o: Real Jerk Energy (Eq. 26) =====
    // Uses MINCO's analytical energy computation
    const jJerk = minco.getEnergy();

    // ===== P1: Flight Corridor + Dynamics (Eq. 39-41) =====
    let jObstacle = 0;
    let jDynamics = 0;

    for (const sample of samples) {
        // Obstacle avoidance (spherical repulsion)
        for (const obs of obstacles) {
            const d = distance3D(sample.position, obs.center as [number, number, number]);
            const safeRadius = obs.radius + PARAMS.SAFETY_MARGIN;
            if (d < safeRadius) {
                // Cubic penalty as in paper (Eq. 41)
                jObstacle += Math.pow(safeRadius - d, 3);
            }
        }

        // Velocity constraint (Eq. 38)
        if (sample.speed > PARAMS.MAX_VELOCITY) {
            jDynamics += Math.pow(sample.speed - PARAMS.MAX_VELOCITY, 2);
        }

        // Acceleration constraint (Eq. 38)
        if (sample.accMagnitude > PARAMS.MAX_ACCELERATION) {
            jDynamics += Math.pow(sample.accMagnitude - PARAMS.MAX_ACCELERATION, 2);
        }
    }

    // ===== H1: Distance Window Penalty (Eq. 40) =====
    let jDistance = 0;
    for (const sample of samples) {
        const target = getTargetAtTime(
            targetPath as [number, number, number][],
            sample.t,
            totalTime
        );
        const d = distance3D(sample.position, target);

        // Quadratic penalty outside [d_l, d_u]
        if (d < PARAMS.MIN_COMM_DISTANCE) {
            jDistance += Math.pow(PARAMS.MIN_COMM_DISTANCE - d, 2);
        } else if (d > PARAMS.MAX_COMM_DISTANCE) {
            jDistance += Math.pow(d - PARAMS.MAX_COMM_DISTANCE, 2);
        }
    }

    // ===== Communication Objective (Throughput + URLLC) =====
    let jThroughput = 0;
    let jUrllc = 0;

    for (const sample of samples) {
        const target = getTargetAtTime(
            targetPath as [number, number, number][],
            sample.t,
            totalTime
        );
        const dist = distance3D(sample.position, target);
        const h1 = sample.position[2];
        const h2 = target[2];

        // UAV-UAV Expected Path Loss (Eq. 27-30)
        const { expected: expectedPL, pLoS } = calculateExpectedPathLoss(
            dist, h1, h2, PARAMS.FC_GHZ, ITU_PARAMS.suburban
        );

        // SINR and Capacity
        const sinr = calculateSINR(PARAMS.TX_POWER_DBM, expectedPL, PARAMS.NOISE_POWER_DBM);
        const capacity = calculateCapacity(PARAMS.BANDWIDTH_HZ, sinr);

        // Maximize throughput (negative for minimization)
        jThroughput -= capacity / 1e6; // Convert to Mbps

        // URLLC: Penalty if LoS probability below threshold
        if (pLoS < PARAMS.URLLC_LOS_THRESHOLD) {
            jUrllc += Math.pow(PARAMS.URLLC_LOS_THRESHOLD - pLoS, 2);
        }
    }

    // ===== Time Normalization =====
    // Multiply by dt to form proper time integrals
    jObstacle *= dt;
    jDynamics *= dt;
    jDistance *= dt;
    jThroughput *= dt;
    jUrllc *= dt;

    // ===== Total Cost =====
    const total =
        PARAMS.W_SMOOTH * jJerk +
        PARAMS.W_OBSTACLE * jObstacle +
        PARAMS.W_DYNAMICS * jDynamics +
        PARAMS.W_DISTANCE * jDistance +
        PARAMS.W_THROUGHPUT * jThroughput +
        PARAMS.W_URLLC * jUrllc;

    return {
        jerk: jJerk,
        obstacle: jObstacle,
        dynamics: jDynamics,
        distance: jDistance,
        throughput: jThroughput,
        urllc: jUrllc,
        total
    };
}

// ==========================================
// 4. Metrics (Core Module Integration)
// ==========================================

function createMetrics(): MetricSpec<OptimizationState>[] {
    return [
        asMetricSpec<OptimizationState>(
            (state) => state.minco.getEnergy(),
            { id: 'jerk_energy', description: 'Jerk Energy (∫||r³(t)||² dt)', direction: 'minimize', category: 'mobility' }
        ),
        asMetricSpec<OptimizationState>(
            (state) => {
                let totalThr = 0;
                for (const sample of state.samples) {
                    const target = getTargetAtTime(
                        state.targetPath as [number, number, number][],
                        sample.t,
                        state.totalTime
                    );
                    const dist = distance3D(sample.position, target);
                    const { expected } = calculateExpectedPathLoss(
                        dist, sample.position[2], target[2], PARAMS.FC_GHZ
                    );
                    const sinr = calculateSINR(PARAMS.TX_POWER_DBM, expected, PARAMS.NOISE_POWER_DBM);
                    totalThr += calculateCapacity(PARAMS.BANDWIDTH_HZ, sinr);
                }
                return totalThr / state.samples.length / 1e6;
            },
            { id: 'throughput', description: 'Avg Throughput (Mbps)', direction: 'maximize', category: 'communication', unit: 'Mbps' }
        ),
        asMetricSpec<OptimizationState>(
            (state) => {
                let totalLoS = 0;
                for (const sample of state.samples) {
                    const target = getTargetAtTime(
                        state.targetPath as [number, number, number][],
                        sample.t,
                        state.totalTime
                    );
                    const dist = distance3D(sample.position, target);
                    totalLoS += calculateLosProbabilityUAV(dist, sample.position[2], target[2]);
                }
                return (totalLoS / state.samples.length) * 100;
            },
            { id: 'los_probability', description: 'Avg LoS Probability (%)', direction: 'maximize', category: 'sensing', unit: '%' }
        )
    ];
}

// ==========================================
// 5. Constraints (Core Module Integration)
// ==========================================

function createConstraints(obstacles: Obstacle[]): ConstraintSpec<OptimizationState>[] {
    return [
        leConstraint<OptimizationState>(
            'max_velocity',
            (state) => Math.max(...state.samples.map(s => s.speed)),
            PARAMS.MAX_VELOCITY,
            'soft',
            { description: 'Max Velocity (m/s)', unit: 'm/s', penaltyWeight: PARAMS.W_DYNAMICS }
        ),
        leConstraint<OptimizationState>(
            'max_acceleration',
            (state) => Math.max(...state.samples.map(s => s.accMagnitude)),
            PARAMS.MAX_ACCELERATION,
            'soft',
            { description: 'Max Acceleration (m/s²)', unit: 'm/s²', penaltyWeight: PARAMS.W_DYNAMICS }
        ),
        geConstraint<OptimizationState>(
            'min_obstacle_clearance',
            (state) => {
                let minClear = Infinity;
                for (const sample of state.samples) {
                    for (const obs of obstacles) {
                        const d = distance3D(sample.position, obs.center as [number, number, number]);
                        minClear = Math.min(minClear, d - obs.radius);
                    }
                }
                return minClear;
            },
            2.0,
            'hard',
            { description: 'Min Obstacle Clearance (m)', unit: 'm' }
        ),
        geConstraint<OptimizationState>(
            'min_comm_distance',
            (state) => {
                let minDist = Infinity;
                for (const sample of state.samples) {
                    const target = getTargetAtTime(
                        state.targetPath as [number, number, number][],
                        sample.t,
                        state.totalTime
                    );
                    minDist = Math.min(minDist, distance3D(sample.position, target));
                }
                return minDist;
            },
            PARAMS.MIN_COMM_DISTANCE,
            'soft',
            { description: 'Min Comm Distance (m)', unit: 'm', penaltyWeight: PARAMS.W_DISTANCE }
        ),
        leConstraint<OptimizationState>(
            'max_comm_distance',
            (state) => {
                let maxDist = 0;
                for (const sample of state.samples) {
                    const target = getTargetAtTime(
                        state.targetPath as [number, number, number][],
                        sample.t,
                        state.totalTime
                    );
                    maxDist = Math.max(maxDist, distance3D(sample.position, target));
                }
                return maxDist;
            },
            PARAMS.MAX_COMM_DISTANCE,
            'soft',
            { description: 'Max Comm Distance (m)', unit: 'm', penaltyWeight: PARAMS.W_DISTANCE }
        )
    ];
}

// ==========================================
// 6. Optimized Trajectory Result
// ==========================================

export class OptimizedTrajectory {
    private traj: PiecewiseTrajectory;
    private waypoints: Vector3Array[];
    private totalTime: number;

    constructor(traj: PiecewiseTrajectory, waypoints: Vector3Array[], totalTime: number) {
        this.traj = traj;
        this.waypoints = waypoints;
        this.totalTime = totalTime;
    }

    /** Evaluate via Differential Flatness */
    evaluate(t: number) {
        return evaluateTrajectory(this.traj, t);
    }

    getTrajectory(): PiecewiseTrajectory { return this.traj; }
    getWaypoints(): Vector3Array[] { return this.waypoints; }
    getTotalTime(): number { return this.totalTime; }
    sample(dt: number): TrajectorySample[] { return sampleTrajectory(this.traj, dt); }
}

// ==========================================
// 7. Trajectory Optimizer (Paper Alg. 1)
// ==========================================

export class IsacTrajectoryOptimizer {
    private startPos: Vector3Array;
    private endPos: Vector3Array;
    private targetPath: Vector3Array[];
    private obstacles: Obstacle[];
    private totalTime: number;
    private numWaypoints: number;

    constructor(config: {
        startPos: Vector3Array;
        endPos: Vector3Array;
        targetPath: Vector3Array[];
        obstacles: Obstacle[];
        totalTime: number;
        numWaypoints?: number;
    }) {
        this.startPos = config.startPos;
        this.endPos = config.endPos;
        this.targetPath = config.targetPath;
        this.obstacles = config.obstacles;
        this.totalTime = config.totalTime;
        this.numWaypoints = config.numWaypoints ?? 12;
    }

    private buildInitialWaypoints(): Vector3Array[] {
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

    private waypointsToVector(wps: Vector3Array[]): number[] {
        const x: number[] = [];
        for (let i = 1; i < wps.length - 1; i++) {
            x.push(wps[i][0], wps[i][1], wps[i][2]);
        }
        return x;
    }

    private vectorToWaypoints(x: number[]): Vector3Array[] {
        const wps: Vector3Array[] = [this.startPos];
        for (let i = 0; i < x.length; i += 3) {
            wps.push([x[i], x[i + 1], x[i + 2]]);
        }
        wps.push(this.endPos);
        return wps;
    }

    /**
     * Build optimization state with specified sampling interval
     * 
     * @param waypoints Waypoint positions
     * @param dt Sampling interval (for two-stage sampling)
     */
    private buildState(waypoints: Vector3Array[], dt: number): OptimizationState {
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
            totalTime: this.totalTime
        };
    }

    /**
     * Run L-BFGS optimization (Paper Alg. 1)
     * 
     * **Key Improvements:**
     * - Two-stage sampling: DT_OPTIMIZE (fast) + DT_VALIDATE (strict)
     * - Central Finite Difference for stable gradients
     * - Cache for redundant evaluations
     */
    optimize(maxIterations: number = 100): { result: OptimizedTrajectory; report: OptimizationReport } {
        const initialWaypoints = this.buildInitialWaypoints();
        const x0 = this.waypointsToVector(initialWaypoints);

        const dtOpt = PARAMS.DT_OPTIMIZE;
        const dtVal = PARAMS.DT_VALIDATE;

        // Track optimization progress
        let iterCount = 0;
        const costHistory: number[] = [];

        // Gradient cache (reduces redundant evaluations)
        const cache = new Map<string, { total: number; jerk: number }>();
        let cacheHits = 0;
        let cacheMisses = 0;

        /** Round vector to cache key */
        const roundKey = (x: number[]): string => {
            const inv = 1 / PARAMS.CACHE_ROUNDING;
            return x.map(v => Math.round(v * inv) / inv).join(',');
        };

        /** Adaptive step for Central FD */
        const adaptiveStep = (xi: number): number => {
            return Math.max(PARAMS.FD_BASE_STEP, Math.abs(xi) * PARAMS.FD_RELATIVE_STEP);
        };

        /** Evaluate penalty cost (with cache) */
        const evaluatePenalty = (x: number[]): { total: number; jerk: number } => {
            const key = PARAMS.ENABLE_CACHE ? roundKey(x) : '';
            if (PARAMS.ENABLE_CACHE && cache.has(key)) {
                cacheHits++;
                return cache.get(key)!;
            }
            cacheMisses++;
            const state = this.buildState(this.vectorToWaypoints(x), dtOpt);
            const cb = computeCostBreakdown(state, dtOpt);
            const entry = { total: cb.total, jerk: cb.jerk };
            if (PARAMS.ENABLE_CACHE) {
                cache.set(key, entry);
            }
            return entry;
        };

        // Objective function with gradient (Eq. 50)
        const evaluate = (x: number[]): { cost: number; gradient: number[] } => {
            const waypoints = this.vectorToWaypoints(x);
            const state = this.buildState(waypoints, dtOpt);

            // Compute cost breakdown
            const costBreak = computeCostBreakdown(state, dtOpt);
            const cost = costBreak.total;

            // === Gradient Computation ===
            // Hybrid: MINCO analytical + Central FD for penalties
            const gradient = new Array(x.length).fill(0);

            // 1. Get MINCO energy gradient w.r.t. coefficients
            const gdJerkByCoeffs = state.minco.getEnergyPartialGradByCoeffs();
            const gdJerkByTimes = state.minco.getEnergyPartialGradByTimes();

            // 2. Propagate to waypoint space
            const { gradByPoints } = state.minco.propagateGrad(gdJerkByCoeffs, gdJerkByTimes);

            // 3. Add analytical jerk gradient (intermediate waypoints only)
            for (let i = 0; i < gradByPoints.length && i * 3 < x.length; i++) {
                gradient[i * 3] += PARAMS.W_SMOOTH * gradByPoints[i][0];
                gradient[i * 3 + 1] += PARAMS.W_SMOOTH * gradByPoints[i][1];
                gradient[i * 3 + 2] += PARAMS.W_SMOOTH * gradByPoints[i][2];
            }

            // 4. Central FD for penalty terms (with cache)
            for (let i = 0; i < x.length; i++) {
                const eps = adaptiveStep(x[i]);
                const xPlus = x.slice();
                xPlus[i] += eps;
                const xMinus = x.slice();
                xMinus[i] -= eps;

                const plus = evaluatePenalty(xPlus);
                const minus = evaluatePenalty(xMinus);

                // Penalty gradient (exclude jerk which is analytical)
                const penPlus = plus.total - PARAMS.W_SMOOTH * plus.jerk;
                const penMinus = minus.total - PARAMS.W_SMOOTH * minus.jerk;

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

        // Initial cost (with validation sampling for accurate initial metrics)
        const initialState = this.buildState(initialWaypoints, dtVal);
        const initialCost = computeCostBreakdown(initialState, dtVal);

        console.log('Starting L-BFGS Optimization (Paper Alg. 1)...');
        console.log(`Sampling: optimize=${dtOpt}s, validate=${dtVal}s`);
        console.log(`Gradient: Central FD, cache=${PARAMS.ENABLE_CACHE}`);
        console.log(`Initial Cost: ${initialCost.total.toFixed(2)}`);
        console.log(`  Jerk Energy: ${initialCost.jerk.toFixed(4)}`);
        console.log(`  Obstacle:    ${initialCost.obstacle.toFixed(4)}`);
        console.log(`  Dynamics:    ${initialCost.dynamics.toFixed(4)}`);
        console.log(`  Distance:    ${initialCost.distance.toFixed(4)}`);
        console.log(`  Throughput:  ${initialCost.throughput.toFixed(4)}`);
        console.log(`  URLLC:       ${initialCost.urllc.toFixed(4)}`);

        // Run L-BFGS (removed invalid epsilon parameter)
        const lbfgsResult = lbfgsOptimize(x0, evaluate, {
            maxIterations,
            gEpsilon: 1e-4,
        });

        console.log(`\nL-BFGS: ${lbfgsStatusMessage(lbfgsResult.status)}`);
        console.log(`Final Cost: ${lbfgsResult.cost.toFixed(2)}, Iterations: ${lbfgsResult.iterations}`);
        if (PARAMS.ENABLE_CACHE) {
            console.log(`Cache: ${cacheHits} hits, ${cacheMisses} misses (${(cacheHits / (cacheHits + cacheMisses) * 100).toFixed(1)}% hit rate)`);
        }

        // Build final state and report (with validation sampling)
        const finalWaypoints = this.vectorToWaypoints(lbfgsResult.x);
        const finalState = this.buildState(finalWaypoints, dtVal);
        const finalCost = computeCostBreakdown(finalState, dtVal);

        const metrics = createMetrics();
        const constraints = createConstraints(this.obstacles);
        const constraintReport = evaluateConstraints(constraints, finalState);

        const report: OptimizationReport = {
            status: lbfgsResult.status === LBFGSStatus.CONVERGENCE || lbfgsResult.status === LBFGSStatus.STOP
                ? 'converged' : 'max_iterations',
            iterations: lbfgsResult.iterations,
            costBreakdown: finalCost,
            metrics: {
                jerkEnergy: metrics[0].evaluate(finalState),
                throughput: metrics[1].evaluate(finalState),
                losProb: metrics[2].evaluate(finalState)
            },
            constraints: constraintReport,
            samples: finalState.samples.length
        };

        const result = new OptimizedTrajectory(finalState.traj, finalWaypoints, this.totalTime);
        return { result, report };
    }
}

// ==========================================
// 8. Report Types
// ==========================================

interface OptimizationReport {
    status: 'converged' | 'max_iterations' | 'failed';
    iterations: number;
    costBreakdown: CostBreakdown;
    metrics: {
        jerkEnergy: number;
        throughput: number;
        losProb: number;
    };
    constraints: ConstraintReport;
    samples: number;
}

function printReport(report: OptimizationReport): void {
    console.log('\n' + '='.repeat(60));
    console.log('        ISAC TRAJECTORY OPTIMIZATION REPORT');
    console.log('        (Paper-Accurate Implementation)');
    console.log('='.repeat(60));

    console.log(`\n Status: ${report.status.toUpperCase()}`);
    console.log(`   Iterations: ${report.iterations}`);
    console.log(`   Samples: ${report.samples}`);

    console.log('\n Cost Breakdown (Eq. 35-50):');
    console.log(`   J_o (Jerk):     ${report.costBreakdown.jerk.toFixed(4)}`);
    console.log(`   P1 (Obstacle):  ${report.costBreakdown.obstacle.toFixed(4)}`);
    console.log(`   P1 (Dynamics):  ${report.costBreakdown.dynamics.toFixed(4)}`);
    console.log(`   H1 (Distance):  ${report.costBreakdown.distance.toFixed(4)}`);
    console.log(`   Throughput:     ${report.costBreakdown.throughput.toFixed(4)}`);
    console.log(`   URLLC Penalty:  ${report.costBreakdown.urllc.toFixed(4)}`);
    console.log(`   ---------------------`);
    console.log(`   TOTAL:          ${report.costBreakdown.total.toFixed(4)}`);

    console.log('\n Performance Metrics (UAV-UAV Model):');
    console.log(`   Jerk Energy:         ${report.metrics.jerkEnergy.toFixed(4)}`);
    console.log(`   Avg Throughput:      ${report.metrics.throughput.toFixed(2)} Mbps`);
    console.log(`   Avg LoS Probability: ${report.metrics.losProb.toFixed(1)}%`);

    console.log('\n Constraint Summary:');
    console.log(`   Feasible: ${report.constraints.feasible ? 'YES' : 'NO'}`);
    console.log(`   Total Penalty: ${report.constraints.totalPenalty.toFixed(2)}`);

    for (const r of report.constraints.results) {
        const icon = r.satisfied ? '[OK]' : '[X]';
        const status = r.satisfied ? 'OK' : `VIOLATED (${r.violation.toFixed(2)})`;
        console.log(`   ${icon} ${r.id}: ${r.value.toFixed(2)} ${r.operator} ${r.threshold} -> ${status}`);
    }

    console.log('\n' + '='.repeat(60));
}

// ==========================================
// 9. Main Entry Point
// ==========================================

export async function runTask1(): Promise<void> {
    console.log('============================================================');
    console.log('     ISAC TRAJECTORY BASELINE - PAPER IMPLEMENTATION');
    console.log('     (Eq. 26-50: MINCO + L-BFGS + UAV-UAV Model)');
    console.log('============================================================\n');

    // Scenario Setup
    const totalTime = 30.0;
    const numTargetPoints = 50;

    // Target UAV path
    const targetPath: Vector3Array[] = [];
    for (let i = 0; i < numTargetPoints; i++) {
        const t = i / (numTargetPoints - 1);
        targetPath.push([t * 150, 50 + 10 * Math.sin(t * Math.PI), 100]);
    }

    // Obstacles
    const obstacles: Obstacle[] = [
        { center: [60, 35, 100], radius: 15 },
        { center: [100, 45, 100], radius: 10 }
    ];

    // Follower UAV
    const startPos: Vector3Array = [0, 30, 100];
    const endPos: Vector3Array = [150, 30, 100];

    console.log('Scenario:');
    console.log(`  Start: (${startPos.join(', ')})`);
    console.log(`  End: (${endPos.join(', ')})`);
    console.log(`  Obstacles: ${obstacles.length}`);
    console.log(`  Total Time: ${totalTime}s`);
    console.log('');

    // Run Optimization
    const optimizer = new IsacTrajectoryOptimizer({
        startPos,
        endPos,
        targetPath,
        obstacles,
        totalTime,
        numWaypoints: 12
    });

    const { result, report } = optimizer.optimize(80);

    // Print Report
    printReport(report);

    // Differential Flatness Demo
    console.log('\n Trajectory Evaluation (Differential Flatness):');
    for (const t of [0, 10, 20, 30]) {
        const state = result.evaluate(t);
        console.log(`  t=${t}s: pos=(${state.position.map(v => v.toFixed(1)).join(', ')}), vel=(${state.velocity.map(v => v.toFixed(2)).join(', ')})`);
    }
}

