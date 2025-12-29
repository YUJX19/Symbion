/**
 * @module tasks/isac-trajectory/task1/objective
 * @description Cost computation and metrics with Core module integration
 * 
 * **Time Normalization:**
 * All sample-based cost terms are multiplied by dt to form proper
 * time integrals, making weights stable across different sampling rates.
 * 
 * **Core Integration:**
 * - Uses `asMetricSpec` from core/objective for type-safe metrics
 * - MetricSpec provides consistent evaluation patterns
 */

import { type MetricSpec, asMetricSpec } from '../../core/objective';

import type { Task1Config } from './config';
import type { OptimizationState, CostBreakdown, Task1Metrics } from './types';
import {
    calculateExpectedPathLoss,
    calculateSINR,
    calculateCapacity,
    calculateLosProbabilityUAV,
    ITU_PARAMS
} from './uav-comm-model';
import { distance3D, getTargetAtTime } from './utils';

// ==========================================
// Cost Computation (Paper Eq. 35-50)
// ==========================================

/**
 * Compute cost breakdown with time normalization
 * 
 * **Key Feature:** All sample-based terms are multiplied by dt
 * to form proper time integrals, making weights stable across
 * different sampling rates.
 * 
 * @param state Current optimization state
 * @param cfg Task configuration
 * @param dt Sampling interval for normalization
 * @returns Detailed cost breakdown
 */
export function computeCostBreakdown(
    state: OptimizationState,
    cfg: Task1Config,
    dt: number
): CostBreakdown {
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
            const safeRadius = obs.radius + cfg.safetyMargin;
            if (d < safeRadius) {
                // Cubic penalty as in paper (Eq. 41)
                jObstacle += Math.pow(safeRadius - d, 3);
            }
        }

        // Velocity constraint (Eq. 38)
        if (sample.speed > cfg.maxVelocity) {
            jDynamics += Math.pow(sample.speed - cfg.maxVelocity, 2);
        }

        // Acceleration constraint (Eq. 38)
        if (sample.accMagnitude > cfg.maxAcceleration) {
            jDynamics += Math.pow(sample.accMagnitude - cfg.maxAcceleration, 2);
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
        if (d < cfg.minCommDistance) {
            jDistance += Math.pow(cfg.minCommDistance - d, 2);
        } else if (d > cfg.maxCommDistance) {
            jDistance += Math.pow(d - cfg.maxCommDistance, 2);
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
            dist, h1, h2, cfg.fcGHz, ITU_PARAMS.suburban
        );

        // SINR and Capacity
        const sinr = calculateSINR(cfg.txPowerDbm, expectedPL, cfg.noisePowerDbm);
        const capacity = calculateCapacity(cfg.bandwidthHz, sinr);

        // Maximize throughput (negative for minimization)
        jThroughput -= capacity / 1e6; // Convert to Mbps

        // URLLC: Penalty if LoS probability below threshold
        if (pLoS < cfg.urllcLosThreshold) {
            jUrllc += Math.pow(cfg.urllcLosThreshold - pLoS, 2);
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
        cfg.weights.smooth * jJerk +
        cfg.weights.obstacle * jObstacle +
        cfg.weights.dynamics * jDynamics +
        cfg.weights.distance * jDistance +
        cfg.weights.throughput * jThroughput +
        cfg.weights.urllc * jUrllc;

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
// Metrics (Core Module Integration)
// ==========================================

/**
 * Create MetricSpecs for Task1
 * 
 * **Core Module Value:**
 * - Type-safe metric definitions with `asMetricSpec`
 * - Consistent direction (minimize/maximize)
 * - Category grouping for analysis
 * 
 * @param cfg Task configuration
 */
export function createMetrics(cfg: Task1Config): MetricSpec<OptimizationState>[] {
    return [
        asMetricSpec<OptimizationState>(
            (state) => state.minco.getEnergy(),
            {
                id: 'jerk_energy',
                description: 'Jerk Energy (∫||r³(t)||² dt)',
                direction: 'minimize',
                category: 'mobility',
            }
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
                        dist, sample.position[2], target[2], cfg.fcGHz
                    );
                    const sinr = calculateSINR(cfg.txPowerDbm, expected, cfg.noisePowerDbm);
                    totalThr += calculateCapacity(cfg.bandwidthHz, sinr);
                }
                return totalThr / state.samples.length / 1e6;
            },
            {
                id: 'throughput_mbps',
                description: 'Avg Throughput (Mbps)',
                direction: 'maximize',
                category: 'communication',
                unit: 'Mbps',
            }
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
            {
                id: 'los_probability_pct',
                description: 'Avg LoS Probability (%)',
                direction: 'maximize',
                category: 'sensing',
                unit: '%',
            }
        ),
        asMetricSpec<OptimizationState>(
            (state) => Math.max(...state.samples.map(s => s.speed)),
            {
                id: 'max_velocity',
                description: 'Max Velocity (m/s)',
                direction: 'minimize',
                category: 'mobility',
                unit: 'm/s',
            }
        ),
        asMetricSpec<OptimizationState>(
            (state) => Math.max(...state.samples.map(s => s.accMagnitude)),
            {
                id: 'max_acceleration',
                description: 'Max Acceleration (m/s²)',
                direction: 'minimize',
                category: 'mobility',
                unit: 'm/s²',
            }
        ),
        asMetricSpec<OptimizationState>(
            (state) => {
                let minClear = Infinity;
                for (const sample of state.samples) {
                    for (const obs of state.obstacles) {
                        const d = distance3D(sample.position, obs.center as [number, number, number]);
                        minClear = Math.min(minClear, d - obs.radius);
                    }
                }
                return minClear;
            },
            {
                id: 'min_obstacle_clearance',
                description: 'Min Obstacle Clearance (m)',
                direction: 'maximize',
                category: 'custom', // Safety category
                unit: 'm',
            }
        ),
    ];
}

/**
 * Extract metrics values from state
 */
export function extractMetrics(state: OptimizationState, cfg: Task1Config): Task1Metrics {
    const metrics = createMetrics(cfg);
    return {
        jerkEnergy: metrics[0].evaluate(state),
        throughputMbps: metrics[1].evaluate(state),
        losProbPct: metrics[2].evaluate(state),
        maxVelocity: metrics[3].evaluate(state),
        maxAcceleration: metrics[4].evaluate(state),
        minObstacleClearance: metrics[5].evaluate(state),
    };
}
