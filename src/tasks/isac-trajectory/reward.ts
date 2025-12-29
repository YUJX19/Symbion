/**
 * @module tasks/isac-trajectory/reward
 * @description Reward function for ISAC trajectory task
 */

import type { ScenarioState } from './scenario';
import type { IsacTrajectoryConfig, IsacRewardWeights, Position3D } from './config';

// ==================== Types ====================

/**
 * Reward breakdown for logging
 */
export interface RewardBreakdown {
    total: number;
    throughputReward: number;
    losPersistenceReward: number;
    energyPenalty: number;
    urllcPenalty: number;
    sensingPenalty: number;
}

/**
 * Metrics for episode summary
 */
export interface EpisodeMetrics {
    avgThroughput: number;
    avgLosPersistence: number;
    totalEnergy: number;
    urllcViolationRate: number;
    avgSensingCoverage: number;
    pathLength: number;
    missionSuccess: boolean;
}

// ==================== Reward Functions ====================

/**
 * Calculate reward for a step
 */
export function calculateReward(
    state: ScenarioState,
    prevState: ScenarioState | null,
    config: IsacTrajectoryConfig
): RewardBreakdown {
    const weights = config.rewardWeights;

    // Throughput reward (normalized by max possible)
    const maxThroughput = config.users.length * config.comm.bandwidthHz * 10; // ~10 bps/Hz max SE
    const throughputReward = weights.throughput * (state.totalThroughput / maxThroughput);

    // LoS persistence reward
    const losPersistenceReward = weights.losPersistence * state.losPercentage;

    // Energy penalty (normalized)
    const stepEnergy = prevState
        ? state.uav.energyUsed - prevState.uav.energyUsed
        : state.uav.energyUsed;
    const maxStepEnergy = config.kinematics.hoverPower * config.waypointDurationS * 2;
    const energyPenalty = -weights.energy * (stepEnergy / maxStepEnergy);

    // URLLC penalty (per unsatisfied URLLC user)
    const urllcUsers = config.users.filter(u => u.isUrllc).length;
    const urllcViolations = state.urllcSatisfied.filter((s, i) =>
        config.users[i].isUrllc && !s
    ).length;
    const urllcPenalty = urllcUsers > 0
        ? -weights.urllcPenalty * (urllcViolations / urllcUsers)
        : 0;

    // Sensing penalty (for low coverage)
    const sensingTargetCoverage = 0.5; // Target 50% coverage
    const sensingPenalty = state.sensingCoverage < sensingTargetCoverage
        ? -weights.sensingPenalty * (sensingTargetCoverage - state.sensingCoverage)
        : 0;

    const total = throughputReward + losPersistenceReward + energyPenalty + urllcPenalty + sensingPenalty;

    return {
        total,
        throughputReward,
        losPersistenceReward,
        energyPenalty,
        urllcPenalty,
        sensingPenalty,
    };
}

/**
 * Calculate sparse reward (end of episode only)
 */
export function calculateSparseReward(
    episodeStates: ScenarioState[],
    config: IsacTrajectoryConfig
): number {
    if (episodeStates.length === 0) return 0;

    const metrics = calculateEpisodeMetrics(episodeStates, config);

    // Reward based on mission success and efficiency
    let reward = 0;

    if (metrics.missionSuccess) {
        reward += 10; // Bonus for mission success
    }

    reward += metrics.avgLosPersistence * 5;
    reward += (1 - metrics.urllcViolationRate) * 5;
    reward -= metrics.totalEnergy / 10000; // Normalize energy

    return reward;
}

/**
 * Calculate episode metrics
 */
export function calculateEpisodeMetrics(
    states: ScenarioState[],
    config: IsacTrajectoryConfig
): EpisodeMetrics {
    if (states.length === 0) {
        return {
            avgThroughput: 0,
            avgLosPersistence: 0,
            totalEnergy: 0,
            urllcViolationRate: 1,
            avgSensingCoverage: 0,
            pathLength: 0,
            missionSuccess: false,
        };
    }

    const avgThroughput = states.reduce((sum, s) => sum + s.totalThroughput, 0) / states.length;
    const avgLosPersistence = states.reduce((sum, s) => sum + s.losPercentage, 0) / states.length;
    const totalEnergy = states[states.length - 1].uav.energyUsed;

    // URLLC violation rate
    let totalViolations = 0;
    let totalUrllcChecks = 0;
    for (const state of states) {
        for (let i = 0; i < config.users.length; i++) {
            if (config.users[i].isUrllc) {
                totalUrllcChecks++;
                if (!state.urllcSatisfied[i]) {
                    totalViolations++;
                }
            }
        }
    }
    const urllcViolationRate = totalUrllcChecks > 0 ? totalViolations / totalUrllcChecks : 0;

    const avgSensingCoverage = states.reduce((sum, s) => sum + s.sensingCoverage, 0) / states.length;

    // Path length
    let pathLength = 0;
    for (let i = 1; i < states.length; i++) {
        const dx = states[i].uav.position.x - states[i - 1].uav.position.x;
        const dy = states[i].uav.position.y - states[i - 1].uav.position.y;
        const dz = states[i].uav.position.z - states[i - 1].uav.position.z;
        pathLength += Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Mission success criteria
    const missionSuccess = urllcViolationRate < 0.05 && avgLosPersistence > 0.7;

    return {
        avgThroughput,
        avgLosPersistence,
        totalEnergy,
        urllcViolationRate,
        avgSensingCoverage,
        pathLength,
        missionSuccess,
    };
}

/**
 * Calculate distance between two positions
 */
export function distance(a: Position3D, b: Position3D): number {
    return Math.sqrt(
        (a.x - b.x) ** 2 +
        (a.y - b.y) ** 2 +
        (a.z - b.z) ** 2
    );
}
