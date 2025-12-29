/**
 * @module tasks/u2u-mcs/reward
 * @description Reward function for U2U-MCS task
 */

import { getMcsEntry, type U2UMcsConfig, type McsEntry } from './config';

// ==================== Types ====================

/**
 * Transmission result for reward calculation
 */
export interface TransmissionResult {
    /** Selected MCS index */
    mcs: number;
    /** Current SINR in dB */
    sinrDb: number;
    /** Whether transmission was acknowledged */
    ack: boolean;
    /** Achieved throughput (bps) */
    throughput: number;
    /** Actual BLER for this transmission */
    bler: number;
    /** Previous MCS (for switch penalty) */
    previousMcs: number;
}

/**
 * Reward breakdown for logging
 */
export interface RewardBreakdown {
    /** Total reward */
    total: number;
    /** Throughput component */
    throughputReward: number;
    /** BLER penalty component */
    blerPenalty: number;
    /** MCS switch penalty component */
    switchPenalty: number;
}

// ==================== BLER Model ====================

/**
 * Calculate BLER based on SINR and MCS using sigmoid approximation
 *
 * This is a simplified model. Real BLER curves are more complex
 * and depend on channel coding, interleaving, etc.
 */
export function calculateBler(sinrDb: number, mcsEntry: McsEntry): number {
    // Sigmoid approximation of BLER curve
    // BLER = 1 / (1 + exp(k * (SINR - threshold)))
    // k controls the steepness of the curve
    const k = 0.8; // Steepness factor
    const threshold = mcsEntry.requiredSinrDb;

    const bler = 1 / (1 + Math.exp(k * (sinrDb - threshold)));
    return Math.min(1, Math.max(0, bler));
}

/**
 * Simulate transmission outcome based on BLER
 */
export function simulateTransmission(
    sinrDb: number,
    mcs: number,
    config: U2UMcsConfig,
    random: () => number
): { ack: boolean; throughput: number; bler: number } {
    const mcsEntry = getMcsEntry(config.mcsTableId, mcs);
    const bler = calculateBler(sinrDb, mcsEntry);

    // Determine if transmission is successful
    const ack = random() > bler;

    // Calculate throughput
    const throughput = ack
        ? mcsEntry.spectralEfficiency * config.bandwidthHz * (config.numRBs / 50) // Normalized
        : 0;

    return { ack, throughput, bler };
}

// ==================== Reward Functions ====================

/**
 * Calculate reward for a transmission
 */
export function calculateReward(
    result: TransmissionResult,
    config: U2UMcsConfig
): RewardBreakdown {
    const { rewardWeights, blerTarget } = config;

    // Throughput reward (normalized)
    const maxThroughput = 5.89 * config.bandwidthHz * (config.numRBs / 50); // Max SE * BW
    const throughputReward = rewardWeights.throughput * (result.throughput / maxThroughput);

    // BLER penalty (penalize if BLER exceeds target)
    let blerPenalty = 0;
    if (result.bler > blerTarget) {
        // Proportional penalty based on how much BLER exceeds target
        const excessBler = result.bler - blerTarget;
        blerPenalty = -rewardWeights.blerPenalty * excessBler;
    }

    // MCS switch penalty
    const switchPenalty = (rewardWeights.switchPenalty ?? 0) > 0 && result.mcs !== result.previousMcs
        ? -(rewardWeights.switchPenalty ?? 0)
        : 0;

    const total = throughputReward + blerPenalty + switchPenalty;

    return {
        total,
        throughputReward,
        blerPenalty,
        switchPenalty,
    };
}

/**
 * Alternative reward: URLLC-focused (prioritize reliability)
 */
export function calculateUrllcReward(
    result: TransmissionResult,
    config: U2UMcsConfig
): RewardBreakdown {
    const { blerTarget } = config;

    // Binary reliability reward
    const isReliable = result.bler <= blerTarget;

    // If reliable, reward throughput. If not, big penalty.
    const throughputReward = isReliable
        ? result.throughput / (5.89 * config.bandwidthHz * (config.numRBs / 50))
        : -1.0;

    const blerPenalty = isReliable ? 0 : -result.bler * 10;

    const total = throughputReward + blerPenalty;

    return {
        total,
        throughputReward,
        blerPenalty,
        switchPenalty: 0,
    };
}

/**
 * Get optimal MCS for given SINR (greedy lookup)
 */
export function getOptimalMcs(
    sinrDb: number,
    config: U2UMcsConfig,
    blerTarget: number = 0.1
): number {
    // Find highest MCS that can achieve target BLER
    let optimalMcs = 0;

    for (let mcs = 22; mcs >= 0; mcs--) {
        const mcsEntry = getMcsEntry(config.mcsTableId, mcs);
        const bler = calculateBler(sinrDb, mcsEntry);

        if (bler <= blerTarget) {
            optimalMcs = mcs;
            break;
        }
    }

    return optimalMcs;
}
