/**
 * @module tasks/u2u-mcs/input
 * @description Input extraction for U2U-MCS task
 */

import type { ScenarioState } from './scenario';
import type { U2UMcsConfig } from './config';

// ==================== Types ====================

/**
 * Link state tracking for recent statistics
 */
export interface LinkState {
    /** Recent ACK history (true = ACK, false = NACK) */
    recentAcks: boolean[];
    /** Recent MCS history */
    recentMcs: number[];
    /** Recent BLER history */
    recentBlers: number[];
    /** Last selected MCS */
    lastMcs: number;
    /** Cumulative throughput */
    cumulativeThroughput: number;
    /** Total packets sent */
    totalPackets: number;
    /** Successful packets */
    successfulPackets: number;
}

/**
 * Full input state for U2U-MCS task
 */
export interface U2UMcsInput {
    /** Current SINR in dB */
    sinrDb: number;
    /** Distance to peer in meters */
    distance: number;
    /** Relative speed in m/s */
    relativeSpeed: number;
    /** Recent ACK rate (0-1) */
    recentAckRate: number;
    /** Recent BLER (0-1) */
    recentBler: number;
    /** Last selected MCS index */
    lastMcs: number;
    /** Normalized step (0-1) */
    normalizedStep: number;
}

// ==================== Input Extraction ====================

/**
 * Create initial link state
 */
export function createInitialLinkState(): LinkState {
    return {
        recentAcks: [],
        recentMcs: [],
        recentBlers: [],
        lastMcs: 0,
        cumulativeThroughput: 0,
        totalPackets: 0,
        successfulPackets: 0,
    };
}

/**
 * Update link state with new transmission result
 */
export function updateLinkState(
    state: LinkState,
    mcs: number,
    ack: boolean,
    throughput: number,
    windowSize: number = 100
): LinkState {
    const recentAcks = [...state.recentAcks, ack].slice(-windowSize);
    const recentMcs = [...state.recentMcs, mcs].slice(-windowSize);

    // Calculate recent BLER
    const recentBler = recentAcks.length > 0
        ? 1 - recentAcks.filter(a => a).length / recentAcks.length
        : 0;

    return {
        recentAcks,
        recentMcs,
        recentBlers: [...state.recentBlers, recentBler].slice(-windowSize),
        lastMcs: mcs,
        cumulativeThroughput: state.cumulativeThroughput + throughput,
        totalPackets: state.totalPackets + 1,
        successfulPackets: state.successfulPackets + (ack ? 1 : 0),
    };
}

/**
 * Extract input from scenario state and link state
 */
export function extractInput(
    scenarioState: ScenarioState,
    linkState: LinkState,
    config: U2UMcsConfig
): U2UMcsInput {
    // Calculate recent ACK rate
    const recentAckRate = linkState.recentAcks.length > 0
        ? linkState.recentAcks.filter(a => a).length / linkState.recentAcks.length
        : 1.0;

    // Calculate recent BLER
    const recentBler = 1 - recentAckRate;

    // Normalized step
    const normalizedStep = scenarioState.step / config.episodeLength;

    return {
        sinrDb: scenarioState.sinrDb,
        distance: scenarioState.distance,
        relativeSpeed: scenarioState.relativeSpeed,
        recentAckRate,
        recentBler,
        lastMcs: linkState.lastMcs,
        normalizedStep,
    };
}

/**
 * Convert input to flat vector
 */
export function inputToVector(input: U2UMcsInput): number[] {
    return [
        input.sinrDb / 30, // Normalize SINR to ~[-1, 1]
        input.distance / 200, // Normalize distance
        input.relativeSpeed / 30, // Normalize speed
        input.recentAckRate,
        input.recentBler,
        input.lastMcs / 22, // Normalize MCS index
        input.normalizedStep,
    ];
}

/**
 * Get input space definition
 */
export function getInputSpace() {
    return {
        kind: 'box' as const,
        shape: [7],
        low: [-1, 0, 0, 0, 0, 0, 0],
        high: [2, 1, 1, 1, 1, 1, 1],
        dtype: 'float32' as const,
    };
}

/**
 * Get output space definition
 */
export function getOutputSpace(enablePowerControl: boolean = false) {
    if (enablePowerControl) {
        // MCS (0-22) + Power offset (-10 to +3 dB in 1dB steps)
        return {
            kind: 'multiDiscrete' as const,
            nvec: [23, 14],
        };
    }
    // Just MCS selection
    return {
        kind: 'discrete' as const,
        n: 23,
    };
}
