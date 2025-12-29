/**
 * @module tasks/u2u-mcs/policies
 * @description Baseline policies for U2U-MCS task
 */

import type { DecisionMaker } from '../../core/runner';
import type { SeededRandom } from '../../core/repro';
import type { U2UMcsInput } from './observation';
import { getMcsEntry, type U2UMcsConfig } from './config';
import { calculateBler } from './reward';

// ==================== Fixed MCS Policy ====================

/**
 * Fixed MCS Policy: Always selects the same MCS
 */
export class FixedMcsPolicy implements DecisionMaker<U2UMcsInput, number> {
    readonly name: string;
    private mcs: number;

    constructor(mcs: number) {
        this.mcs = Math.max(0, Math.min(22, mcs));
        this.name = `fixed-mcs-${this.mcs}`;
    }

    decide(_input: U2UMcsInput, _rng: SeededRandom): number {
        return this.mcs;
    }

    reset(): void {
        // No state to reset
    }
}

// ==================== Random MCS Policy ====================

/**
 * Random MCS Policy: Randomly selects MCS
 */
export class RandomMcsPolicy implements DecisionMaker<U2UMcsInput, number> {
    readonly name = 'random-mcs';
    private maxMcs: number;

    constructor(maxMcs: number = 22) {
        this.maxMcs = maxMcs;
    }

    decide(_input: U2UMcsInput, rng: SeededRandom): number {
        return rng.randint(0, this.maxMcs + 1);
    }

    reset(): void {
        // No state to reset
    }
}

// ==================== Greedy SNR-based Policy ====================

/**
 * Greedy MCS Policy: Selects highest MCS that achieves target BLER
 */
export class GreedyMcsPolicy implements DecisionMaker<U2UMcsInput, number> {
    readonly name: string;
    private config: U2UMcsConfig;
    private blerTarget: number;

    constructor(config: U2UMcsConfig, blerTarget?: number) {
        this.config = config;
        this.blerTarget = blerTarget ?? config.blerTarget;
        this.name = `greedy-mcs-bler${this.blerTarget}`;
    }

    decide(input: U2UMcsInput, _rng: SeededRandom): number {
        // Find highest MCS that can achieve target BLER
        for (let mcs = 22; mcs >= 0; mcs--) {
            const mcsEntry = getMcsEntry(this.config.mcsTableId, mcs);
            const bler = calculateBler(input.sinrDb, mcsEntry);

            if (bler <= this.blerTarget) {
                return mcs;
            }
        }

        return 0; // Fallback to lowest MCS
    }

    reset(): void {
        // No state to reset
    }
}

// ==================== BLER-based Adaptive Policy ====================

/**
 * BLER Adaptive Policy: Adjusts MCS based on recent BLER
 */
export class BlerAdaptivePolicy implements DecisionMaker<U2UMcsInput, number> {
    readonly name = 'bler-adaptive';
    private config: U2UMcsConfig;
    private currentMcs: number = 10; // Start mid-range

    constructor(config: U2UMcsConfig) {
        this.config = config;
    }

    decide(input: U2UMcsInput, _rng: SeededRandom): number {
        const { blerTarget } = this.config;

        // Adjust based on recent BLER
        if (input.recentBler > blerTarget * 2) {
            // BLER too high, decrease MCS by 2
            this.currentMcs = Math.max(0, this.currentMcs - 2);
        } else if (input.recentBler > blerTarget) {
            // BLER slightly high, decrease MCS by 1
            this.currentMcs = Math.max(0, this.currentMcs - 1);
        } else if (input.recentBler < blerTarget * 0.1) {
            // BLER very low, can try higher MCS
            this.currentMcs = Math.min(22, this.currentMcs + 1);
        }

        return this.currentMcs;
    }

    reset(): void {
        this.currentMcs = 10;
    }
}

// ==================== Outer Loop Link Adaptation ====================

/**
 * OLLA (Outer Loop Link Adaptation) Policy
 * Adjusts SINR offset based on ACK/NACK feedback
 */
export class OllaPolicy implements DecisionMaker<U2UMcsInput, number> {
    readonly name = 'olla';
    private config: U2UMcsConfig;
    private sinrOffset: number = 0;
    private readonly stepUp: number;
    private readonly stepDown: number;

    constructor(config: U2UMcsConfig, stepUp: number = 0.1, stepDown?: number) {
        this.config = config;
        this.stepUp = stepUp;
        // stepDown = stepUp * blerTarget / (1 - blerTarget)
        this.stepDown = stepDown ?? (stepUp * config.blerTarget / (1 - config.blerTarget));
    }

    decide(input: U2UMcsInput, _rng: SeededRandom): number {
        // Adjust offset based on recent ACK rate
        const ackRate = input.recentAckRate;

        if (ackRate > 1 - this.config.blerTarget) {
            // Doing well, try higher rates (decrease offset)
            this.sinrOffset -= this.stepUp;
        } else {
            // Not meeting target, be more conservative (increase offset)
            this.sinrOffset += this.stepDown;
        }

        // Clamp offset
        this.sinrOffset = Math.max(-5, Math.min(10, this.sinrOffset));

        // Select MCS based on adjusted SINR
        const effectiveSinr = input.sinrDb - this.sinrOffset;

        for (let mcs = 22; mcs >= 0; mcs--) {
            const mcsEntry = getMcsEntry(this.config.mcsTableId, mcs);
            if (effectiveSinr >= mcsEntry.requiredSinrDb) {
                return mcs;
            }
        }

        return 0;
    }

    reset(): void {
        this.sinrOffset = 0;
    }
}

// ==================== Factory Functions ====================

/**
 * Create a policy by name
 */
export function createPolicy(
    name: string,
    config: U2UMcsConfig
): DecisionMaker<U2UMcsInput, number> {
    if (name.startsWith('fixed-')) {
        const mcs = parseInt(name.split('-')[1], 10);
        return new FixedMcsPolicy(mcs);
    }

    switch (name) {
        case 'random':
            return new RandomMcsPolicy();
        case 'greedy':
            return new GreedyMcsPolicy(config);
        case 'bler-adaptive':
            return new BlerAdaptivePolicy(config);
        case 'olla':
            return new OllaPolicy(config);
        default:
            throw new Error(`Unknown policy: ${name}`);
    }
}

/**
 * Get list of available baseline policies
 */
export function getAvailablePolicies(): string[] {
    return [
        'fixed-0', 'fixed-5', 'fixed-10', 'fixed-15', 'fixed-20',
        'random',
        'greedy',
        'bler-adaptive',
        'olla',
    ];
}
