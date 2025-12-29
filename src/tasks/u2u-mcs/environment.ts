/**
 * @module tasks/u2u-mcs/environment
 * @description U2U-MCS Sequence Prediction Environment
 * 
 * Simulation environment for dynamic MCS selection using sequence prediction.
 * Suitable for LSTM-based online learning and heuristic baseline methods.
 */

import type { Environment } from '../../core/runner';
import type { Space } from '../../core/space';
import { createRng, SeededRandom } from '../../core/repro';
import { DEFAULT_U2U_MCS_CONFIG, type U2UMcsConfig } from './config';
import { U2UScenario, ScenarioState } from './scenario';
import {
    LinkState,
    createInitialLinkState,
    updateLinkState,
    extractInput,
    inputToVector,
    getInputSpace,
    getOutputSpace,
} from './observation';
import { simulateTransmission, calculateReward, RewardBreakdown } from './reward';

// ==================== Types ====================

/**
 * Extended info returned by step()
 */
export interface U2UMcsStepInfo {
    /** Current step number */
    step: number;
    /** Selected MCS */
    mcs: number;
    /** Current SINR (dB) */
    sinrDb: number;
    /** Distance (m) */
    distance: number;
    /** Relative speed (m/s) */
    relativeSpeed: number;
    /** Transmission acknowledged */
    ack: boolean;
    /** Achieved throughput (bps) */
    throughput: number;
    /** BLER for this transmission */
    bler: number;
    /** Reward breakdown */
    rewardBreakdown: RewardBreakdown;
    /** Cumulative throughput */
    cumulativeThroughput: number;
    /** Average BLER so far */
    avgBler: number;
    /** URLLC success rate (BLER < target) */
    urllcSuccessRate: number;
}

/**
 * Episode summary for logging
 */
export interface U2UMcsEpisodeSummary {
    /** Total steps */
    totalSteps: number;
    /** Average throughput (bps) */
    avgThroughput: number;
    /** 5th percentile throughput */
    p5Throughput: number;
    /** Average BLER */
    avgBler: number;
    /** URLLC success rate */
    urllcSuccessRate: number;
    /** MCS histogram (count per MCS) */
    mcsHistogram: number[];
    /** MCS switch rate */
    mcsSwitchRate: number;
    /** Total reward */
    totalReward: number;
}

/**
 * U2U-MCS Environment
 */
export class U2UMcsEnvironment implements Environment<number[], number, U2UMcsStepInfo> {
    readonly inputSpace: Space;
    readonly outputSpace: Space;

    private config: U2UMcsConfig;
    private scenario: U2UScenario;
    private linkState: LinkState;
    private rng: SeededRandom;
    private currentStep: number = 0;
    private scenarioState: ScenarioState | null = null;

    // For episode summary
    private throughputHistory: number[] = [];
    private blerHistory: number[] = [];
    private mcsHistory: number[] = [];
    private totalMetric: number = 0;

    constructor(config: Partial<U2UMcsConfig> = {}) {
        this.config = { ...DEFAULT_U2U_MCS_CONFIG, ...config };
        this.inputSpace = getInputSpace();
        this.outputSpace = getOutputSpace(this.config.enablePowerControl);

        this.scenario = new U2UScenario(this.config);
        this.linkState = createInitialLinkState();
        this.rng = createRng(this.config.seed);
    }

    async reset(seed?: number): Promise<{ input: number[]; info: U2UMcsStepInfo }> {
        const actualSeed = seed ?? this.config.seed;
        this.rng = createRng(actualSeed);
        this.scenario.reset(actualSeed);
        this.linkState = createInitialLinkState();
        this.currentStep = 0;

        // Reset history
        this.throughputHistory = [];
        this.blerHistory = [];
        this.mcsHistory = [];
        this.totalMetric = 0;

        // Generate initial state
        this.scenarioState = this.scenario.generateState();

        const inputState = extractInput(this.scenarioState, this.linkState, this.config);
        const input = inputToVector(inputState);

        const info: U2UMcsStepInfo = {
            step: 0,
            mcs: 0,
            sinrDb: this.scenarioState.sinrDb,
            distance: this.scenarioState.distance,
            relativeSpeed: this.scenarioState.relativeSpeed,
            ack: true,
            throughput: 0,
            bler: 0,
            rewardBreakdown: { total: 0, throughputReward: 0, blerPenalty: 0, switchPenalty: 0 },
            cumulativeThroughput: 0,
            avgBler: 0,
            urllcSuccessRate: 1,
        };

        return { input, info };
    }

    async step(output: number): Promise<{
        input: number[];
        metric: number;
        done: boolean;
        truncated: boolean;
        info: U2UMcsStepInfo;
    }> {
        if (!this.scenarioState) {
            throw new Error('Environment not reset. Call reset() first.');
        }

        const mcs = Math.max(0, Math.min(22, Math.floor(output)));
        const previousMcs = this.linkState.lastMcs;

        // Simulate transmission
        const { ack, throughput, bler } = simulateTransmission(
            this.scenarioState.sinrDb,
            mcs,
            this.config,
            () => this.rng.random()
        );

        // Calculate reward/metric
        const rewardBreakdown = calculateReward(
            {
                mcs,
                sinrDb: this.scenarioState.sinrDb,
                ack,
                throughput,
                bler,
                previousMcs,
            },
            this.config
        );

        // Update link state
        this.linkState = updateLinkState(this.linkState, mcs, ack, throughput);

        // Update history
        this.throughputHistory.push(throughput);
        this.blerHistory.push(bler);
        this.mcsHistory.push(mcs);
        this.totalMetric += rewardBreakdown.total;

        // Move to next step
        this.currentStep++;
        this.scenarioState = this.scenario.generateState();

        // Extract new input
        const inputState = extractInput(this.scenarioState, this.linkState, this.config);
        const input = inputToVector(inputState);

        // Check termination
        const done = false; // Episode continues until truncated
        const truncated = this.currentStep >= this.config.episodeLength;

        // Calculate metrics
        const avgBler = this.blerHistory.reduce((a, b) => a + b, 0) / this.blerHistory.length;
        const urllcSuccess = this.blerHistory.filter(b => b <= this.config.blerTarget).length;
        const urllcSuccessRate = urllcSuccess / this.blerHistory.length;

        const info: U2UMcsStepInfo = {
            step: this.currentStep,
            mcs,
            sinrDb: this.scenarioState.sinrDb,
            distance: this.scenarioState.distance,
            relativeSpeed: this.scenarioState.relativeSpeed,
            ack,
            throughput,
            bler,
            rewardBreakdown,
            cumulativeThroughput: this.linkState.cumulativeThroughput,
            avgBler,
            urllcSuccessRate,
        };

        return {
            input,
            metric: rewardBreakdown.total,
            done,
            truncated,
            info,
        };
    }

    /**
     * Get episode summary for logging
     */
    getEpisodeSummary(): U2UMcsEpisodeSummary {
        const sortedThroughput = [...this.throughputHistory].sort((a, b) => a - b);
        const p5Index = Math.floor(sortedThroughput.length * 0.05);

        // MCS histogram
        const mcsHistogram = new Array(23).fill(0);
        for (const mcs of this.mcsHistory) {
            mcsHistogram[mcs]++;
        }

        // MCS switch count
        let switches = 0;
        for (let i = 1; i < this.mcsHistory.length; i++) {
            if (this.mcsHistory[i] !== this.mcsHistory[i - 1]) {
                switches++;
            }
        }

        return {
            totalSteps: this.currentStep,
            avgThroughput: this.throughputHistory.reduce((a, b) => a + b, 0) / this.throughputHistory.length,
            p5Throughput: sortedThroughput[p5Index] ?? 0,
            avgBler: this.blerHistory.reduce((a, b) => a + b, 0) / this.blerHistory.length,
            urllcSuccessRate: this.blerHistory.filter(b => b <= this.config.blerTarget).length / this.blerHistory.length,
            mcsHistogram,
            mcsSwitchRate: switches / Math.max(1, this.mcsHistory.length - 1),
            totalReward: this.totalMetric,
        };
    }

    close(): void {
        // No resources to release
    }
}

/**
 * Create U2U-MCS environment
 */
export function createEnvironment(config: Partial<U2UMcsConfig> = {}): U2UMcsEnvironment {
    return new U2UMcsEnvironment(config);
}
