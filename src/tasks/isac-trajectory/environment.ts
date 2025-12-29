/**
 * @module tasks/isac-trajectory/environment
 * @description ISAC Trajectory Optimization Environment
 * 
 * Simulation environment for evaluating UAV trajectory planning methods.
 * Suitable for heuristic planners and iterative optimization algorithms.
 */

import type { Environment } from '../../core/runner';
import type { Space } from '../../core/space';
import { createRng, SeededRandom } from '../../core/repro';
import { createConfig, type IsacTrajectoryConfig, type Position3D } from './config';
import { IsacScenario, ScenarioState } from './scenario';
import {
    extractInput,
    inputToVector,
    getInputSpace,
    getOutputSpace,
    continuousOutputToOffset,
} from './observation';
import { calculateReward, RewardBreakdown, calculateEpisodeMetrics, EpisodeMetrics } from './reward';

// ==================== Types ====================

/**
 * Step info for ISAC trajectory
 */
export interface IsacStepInfo {
    step: number;
    position: Position3D;
    velocity: Position3D;
    losPercentage: number;
    totalThroughput: number;
    urllcSatisfied: boolean[];
    sensingCoverage: number;
    energyUsed: number;
    rewardBreakdown: RewardBreakdown;
}

// ==================== Environment Class ====================

/**
 * ISAC Trajectory Environment
 */
export class IsacTrajectoryEnvironment implements Environment<number[], number[], IsacStepInfo> {
    readonly inputSpace: Space;
    readonly outputSpace: Space;

    private config: IsacTrajectoryConfig;
    private scenario: IsacScenario;
    private rng: SeededRandom;
    private currentStep: number = 0;
    private previousState: ScenarioState | null = null;
    private currentState: ScenarioState | null = null;

    // History for episode metrics
    private stateHistory: ScenarioState[] = [];
    private metricHistory: number[] = [];

    constructor(config: Partial<IsacTrajectoryConfig> = {}) {
        this.config = createConfig(config);
        this.inputSpace = getInputSpace(this.config);
        this.outputSpace = getOutputSpace(this.config, true);

        this.scenario = new IsacScenario(this.config);
        this.rng = createRng(this.config.seed);
    }

    async reset(seed?: number): Promise<{ input: number[]; info: IsacStepInfo }> {
        const actualSeed = seed ?? this.config.seed;
        this.rng = createRng(actualSeed);
        this.scenario.reset(actualSeed);
        this.currentStep = 0;
        this.previousState = null;
        this.stateHistory = [];
        this.metricHistory = [];

        // Generate initial state
        this.currentState = this.scenario.generateState();
        this.stateHistory.push(this.currentState);

        const inputState = extractInput(this.currentState, this.config);
        const input = inputToVector(inputState);

        const info: IsacStepInfo = {
            step: 0,
            position: { ...this.currentState.uav.position },
            velocity: { ...this.currentState.uav.velocity },
            losPercentage: this.currentState.losPercentage,
            totalThroughput: this.currentState.totalThroughput,
            urllcSatisfied: [...this.currentState.urllcSatisfied],
            sensingCoverage: this.currentState.sensingCoverage,
            energyUsed: this.currentState.uav.energyUsed,
            rewardBreakdown: {
                total: 0,
                throughputReward: 0,
                losPersistenceReward: 0,
                energyPenalty: 0,
                urllcPenalty: 0,
                sensingPenalty: 0,
            },
        };

        return { input, info };
    }

    async step(output: number[]): Promise<{
        input: number[];
        metric: number;
        done: boolean;
        truncated: boolean;
        info: IsacStepInfo;
    }> {
        if (!this.currentState) {
            throw new Error('Environment not reset. Call reset() first.');
        }

        // Convert output to position offset
        const offset = continuousOutputToOffset(output, this.config);

        // Calculate target position
        const currentPos = this.scenario.getCurrentPosition();
        const targetPosition: Position3D = {
            x: currentPos.x + offset.x,
            y: currentPos.y + offset.y,
            z: currentPos.z + offset.z,
        };

        // Move UAV
        this.scenario.moveUav(targetPosition);

        // Store previous state
        this.previousState = this.currentState;

        // Generate new state
        this.currentState = this.scenario.generateState();
        this.currentStep++;
        this.stateHistory.push(this.currentState);

        // Calculate metric
        const rewardBreakdown = calculateReward(
            this.currentState,
            this.previousState,
            this.config
        );
        this.metricHistory.push(rewardBreakdown.total);

        // Extract input
        const inputState = extractInput(this.currentState, this.config);
        const input = inputToVector(inputState);

        // Check termination
        const done = false;
        const truncated = this.currentStep >= this.config.episodeHorizon;

        const info: IsacStepInfo = {
            step: this.currentStep,
            position: { ...this.currentState.uav.position },
            velocity: { ...this.currentState.uav.velocity },
            losPercentage: this.currentState.losPercentage,
            totalThroughput: this.currentState.totalThroughput,
            urllcSatisfied: [...this.currentState.urllcSatisfied],
            sensingCoverage: this.currentState.sensingCoverage,
            energyUsed: this.currentState.uav.energyUsed,
            rewardBreakdown,
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
     * Get episode metrics
     */
    getEpisodeMetrics(): EpisodeMetrics {
        return calculateEpisodeMetrics(this.stateHistory, this.config);
    }

    /**
     * Get state history for analysis
     */
    getStateHistory(): ScenarioState[] {
        return [...this.stateHistory];
    }

    /**
     * Get metric history
     */
    getMetricHistory(): number[] {
        return [...this.metricHistory];
    }

    close(): void {
        // No resources to release
    }
}

/**
 * Create ISAC trajectory environment
 */
export function createEnvironment(config: Partial<IsacTrajectoryConfig> = {}): IsacTrajectoryEnvironment {
    return new IsacTrajectoryEnvironment(config);
}
