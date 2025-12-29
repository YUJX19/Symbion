/**
 * @module core/runner
 * @description Unified experiment runner for reproducible experiments
 *
 * Provides the foundational interfaces and runner for iterative decision-making
 * algorithms. This is the core infrastructure layer that domain-specific modules
 * (ai, isac, models, extras) build upon.
 */

import type { Space, SpaceValue } from './space';
import { combineMetricsWithTracking, type MetricSpec, type MetricEvaluationResult } from './objective';
import { evaluateConstraints, type ConstraintSpec, type ConstraintReport } from './constraint';
import type { Logger, StepLogEntry, EpisodeLogEntry, ReportLogEntry } from './logging';
import { createRng, SeededRandom, type TaskConfig } from './repro';
import { NotInitializedError } from './errors';

// ==================== Core Interfaces ====================

/**
 * Iterative decision environment interface
 * 
 * Provides a standardized step/reset pattern for iterative algorithms including:
 * - Optimization methods (e.g., L-BFGS, gradient descent)
 * - Online learning algorithms (e.g., LSTM with online updates)
 * - Heuristic planners and rule-based policies
 * - Reinforcement learning agents (if applicable)
 * 
 * This is the foundational interface for the core module. Domain-specific
 * modules (ai, isac, models) may provide specialized wrappers.
 */
export interface Environment<In = SpaceValue, Out = SpaceValue, Info = Record<string, unknown>> {
    /** Input space definition (state, observation, features) */
    inputSpace: Space;
    /** Output space definition (decision, action, prediction) */
    outputSpace: Space;

    /**
     * Reset the environment to initial state
     * @param seed - Optional seed for reproducibility
     * @returns Initial input and info
     */
    reset(seed?: number): Promise<{ input: In; info: Info }>;

    /**
     * Execute one iteration step
     * @param output - Output decision to apply
     * @returns Step result
     */
    step(output: Out): Promise<{
        input: In;
        metric: number;
        done: boolean;
        truncated: boolean;
        info: Info;
    }>;

    /**
     * Close the environment and release resources
     */
    close(): void;
}

/**
 * Decision maker interface (selects outputs given inputs)
 * 
 * This is the generic interface for any decision-making algorithm.
 */
export interface DecisionMaker<In = SpaceValue, Out = SpaceValue> {
    /** Name for logging */
    name: string;

    /**
     * Make a decision given input
     * @param input - Current input state
     * @param rng - Seeded random generator for reproducibility
     * @returns Output decision
     */
    decide(input: In, rng: SeededRandom): Out | Promise<Out>;

    /**
     * Reset internal state (e.g., recurrent state, optimizer state)
     */
    reset?(): void;
}

// ==================== Runner Configuration ====================

/**
 * Runner configuration
 */
export interface RunnerConfig<S = unknown> {
    /** Task configuration */
    taskConfig: TaskConfig;
    /** Environment instance */
    environment: Environment;
    /** Decision maker instance */
    decisionMaker: DecisionMaker;
    /** Metrics to track (optional) */
    metrics?: MetricSpec<S>[];
    /** Constraints to check (optional) */
    constraints?: ConstraintSpec<S>[];
    /** State extractor (extracts metric/constraint state from step info) */
    stateExtractor?: (input: SpaceValue, info: Record<string, unknown>) => S;
    /** Logger(s) for output */
    loggers?: Logger[];
    /** Maximum steps per episode */
    maxStepsPerEpisode?: number;
    /** Total episodes to run */
    totalEpisodes?: number;
    /** Stop on hard constraint violation */
    stopOnConstraintViolation?: boolean;
}

/**
 * Step result with metrics and constraints
 */
export interface StepResult<In = SpaceValue> {
    /** Current input (state) */
    input: In;
    /** Applied output (decision) */
    output: SpaceValue;
    /** Performance metric (reward/cost/error) */
    metric: number;
    /** Episode terminated */
    done: boolean;
    /** Episode truncated (time limit) */
    truncated: boolean;
    /** Additional info */
    info: Record<string, unknown>;
    /** Tracked metrics */
    metrics?: MetricEvaluationResult;
    /** Constraint evaluation */
    constraints?: ConstraintReport;
}

/**
 * Episode result
 */
export interface EpisodeResult {
    episode: number;
    totalSteps: number;
    totalMetric: number;
    avgMetric: number;
    success: boolean;
    constraintViolations: number;
    metrics?: Record<string, number>;
    steps?: StepResult[];
}

/**
 * Run result (full experiment)
 */
export interface RunResult {
    taskConfig: TaskConfig;
    totalEpisodes: number;
    totalSteps: number;
    avgEpisodeMetric: number;
    avgEpisodeLength: number;
    successRate: number;
    objectiveBreakdown?: Record<string, number>;
    episodes: EpisodeResult[];
}

// ==================== Runner Class ====================

/**
 * Unified experiment runner
 *
 * Orchestrates the experiment loop: reset → step → collect → log
 */
export class Runner<S = unknown> {
    private config: Required<RunnerConfig<S>>;
    private rng: SeededRandom;
    private currentEpisode: number = 0;
    private currentStep: number = 0;
    private initialized: boolean = false;
    private currentInput: SpaceValue | null = null;
    private metricsCombiner?: ReturnType<typeof combineMetricsWithTracking<S>>;

    constructor(config: RunnerConfig<S>) {
        this.config = {
            metrics: [],
            constraints: [],
            stateExtractor: (input, info) => ({ input, ...info } as S),
            loggers: [],
            maxStepsPerEpisode: 1000,
            totalEpisodes: 1,
            stopOnConstraintViolation: false,
            ...config,
        };

        this.rng = createRng(config.taskConfig.seed);

        // Create metrics combiner if metrics provided
        if (this.config.metrics.length > 0) {
            this.metricsCombiner = combineMetricsWithTracking(this.config.metrics);
        }
    }

    /**
     * Reset the environment for a new episode
     */
    async reset(): Promise<SpaceValue> {
        this.currentStep = 0;
        this.config.decisionMaker.reset?.();

        const { input } = await this.config.environment.reset(
            this.config.taskConfig.seed + this.currentEpisode
        );

        this.currentInput = input;
        this.initialized = true;

        return input;
    }

    /**
     * Take a single step
     */
    async step(): Promise<StepResult> {
        if (!this.initialized || this.currentInput === null) {
            throw new NotInitializedError();
        }

        // Get output from decision maker
        const output = await this.config.decisionMaker.decide(
            this.currentInput,
            this.rng
        );

        // Step environment
        const stepResult = await this.config.environment.step(output);
        this.currentStep++;

        // Extract state for metrics/constraints
        const state = this.config.stateExtractor(
            stepResult.input,
            stepResult.info
        );

        // Evaluate metrics
        const metrics = this.metricsCombiner?.evaluate(state);

        // Evaluate constraints
        const constraints = this.config.constraints.length > 0
            ? evaluateConstraints(this.config.constraints, state)
            : undefined;

        // Check for constraint-based termination
        let done = stepResult.done;
        if (this.config.stopOnConstraintViolation && constraints && !constraints.feasible) {
            done = true;
        }

        // Check for max steps truncation
        const truncated = stepResult.truncated ||
            this.currentStep >= this.config.maxStepsPerEpisode;

        // Log step
        for (const logger of this.config.loggers) {
            const logEntry: Omit<StepLogEntry, 'logType' | 'schemaVersion' | 'timestamp'> = {
                task: this.config.taskConfig.taskName,
                seed: this.config.taskConfig.seed,
                episode: this.currentEpisode,
                step: this.currentStep,
                observation: this.currentInput,  // Logger still uses observation for compatibility
                action: output,
                reward: stepResult.metric,
                done,
                truncated,
                info: stepResult.info,
                metrics: metrics ? Object.fromEntries(
                    Object.entries(metrics.breakdown).map(([k, v]) => [k, v.raw])
                ) : undefined,
            };
            logger.logStep(logEntry);
        }

        // Update current input
        this.currentInput = stepResult.input;

        return {
            input: stepResult.input,
            output,
            metric: stepResult.metric,
            done,
            truncated,
            info: stepResult.info,
            metrics,
            constraints,
        };
    }

    /**
     * Run a single episode
     */
    async runEpisode(): Promise<EpisodeResult> {
        await this.reset();

        const steps: StepResult[] = [];
        let totalMetric = 0;
        let constraintViolations = 0;
        let done = false;
        let truncated = false;

        while (!done && !truncated) {
            const result = await this.step();
            steps.push(result);
            totalMetric += result.metric;

            if (result.constraints && !result.constraints.feasible) {
                constraintViolations++;
            }

            done = result.done;
            truncated = result.truncated;
        }

        const episodeResult: EpisodeResult = {
            episode: this.currentEpisode,
            totalSteps: steps.length,
            totalMetric,
            avgMetric: steps.length > 0 ? totalMetric / steps.length : 0,
            success: done && constraintViolations === 0,
            constraintViolations,
            metrics: steps.length > 0 && steps[steps.length - 1].metrics
                ? Object.fromEntries(
                    Object.entries(steps[steps.length - 1].metrics!.breakdown)
                        .map(([k, v]) => [k, v.raw])
                )
                : undefined,
            steps,
        };

        // Log episode
        for (const logger of this.config.loggers) {
            const logEntry: Omit<EpisodeLogEntry, 'logType' | 'schemaVersion' | 'timestamp'> = {
                task: this.config.taskConfig.taskName,
                seed: this.config.taskConfig.seed,
                episode: this.currentEpisode,
                totalSteps: episodeResult.totalSteps,
                totalReward: episodeResult.totalMetric,  // Logger still uses reward for compatibility
                avgReward: episodeResult.avgMetric,
                success: episodeResult.success,
                constraintViolations: episodeResult.constraintViolations,
                metrics: episodeResult.metrics,
            };
            logger.logEpisode(logEntry);
        }

        this.currentEpisode++;
        return episodeResult;
    }

    /**
     * Run the full experiment (all episodes)
     */
    async run(): Promise<RunResult> {
        const episodes: EpisodeResult[] = [];

        for (let i = 0; i < this.config.totalEpisodes; i++) {
            const episodeResult = await this.runEpisode();
            episodes.push(episodeResult);
        }

        // Compute aggregate statistics
        const totalSteps = episodes.reduce((sum, e) => sum + e.totalSteps, 0);
        const avgEpisodeMetric = episodes.reduce((sum, e) => sum + e.totalMetric, 0) / episodes.length;
        const avgEpisodeLength = totalSteps / episodes.length;
        const successRate = episodes.filter(e => e.success).length / episodes.length;

        // Compute objective breakdown (average across episodes)
        let objectiveBreakdown: Record<string, number> | undefined;
        if (episodes.length > 0 && episodes[0].metrics) {
            objectiveBreakdown = {};
            const keys = Object.keys(episodes[0].metrics);
            for (const key of keys) {
                const values = episodes
                    .filter(e => e.metrics && key in e.metrics)
                    .map(e => e.metrics![key]);
                objectiveBreakdown[key] = values.reduce((a, b) => a + b, 0) / values.length;
            }
        }

        const runResult: RunResult = {
            taskConfig: this.config.taskConfig,
            totalEpisodes: episodes.length,
            totalSteps,
            avgEpisodeMetric,
            avgEpisodeLength,
            successRate,
            objectiveBreakdown,
            episodes,
        };

        // Log final report
        for (const logger of this.config.loggers) {
            const logEntry: Omit<ReportLogEntry, 'logType' | 'schemaVersion' | 'timestamp'> = {
                task: this.config.taskConfig.taskName,
                seed: this.config.taskConfig.seed,
                totalEpisodes: runResult.totalEpisodes,
                totalSteps: runResult.totalSteps,
                avgEpisodeReward: runResult.avgEpisodeMetric,  // Logger still uses reward
                avgEpisodeLength: runResult.avgEpisodeLength,
                successRate: runResult.successRate,
                objectiveBreakdown: runResult.objectiveBreakdown,
                config: this.config.taskConfig.hyperparams,
            };
            logger.logReport(logEntry);
            logger.flush();
        }

        return runResult;
    }

    /**
     * Close the runner and release resources
     */
    close(): void {
        this.config.environment.close();
        for (const logger of this.config.loggers) {
            logger.close();
        }
    }
}

// ==================== Factory Functions ====================

/**
 * Create and run a quick experiment
 */
export async function runExperiment<S = unknown>(
    config: RunnerConfig<S>
): Promise<RunResult> {
    const runner = new Runner(config);
    try {
        return await runner.run();
    } finally {
        runner.close();
    }
}

// ==================== Dummy Components (for testing) ====================

/**
 * Create a dummy environment for testing
 */
export function createDummyEnvironment(
    inputDim: number = 4,
    outputDim: number = 2
): Environment {
    let step = 0;
    const maxSteps = 100;

    return {
        inputSpace: { kind: 'box', shape: [inputDim], low: -1, high: 1 },
        outputSpace: { kind: 'box', shape: [outputDim], low: -1, high: 1 },

        async reset(seed?: number) {
            step = 0;
            const rng = createRng(seed ?? 0);
            const input = Array(inputDim).fill(0).map(() => rng.uniform(-1, 1));
            return { input, info: {} };
        },

        async step(_output) {
            step++;
            const input = Array(inputDim).fill(0).map(() => Math.random() * 2 - 1);
            const metric = -Math.abs(input[0]); // Simple metric
            const done = step >= maxSteps;

            return {
                input,
                metric,
                done,
                truncated: false,
                info: { step },
            };
        },

        close() { /* no-op */ },
    };
}

/**
 * Create a random decision maker for testing
 */
export function createRandomDecisionMaker(outputSpace: Space): DecisionMaker {
    return {
        name: 'random',

        decide(_input, rng) {
            // Inline sample implementation to avoid circular dependency
            if (outputSpace.kind === 'discrete') {
                return rng.randint(0, outputSpace.n);
            } else if (outputSpace.kind === 'box') {
                const size = outputSpace.shape.reduce((a: number, b: number) => a * b, 1);
                const result: number[] = [];
                const low = typeof outputSpace.low === 'number' ? outputSpace.low : -1;
                const high = typeof outputSpace.high === 'number' ? outputSpace.high : 1;
                for (let i = 0; i < size; i++) {
                    result.push(rng.uniform(low, high));
                }
                return result;
            } else if (outputSpace.kind === 'multiDiscrete') {
                return outputSpace.nvec.map(n => rng.randint(0, n));
            }
            throw new Error(`Unsupported output space kind: ${outputSpace.kind}`);
        },

        reset() { /* no-op */ },
    };
}
