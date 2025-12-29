/**
 * Core Runner Module Tests
 * Tests for the unified experiment runner framework
 */

import { describe, it, expect, beforeEach, vi } from 'vitest';
import {
    // Runner
    Runner,
    runExperiment,
    createDummyEnvironment,
    createRandomDecisionMaker,
    // Types
    type Environment,
    type DecisionMaker,
    type RunnerConfig,
    type StepResult,
    type EpisodeResult,
    type RunResult,
    // Task config
    createTaskConfig,
    // Metrics
    simpleMetric,
    type MetricSpec,
    // Constraints
    leConstraint,
    geConstraint,
    type ConstraintSpec,
    // Space
    box,
    discrete,
    // Logging
    MemoryLogger,
    type LoggerConfig,
    // Errors
    NotInitializedError,
} from '../src/core';

// ==================== Test Utils ====================

function createTestLoggerConfig(): LoggerConfig {
    return { task: 'test-task', seed: 42 };
}

function createTestLogger() {
    const logger = new MemoryLogger(createTestLoggerConfig());
    return logger;
}

function createFullTaskConfig(overrides: { taskName?: string; seed?: number; hyperparams?: Record<string, unknown> } = {}) {
    return createTaskConfig({
        taskName: overrides.taskName ?? 'test-task',
        seed: overrides.seed ?? 42,
        schemaHash: 'test-schema-hash-12345678',
        hyperparams: overrides.hyperparams ?? {},
    });
}

// ==================== Test State Type ====================

interface TestState {
    value: number;
    reward: number;
    step: number;
}

// ==================== Mock Environment ====================

function createMockEnvironment(options: {
    maxSteps?: number;
    metricPerStep?: number;
} = {}): Environment {
    const { maxSteps = 10, metricPerStep = 1.0 } = options;
    let currentStep = 0;

    return {
        inputSpace: box([4], -1, 1),
        outputSpace: box([2], -1, 1),

        async reset(_seed?: number) {
            currentStep = 0;
            return {
                input: [0, 0, 0, 0],
                info: { step: 0 },
            };
        },

        async step(_action) {
            currentStep++;
            const done = currentStep >= maxSteps;
            return {
                input: [currentStep / maxSteps, 0, 0, 0],
                metric: metricPerStep,
                done,
                truncated: false,
                info: { step: currentStep, value: currentStep * 10 },
            };
        },

        close() { },
    };
}

// ==================== Mock Decision Maker ====================

function createMockDecisionMaker(): DecisionMaker {
    return {
        name: 'mock-decision-maker',
        decide(_input, _rng) {
            return [0, 0];
        },
        reset() { },
    };
}

// ==================== createDummyEnvironment ====================

describe('createDummyEnvironment', () => {
    it('should create environment with default dimensions', () => {
        const env = createDummyEnvironment();
        expect(env.inputSpace).toBeDefined();
        expect(env.outputSpace).toBeDefined();
    });

    it('should create environment with custom dimensions', () => {
        const env = createDummyEnvironment(10, 5);
        expect((env.inputSpace as any).shape).toEqual([10]);
        expect((env.outputSpace as any).shape).toEqual([5]);
    });

    it('should reset to initial state', async () => {
        const env = createDummyEnvironment(4);
        const { input, info } = await env.reset(42);

        expect(input).toHaveLength(4);
        expect(info).toBeDefined();
    });

    it('should step with observation and reward', async () => {
        const env = createDummyEnvironment();
        await env.reset();
        const result = await env.step([0, 0]);

        expect(result.input).toBeDefined();
        expect(typeof result.metric).toBe('number');
        expect(typeof result.done).toBe('boolean');
        expect(typeof result.truncated).toBe('boolean');
    });

    it('should terminate after max steps', async () => {
        const env = createDummyEnvironment();
        await env.reset();

        let done = false;
        let steps = 0;
        while (!done && steps < 200) {
            const result = await env.step([0, 0]);
            done = result.done;
            steps++;
        }

        expect(done).toBe(true);
        expect(steps).toBe(100); // Default max steps
    });
});

// ==================== createRandomDecisionMaker ====================

describe('createRandomDecisionMaker', () => {
    it('should create policy for box action space', () => {
        const policy = createRandomDecisionMaker(box([3], -1, 1));
        expect(policy.name).toBe('random');
    });

    it('should create policy for discrete action space', () => {
        const policy = createRandomDecisionMaker(discrete(5));
        expect(policy.name).toBe('random');
    });

    it('should select valid actions from box space', async () => {
        const outputSpace = box([2], -1, 1);
        const policy = createRandomDecisionMaker(outputSpace);
        const { createRng } = await import('../src/core');
        const rng = createRng(42);

        const action = policy.decide([0, 0, 0, 0], rng) as number[];

        expect(action).toHaveLength(2);
        action.forEach(a => {
            expect(a).toBeGreaterThanOrEqual(-1);
            expect(a).toBeLessThanOrEqual(1);
        });
    });

    it('should select valid actions from discrete space', async () => {
        const outputSpace = discrete(5);
        const policy = createRandomDecisionMaker(outputSpace);
        const { createRng } = await import('../src/core');
        const rng = createRng(42);

        const action = policy.decide([0, 0, 0, 0], rng) as number;

        expect(Number.isInteger(action)).toBe(true);
        expect(action).toBeGreaterThanOrEqual(0);
        expect(action).toBeLessThan(5);
    });
});

// ==================== Runner ====================

describe('Runner', () => {
    let taskConfig: ReturnType<typeof createTaskConfig>;

    beforeEach(() => {
        taskConfig = createFullTaskConfig();
    });

    describe('constructor', () => {
        it('should create runner with minimal config', () => {
            const runner = new Runner({
                taskConfig,
                environment: createMockEnvironment(),
                decisionMaker: createMockDecisionMaker(),
            });

            expect(runner).toBeDefined();
            runner.close();
        });

        it('should create runner with full config', () => {
            const metrics: MetricSpec<TestState>[] = [
                simpleMetric('value', (s) => s.value, 'maximize'),
            ];

            const constraints: ConstraintSpec<TestState>[] = [
                leConstraint('max_value', (s) => s.value, 100, 'hard'),
            ];

            const runner = new Runner<TestState>({
                taskConfig,
                environment: createMockEnvironment(),
                decisionMaker: createMockDecisionMaker(),
                metrics,
                constraints,
                stateExtractor: (obs, info) => ({
                    value: (info.value as number) ?? 0,
                    reward: 0,
                    step: (info.step as number) ?? 0,
                }),
                maxStepsPerEpisode: 50,
                totalEpisodes: 5,
            });

            expect(runner).toBeDefined();
            runner.close();
        });
    });

    describe('reset', () => {
        it('should reset and return initial observation', async () => {
            const runner = new Runner({
                taskConfig,
                environment: createMockEnvironment(),
                decisionMaker: createMockDecisionMaker(),
            });

            const input = await runner.reset();

            expect(input).toBeDefined();
            expect(Array.isArray(input)).toBe(true);
            runner.close();
        });
    });

    describe('step', () => {
        it('should throw if not initialized', async () => {
            const runner = new Runner({
                taskConfig,
                environment: createMockEnvironment(),
                decisionMaker: createMockDecisionMaker(),
            });

            await expect(runner.step()).rejects.toThrow(NotInitializedError);
            runner.close();
        });

        it('should complete step after reset', async () => {
            const runner = new Runner({
                taskConfig,
                environment: createMockEnvironment(),
                decisionMaker: createMockDecisionMaker(),
            });

            await runner.reset();
            const result = await runner.step();

            expect(result).toMatchObject({
                input: expect.any(Array),
                output: expect.any(Array),
                metric: expect.any(Number),
                done: expect.any(Boolean),
                truncated: expect.any(Boolean),
                info: expect.any(Object),
            });

            runner.close();
        });

        it('should include metrics when configured', async () => {
            const runner = new Runner<TestState>({
                taskConfig,
                environment: createMockEnvironment(),
                decisionMaker: createMockDecisionMaker(),
                metrics: [
                    simpleMetric('value', (s) => s.value, 'maximize'),
                ],
                stateExtractor: (obs, info) => ({
                    value: (info.value as number) ?? 0,
                    reward: 0,
                    step: 0,
                }),
            });

            await runner.reset();
            const result = await runner.step();

            expect(result.metrics).toBeDefined();
            expect(result.metrics!.breakdown.value).toBeDefined();
            runner.close();
        });

        it('should include constraints when configured', async () => {
            const runner = new Runner<TestState>({
                taskConfig,
                environment: createMockEnvironment(),
                decisionMaker: createMockDecisionMaker(),
                constraints: [
                    leConstraint('max_value', (s) => s.value, 100, 'hard'),
                ],
                stateExtractor: (obs, info) => ({
                    value: (info.value as number) ?? 0,
                    reward: 0,
                    step: 0,
                }),
            });

            await runner.reset();
            const result = await runner.step();

            expect(result.constraints).toBeDefined();
            expect(result.constraints!.results).toHaveLength(1);
            runner.close();
        });
    });

    describe('runEpisode', () => {
        it('should run complete episode', async () => {
            const runner = new Runner({
                taskConfig,
                environment: createMockEnvironment({ maxSteps: 5 }),
                decisionMaker: createMockDecisionMaker(),
            });

            const result = await runner.runEpisode();

            expect(result.episode).toBe(0);
            expect(result.totalSteps).toBe(5);
            expect(result.totalMetric).toBe(5); // 5 steps * 1.0 reward
            expect(result.steps).toHaveLength(5);

            runner.close();
        });

        it('should track constraint violations', async () => {
            const runner = new Runner<TestState>({
                taskConfig,
                environment: createMockEnvironment({ maxSteps: 5 }),
                decisionMaker: createMockDecisionMaker(),
                constraints: [
                    // Will fail when value > 25 (step 3+)
                    leConstraint('max_value', (s) => s.value, 25, 'hard'),
                ],
                stateExtractor: (obs, info) => ({
                    value: (info.value as number) ?? 0,
                    reward: 0,
                    step: 0,
                }),
            });

            const result = await runner.runEpisode();

            expect(result.constraintViolations).toBeGreaterThan(0);
            runner.close();
        });

        it('should stop on constraint violation when configured', async () => {
            const runner = new Runner<TestState>({
                taskConfig,
                environment: createMockEnvironment({ maxSteps: 10 }),
                decisionMaker: createMockDecisionMaker(),
                constraints: [
                    // Will fail when value > 25 (step 3+)
                    leConstraint('max_value', (s) => s.value, 25, 'hard'),
                ],
                stateExtractor: (obs, info) => ({
                    value: (info.value as number) ?? 0,
                    reward: 0,
                    step: 0,
                }),
                stopOnConstraintViolation: true,
            });

            const result = await runner.runEpisode();

            expect(result.totalSteps).toBeLessThan(10);
            runner.close();
        });

        it('should respect maxStepsPerEpisode', async () => {
            const runner = new Runner({
                taskConfig,
                environment: createMockEnvironment({ maxSteps: 100 }),
                decisionMaker: createMockDecisionMaker(),
                maxStepsPerEpisode: 5,
            });

            const result = await runner.runEpisode();

            expect(result.totalSteps).toBe(5);
            runner.close();
        });
    });

    describe('run', () => {
        it('should run all episodes', async () => {
            const runner = new Runner({
                taskConfig,
                environment: createMockEnvironment({ maxSteps: 3 }),
                decisionMaker: createMockDecisionMaker(),
                totalEpisodes: 5,
            });

            const result = await runner.run();

            expect(result.totalEpisodes).toBe(5);
            expect(result.episodes).toHaveLength(5);

            runner.close();
        });

        it('should calculate aggregate statistics', async () => {
            const runner = new Runner({
                taskConfig,
                environment: createMockEnvironment({ maxSteps: 5, metricPerStep: 2 }),
                decisionMaker: createMockDecisionMaker(),
                totalEpisodes: 3,
            });

            const result = await runner.run();

            expect(result.avgEpisodeMetric).toBe(10); // 5 steps * 2 reward
            expect(result.avgEpisodeLength).toBe(5);
            expect(result.totalSteps).toBe(15); // 3 episodes * 5 steps

            runner.close();
        });

        it('should include task config', async () => {
            const runner = new Runner({
                taskConfig,
                environment: createMockEnvironment({ maxSteps: 1 }),
                decisionMaker: createMockDecisionMaker(),
                totalEpisodes: 1,
            });

            const result = await runner.run();

            expect(result.taskConfig).toBe(taskConfig);
            runner.close();
        });
    });

    describe('logging', () => {
        it('should log steps to loggers', async () => {
            const logger = createTestLogger();

            const runner = new Runner({
                taskConfig,
                environment: createMockEnvironment({ maxSteps: 3 }),
                decisionMaker: createMockDecisionMaker(),
                loggers: [logger],
            });

            await runner.reset();
            await runner.step();

            const entries = logger.getAllLogs();
            expect(entries.length).toBeGreaterThan(0);

            runner.close();
        });

        it('should log episodes to loggers', async () => {
            const logger = createTestLogger();

            const runner = new Runner({
                taskConfig,
                environment: createMockEnvironment({ maxSteps: 2 }),
                decisionMaker: createMockDecisionMaker(),
                loggers: [logger],
            });

            await runner.runEpisode();

            const entries = logger.getAllLogs();
            const episodeEntries = entries.filter(
                (e: any) => e.logType === 'episode'
            );
            expect(episodeEntries.length).toBe(1);

            runner.close();
        });

        it('should log report at end of run', async () => {
            const logger = createTestLogger();

            const runner = new Runner({
                taskConfig,
                environment: createMockEnvironment({ maxSteps: 1 }),
                decisionMaker: createMockDecisionMaker(),
                loggers: [logger],
                totalEpisodes: 1,
            });

            await runner.run();

            const entries = logger.getAllLogs();
            const reportEntries = entries.filter(
                (e: any) => e.logType === 'report'
            );
            expect(reportEntries.length).toBe(1);

            runner.close();
        });
    });

    describe('close', () => {
        it('should close environment and loggers', () => {
            const closeSpy = vi.fn();
            const env = createMockEnvironment();
            env.close = closeSpy;

            const loggerCloseSpy = vi.fn();
            const logger = createTestLogger();
            logger.close = loggerCloseSpy;

            const runner = new Runner({
                taskConfig,
                environment: env,
                decisionMaker: createMockDecisionMaker(),
                loggers: [logger],
            });

            runner.close();

            expect(closeSpy).toHaveBeenCalled();
            expect(loggerCloseSpy).toHaveBeenCalled();
        });
    });
});

// ==================== runExperiment ====================

describe('runExperiment', () => {
    it('should run and close runner automatically', async () => {
        const closeSpy = vi.fn();
        const env = createMockEnvironment({ maxSteps: 2 });
        const originalClose = env.close;
        env.close = () => {
            originalClose();
            closeSpy();
        };

        const result = await runExperiment({
            taskConfig: createFullTaskConfig({ taskName: 'test', seed: 1 }),
            environment: env,
            decisionMaker: createMockDecisionMaker(),
            totalEpisodes: 1,
        });

        expect(result.totalEpisodes).toBe(1);
        expect(closeSpy).toHaveBeenCalled();
    });
});

// ==================== Integration Test ====================

describe('Runner Integration', () => {
    it('should complete full experiment workflow', async () => {
        const taskConfig = createFullTaskConfig({
            taskName: 'integration-test',
            seed: 12345,
            hyperparams: { learningRate: 0.01 },
        });

        const metrics: MetricSpec<TestState>[] = [
            simpleMetric('value', (s) => s.value, 'maximize'),
            simpleMetric('reward', (s) => s.metric, 'maximize'),
        ];

        const constraints: ConstraintSpec<TestState>[] = [
            leConstraint('max_value', (s) => s.value, 1000, 'hard'),
            geConstraint('min_value', (s) => s.value, 0, 'soft'),
        ];

        const logger = createTestLogger();

        const result = await runExperiment<TestState>({
            taskConfig,
            environment: createMockEnvironment({ maxSteps: 5, metricPerStep: 2 }),
            decisionMaker: createMockDecisionMaker(),
            metrics,
            constraints,
            stateExtractor: (obs, info) => ({
                value: (info.value as number) ?? 0,
                reward: 2,
                step: (info.step as number) ?? 0,
            }),
            loggers: [logger],
            maxStepsPerEpisode: 5,
            totalEpisodes: 3,
        });

        // Check result structure
        expect(result.taskConfig).toBe(taskConfig);
        expect(result.totalEpisodes).toBe(3);
        expect(result.totalSteps).toBe(15);
        expect(result.avgEpisodeMetric).toBe(10);
        expect(result.avgEpisodeLength).toBe(5);
        expect(result.objectiveBreakdown).toBeDefined();
        expect(result.objectiveBreakdown!.value).toBeDefined();

        // Check logging
        const entries = logger.getAllLogs();
        const stepEntries = entries.filter((e: any) => e.logType === 'step');
        const episodeEntries = entries.filter((e: any) => e.logType === 'episode');
        const reportEntries = entries.filter((e: any) => e.logType === 'report');

        expect(stepEntries.length).toBe(15); // 3 episodes * 5 steps
        expect(episodeEntries.length).toBe(3);
        expect(reportEntries.length).toBe(1);
    });
});
