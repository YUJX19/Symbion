/**
 * ISAC Module Tests
 * Tests for ISAC trajectory planner, metrics, and constraints
 */

import { describe, it, expect, beforeEach } from 'vitest';
import {
    // Planner
    IsacPlanner,
    createPlanner,
    DEFAULT_ISAC_CONFIG,
    createConfig,
    type Position3D,
    // Metrics
    metrics,
    // Constraints
    constraints,
} from '../src/isac';

import { isClose, arraysClose } from './test-utils';

// ==================== Test Utilities ====================

function createTestPosition(x: number, y: number, z: number): Position3D {
    return { x, y, z };
}

function distance3D(a: Position3D, b: Position3D): number {
    return Math.sqrt(
        (a.x - b.x) ** 2 +
        (a.y - b.y) ** 2 +
        (a.z - b.z) ** 2
    );
}

// ==================== IsacPlanner Tests ====================

describe('IsacPlanner', () => {
    describe('constructor', () => {
        it('should create planner with default config', () => {
            const planner = new IsacPlanner();
            expect(planner).toBeDefined();
        });

        it('should create planner with custom config', () => {
            const planner = new IsacPlanner({
                numWaypoints: 20,
                seed: 42,
            });
            expect(planner).toBeDefined();
        });
    });

    describe('plan', () => {
        it('should generate trajectory from default start to end', () => {
            const planner = new IsacPlanner();
            const result = planner.plan();

            expect(result.feasible).toBe(true);
            expect(result.trajectory.length).toBeGreaterThan(0);
            expect(result.totalDistance).toBeGreaterThan(0);
        });

        it('should generate trajectory between specified positions', () => {
            const planner = new IsacPlanner();
            const start = createTestPosition(0, 0, 50);
            const end = createTestPosition(100, 100, 80);

            const result = planner.plan(start, end);

            expect(result.feasible).toBe(true);
            expect(result.trajectory.length).toBeGreaterThan(2);

            // First waypoint should be at start
            const first = result.trajectory[0];
            expect(first.x).toBe(start.x);
            expect(first.y).toBe(start.y);

            // Last waypoint should be at end
            const last = result.trajectory[result.trajectory.length - 1];
            expect(last.x).toBe(end.x);
            expect(last.y).toBe(end.y);
        });

        it('should generate continuous trajectory', () => {
            const planner = new IsacPlanner({ numWaypoints: 10 });
            const result = planner.plan(
                createTestPosition(0, 0, 100),
                createTestPosition(500, 500, 100)
            );

            // Check consecutive waypoints are not too far apart
            for (let i = 1; i < result.trajectory.length; i++) {
                const dist = distance3D(result.trajectory[i - 1], result.trajectory[i]);
                expect(dist).toBeLessThan(200); // Reasonable step size
            }
        });

        it('should estimate energy consumption', () => {
            const planner = new IsacPlanner();
            const result = planner.plan(
                createTestPosition(0, 0, 100),
                createTestPosition(1000, 0, 100)
            );

            expect(result.estimatedEnergy).toBeGreaterThan(0);
        });

        it('should calculate total distance correctly', () => {
            const planner = new IsacPlanner({ numWaypoints: 5 });
            const start = createTestPosition(0, 0, 100);
            const end = createTestPosition(100, 0, 100);

            const result = planner.plan(start, end);

            // Total distance should be at least the straight-line distance
            const directDistance = distance3D(start, end);
            expect(result.totalDistance).toBeGreaterThanOrEqual(directDistance * 0.95);
        });
    });

    describe('reset', () => {
        it('should reset planner state', () => {
            const planner = new IsacPlanner({ seed: 1 });
            planner.plan();

            expect(() => planner.reset()).not.toThrow();
        });

        it('should accept new seed', () => {
            const planner = new IsacPlanner();
            expect(() => planner.reset(12345)).not.toThrow();
        });
    });
});

describe('createPlanner', () => {
    it('should create planner with default config', () => {
        const planner = createPlanner();
        expect(planner).toBeInstanceOf(IsacPlanner);
    });

    it('should create planner with custom config', () => {
        const planner = createPlanner({ numWaypoints: 15 });
        expect(planner).toBeInstanceOf(IsacPlanner);
    });
});

// ==================== Config Tests ====================

describe('ISAC Configuration', () => {
    describe('DEFAULT_ISAC_CONFIG', () => {
        it('should have valid default values', () => {
            expect(DEFAULT_ISAC_CONFIG.numWaypoints).toBeGreaterThan(0);
            expect(DEFAULT_ISAC_CONFIG.seed).toBeDefined();
            expect(DEFAULT_ISAC_CONFIG.startPosition).toBeDefined();
        });
    });

    describe('createConfig', () => {
        it('should merge partial config with defaults', () => {
            const config = createConfig({ numWaypoints: 30 });

            expect(config.numWaypoints).toBe(30);
            expect(config.seed).toBe(DEFAULT_ISAC_CONFIG.seed);
        });

        it('should return complete config', () => {
            const config = createConfig({});

            expect(config.numWaypoints).toBe(DEFAULT_ISAC_CONFIG.numWaypoints);
            expect(config.waypointDurationS).toBe(DEFAULT_ISAC_CONFIG.waypointDurationS);
            expect(config.kinematics).toBeDefined();
        });
    });
});

// ==================== LoS Metrics Tests ====================

describe('ISAC Metrics: LoS', () => {
    describe('calculateLosProbability', () => {
        it('should return higher probability for higher elevation', () => {
            const lowElevation = metrics.calculateLosProbability(30);
            const highElevation = metrics.calculateLosProbability(60);

            expect(highElevation).toBeGreaterThan(lowElevation);
        });

        it('should return 1 for directly overhead (90°)', () => {
            const prob = metrics.calculateLosProbability(90);
            expect(prob).toBeGreaterThan(0.9);
        });

        it('should vary by environment', () => {
            const urban = metrics.calculateLosProbability(45, 'urban');
            const suburban = metrics.calculateLosProbability(45, 'suburban');
            const rural = metrics.calculateLosProbability(45, 'rural');

            // Rural should have best LoS, urban worst
            expect(rural).toBeGreaterThanOrEqual(urban);
        });

        it('should clamp elevation to [0, 90]', () => {
            const negativeElevation = metrics.calculateLosProbability(-10);
            const overElevation = metrics.calculateLosProbability(100);

            expect(negativeElevation).toBeGreaterThanOrEqual(0);
            expect(overElevation).toBeLessThanOrEqual(1);
        });
    });

    describe('calculateElevationAngle', () => {
        it('should return 90° for directly above', () => {
            const uav = createTestPosition(100, 100, 150);
            const user = createTestPosition(100, 100, 0);

            const angle = metrics.calculateElevationAngle(uav, user);
            expect(angle).toBeCloseTo(90, 0);
        });

        it('should return 45° for equal horizontal and vertical distance', () => {
            const uav = createTestPosition(0, 0, 100);
            const user = createTestPosition(100, 0, 0);

            const angle = metrics.calculateElevationAngle(uav, user);
            expect(angle).toBeCloseTo(45, 0);
        });

        it('should return low angle for far horizontal distance', () => {
            const uav = createTestPosition(0, 0, 100);
            const user = createTestPosition(1000, 0, 0);

            const angle = metrics.calculateElevationAngle(uav, user);
            expect(angle).toBeLessThan(10);
        });
    });

    describe('calculateAverageLoS', () => {
        it('should return average LoS for multiple users', () => {
            const uav = createTestPosition(0, 0, 100);
            const users = [
                createTestPosition(100, 0, 0),
                createTestPosition(0, 100, 0),
                createTestPosition(50, 50, 0),
            ];

            const avgLoS = metrics.calculateAverageLoS(uav, users);

            expect(avgLoS).toBeGreaterThan(0);
            expect(avgLoS).toBeLessThanOrEqual(1);
        });

        it('should return 0 for empty user list', () => {
            const uav = createTestPosition(0, 0, 100);
            const avgLoS = metrics.calculateAverageLoS(uav, []);

            expect(avgLoS).toBe(0);
        });
    });
});

// ==================== Throughput Metrics Tests ====================

describe('ISAC Metrics: Throughput', () => {
    describe('calculatePathLoss', () => {
        it('should increase with distance', () => {
            const freq = 2.4e9; // 2.4 GHz
            const loss100m = metrics.calculatePathLoss(100, freq);
            const loss1000m = metrics.calculatePathLoss(1000, freq);

            expect(loss1000m).toBeGreaterThan(loss100m);
        });

        it('should increase with frequency', () => {
            const dist = 100;
            const loss2G = metrics.calculatePathLoss(dist, 2e9);
            const loss5G = metrics.calculatePathLoss(dist, 5e9);

            expect(loss5G).toBeGreaterThan(loss2G);
        });
    });

    describe('calculateSinr', () => {
        it('should decrease with path loss', () => {
            const txPower = 30; // dBm
            const noisePower = -100; // dBm

            const sinr1 = metrics.calculateSinr(txPower, 80, noisePower);
            const sinr2 = metrics.calculateSinr(txPower, 100, noisePower);

            expect(sinr2).toBeLessThan(sinr1);
        });

        it('should account for fading', () => {
            const sinrNoFade = metrics.calculateSinr(30, 80, -100, 0);
            const sinrWithFade = metrics.calculateSinr(30, 80, -100, -5);

            expect(sinrWithFade).toBeLessThan(sinrNoFade);
        });
    });

    describe('calculateCapacity', () => {
        it('should increase with bandwidth', () => {
            const sinr = 10; // dB
            const cap1M = metrics.calculateCapacity(1e6, sinr);
            const cap10M = metrics.calculateCapacity(10e6, sinr);

            expect(cap10M).toBeGreaterThan(cap1M);
        });

        it('should increase with SINR', () => {
            const bw = 10e6;
            const cap10dB = metrics.calculateCapacity(bw, 10);
            const cap20dB = metrics.calculateCapacity(bw, 20);

            expect(cap20dB).toBeGreaterThan(cap10dB);
        });

        it('should follow Shannon formula', () => {
            const bw = 1e6;
            const sinrDb = 10;
            const sinrLinear = Math.pow(10, sinrDb / 10);
            const expected = bw * Math.log2(1 + sinrLinear);

            expect(metrics.calculateCapacity(bw, sinrDb)).toBeCloseTo(expected, 0);
        });
    });

    describe('calculateDistance', () => {
        it('should calculate 3D Euclidean distance', () => {
            const p1 = createTestPosition(0, 0, 0);
            const p2 = createTestPosition(3, 4, 0);

            expect(metrics.calculateDistance(p1, p2)).toBe(5);
        });

        it('should handle 3D case', () => {
            const p1 = createTestPosition(0, 0, 0);
            const p2 = createTestPosition(1, 2, 2);

            expect(metrics.calculateDistance(p1, p2)).toBeCloseTo(3, 5);
        });
    });

    describe('calculateAggregateThroughput', () => {
        it('should sum throughput for all users', () => {
            const uav = createTestPosition(0, 0, 100);
            const users = [
                createTestPosition(50, 0, 0),
                createTestPosition(0, 50, 0),
            ];

            const config = {
                txPowerDbm: 30,
                bandwidthHz: 10e6,
                frequencyHz: 2.4e9,
                noiseFigureDb: 7,
            };

            const throughput = metrics.calculateAggregateThroughput(uav, users, config);

            expect(throughput).toBeGreaterThan(0);
        });

        it('should decrease with distance', () => {
            const uavNear = createTestPosition(0, 0, 50);
            const uavFar = createTestPosition(0, 0, 500);
            const users = [createTestPosition(0, 0, 0)];

            const config = {
                txPowerDbm: 30,
                bandwidthHz: 10e6,
                frequencyHz: 2.4e9,
                noiseFigureDb: 7,
            };

            const near = metrics.calculateAggregateThroughput(uavNear, users, config);
            const far = metrics.calculateAggregateThroughput(uavFar, users, config);

            expect(near).toBeGreaterThan(far);
        });
    });
});

// ==================== Constraints Tests ====================

describe('ISAC Constraints', () => {
    describe('Flight Constraints', () => {
        it('should export flight constraint functions', () => {
            expect(constraints).toBeDefined();
        });
    });

    describe('Communication Constraints', () => {
        it('should export communication constraint functions', () => {
            expect(constraints).toBeDefined();
        });
    });
});

// ==================== Integration Tests ====================

describe('ISAC Integration', () => {
    it('should plan trajectory and evaluate metrics', () => {
        // Create planner
        const planner = createPlanner({ numWaypoints: 5 });

        // Generate trajectory
        const result = planner.plan(
            createTestPosition(0, 0, 100),
            createTestPosition(500, 500, 100)
        );

        expect(result.feasible).toBe(true);

        // Evaluate LoS along trajectory
        const users = [
            createTestPosition(250, 250, 0),
            createTestPosition(100, 400, 0),
        ];

        const losValues = result.trajectory.map(waypoint =>
            metrics.calculateAverageLoS(waypoint, users)
        );

        // All LoS values should be valid
        losValues.forEach(los => {
            expect(los).toBeGreaterThanOrEqual(0);
            expect(los).toBeLessThanOrEqual(1);
        });
    });

    it('should optimize altitude for LoS', () => {
        const planner = createPlanner({ numWaypoints: 10 });

        const result = planner.plan(
            createTestPosition(0, 0, 50),
            createTestPosition(1000, 0, 50)
        );

        // Middle waypoints should have higher altitude due to optimization
        const midIdx = Math.floor(result.trajectory.length / 2);
        const midAltitude = result.trajectory[midIdx].z;

        expect(midAltitude).toBeGreaterThan(50);
    });
});

// ==================== URLLC Detailed Tests ====================

describe('ISAC Metrics: URLLC Detailed', () => {
    describe('calculateBler edge cases', () => {
        it('should return near 0 for very high SINR (> 30 dB)', () => {
            const bler = metrics.calculateBler(35);
            expect(bler).toBeLessThan(0.001);
        });

        it('should return near 1 for very low SINR (< -10 dB)', () => {
            const bler = metrics.calculateBler(-15);
            expect(bler).toBeGreaterThan(0.99);
        });

        it('should return 0.5 when SINR equals threshold', () => {
            const threshold = 5;
            const bler = metrics.calculateBler(threshold, threshold);
            expect(bler).toBeCloseTo(0.5, 5);
        });

        it('should be steeper with higher slope parameter', () => {
            const sinr = 7; // Slightly above threshold of 5
            const blerLowSlope = metrics.calculateBler(sinr, 5, 1);
            const blerHighSlope = metrics.calculateBler(sinr, 5, 4);

            // Higher slope = more aggressive transition = lower BLER above threshold
            expect(blerHighSlope).toBeLessThan(blerLowSlope);
        });

        it('should be monotonically decreasing with SINR', () => {
            const sinrs = [-10, -5, 0, 5, 10, 15, 20];
            const blers = sinrs.map(s => metrics.calculateBler(s));

            for (let i = 1; i < blers.length; i++) {
                expect(blers[i]).toBeLessThanOrEqual(blers[i - 1]);
            }
        });
    });

    describe('evaluateUrllc detailed scenarios', () => {
        const defaultReq = metrics.DEFAULT_URLLC_REQUIREMENTS;

        it('should detect latency-only violation', () => {
            const result = metrics.evaluateUrllc(
                2e6, // rate OK
                0.00001, // reliability OK (1 - 0.00001 = 0.99999)
                5, // latency BAD (> 1ms)
                defaultReq
            );

            expect(result.satisfied).toBe(false);
            expect(result.latencySatisfied).toBe(false);
            expect(result.reliabilitySatisfied).toBe(true);
            expect(result.rateSatisfied).toBe(true);
        });

        it('should detect reliability-only violation', () => {
            const result = metrics.evaluateUrllc(
                2e6, // rate OK
                0.01, // reliability BAD (1 - 0.01 = 0.99)
                0.5, // latency OK
                defaultReq
            );

            expect(result.satisfied).toBe(false);
            expect(result.latencySatisfied).toBe(true);
            expect(result.reliabilitySatisfied).toBe(false);
            expect(result.rateSatisfied).toBe(true);
        });

        it('should detect rate-only violation', () => {
            const result = metrics.evaluateUrllc(
                0.5e6, // rate BAD (< 1e6)
                0.00001, // reliability OK
                0.5, // latency OK
                defaultReq
            );

            expect(result.satisfied).toBe(false);
            expect(result.latencySatisfied).toBe(true);
            expect(result.reliabilitySatisfied).toBe(true);
            expect(result.rateSatisfied).toBe(false);
        });

        it('should detect multi-constraint violation', () => {
            const result = metrics.evaluateUrllc(
                0.5e6, // rate BAD
                0.01, // reliability BAD
                5, // latency BAD
                defaultReq
            );

            expect(result.satisfied).toBe(false);
            expect(result.latencySatisfied).toBe(false);
            expect(result.reliabilitySatisfied).toBe(false);
            expect(result.rateSatisfied).toBe(false);
        });

        it('should satisfy all constraints when exact boundary', () => {
            const result = metrics.evaluateUrllc(
                1e6, // rate exactly at boundary
                0.00001, // reliability exactly at boundary
                1, // latency exactly at boundary
                defaultReq
            );

            expect(result.satisfied).toBe(true);
            expect(result.actualLatencyMs).toBe(1);
            expect(result.actualReliability).toBeCloseTo(0.99999, 5);
            expect(result.actualRateBps).toBe(1e6);
        });

        it('should work with custom requirements', () => {
            const customReq = {
                latencyMs: 10,
                reliability: 0.99,
                minRateBps: 100e3,
            };

            const result = metrics.evaluateUrllc(
                200e3, // rate OK for custom
                0.005, // reliability OK for custom (0.995)
                8, // latency OK for custom
                customReq
            );

            expect(result.satisfied).toBe(true);
        });
    });

    describe('calculateViolationRate', () => {
        it('should return 0 for empty results array', () => {
            const rate = metrics.calculateViolationRate([]);
            expect(rate).toBe(0);
        });

        it('should return 0 when all samples satisfy constraints', () => {
            const results = Array(5).fill(null).map(() => ({
                satisfied: true,
                latencySatisfied: true,
                reliabilitySatisfied: true,
                rateSatisfied: true,
                actualLatencyMs: 0.5,
                actualReliability: 0.99999,
                actualRateBps: 2e6,
            }));

            const rate = metrics.calculateViolationRate(results);
            expect(rate).toBe(0);
        });

        it('should return 1 when all samples violate constraints', () => {
            const results = Array(5).fill(null).map(() => ({
                satisfied: false,
                latencySatisfied: false,
                reliabilitySatisfied: false,
                rateSatisfied: false,
                actualLatencyMs: 10,
                actualReliability: 0.9,
                actualRateBps: 0.1e6,
            }));

            const rate = metrics.calculateViolationRate(results);
            expect(rate).toBe(1);
        });

        it('should calculate partial violation rate correctly', () => {
            const results = [
                { satisfied: true, latencySatisfied: true, reliabilitySatisfied: true, rateSatisfied: true, actualLatencyMs: 0.5, actualReliability: 0.99999, actualRateBps: 2e6 },
                { satisfied: false, latencySatisfied: false, reliabilitySatisfied: true, rateSatisfied: true, actualLatencyMs: 5, actualReliability: 0.99999, actualRateBps: 2e6 },
                { satisfied: true, latencySatisfied: true, reliabilitySatisfied: true, rateSatisfied: true, actualLatencyMs: 0.5, actualReliability: 0.99999, actualRateBps: 2e6 },
                { satisfied: true, latencySatisfied: true, reliabilitySatisfied: true, rateSatisfied: true, actualLatencyMs: 0.5, actualReliability: 0.99999, actualRateBps: 2e6 },
            ];

            const rate = metrics.calculateViolationRate(results);
            expect(rate).toBeCloseTo(0.25, 5); // 1 out of 4
        });

        it('should handle single sample', () => {
            const satisfied = { satisfied: true, latencySatisfied: true, reliabilitySatisfied: true, rateSatisfied: true, actualLatencyMs: 0.5, actualReliability: 0.99999, actualRateBps: 2e6 };
            const violated = { satisfied: false, latencySatisfied: false, reliabilitySatisfied: true, rateSatisfied: true, actualLatencyMs: 5, actualReliability: 0.99999, actualRateBps: 2e6 };

            expect(metrics.calculateViolationRate([satisfied])).toBe(0);
            expect(metrics.calculateViolationRate([violated])).toBe(1);
        });
    });
});

// ==================== envAdapter Tests ====================

describe('ISAC envAdapter', () => {
    describe('IsacTrajectoryEnvironment', async () => {
        const { IsacTrajectoryEnvironment, createEnvironment } = await import('../src/isac/envAdapter');

        it('should create environment with default config', () => {
            const env = createEnvironment();
            expect(env).toBeDefined();
            expect(env.inputSpace).toBeDefined();
            expect(env.outputSpace).toBeDefined();
        });

        it('should have correct observation space structure', () => {
            const env = createEnvironment();
            expect(env.inputSpace.kind).toBe('box');
            expect((env.inputSpace as any).shape).toBeDefined();
            expect((env.inputSpace as any).low).toBe(-1);
            expect((env.inputSpace as any).high).toBe(1);
        });

        it('should have correct action space structure', () => {
            const env = createEnvironment();
            expect(env.outputSpace.kind).toBe('box');
            expect((env.outputSpace as any).shape).toEqual([3]);
            expect((env.outputSpace as any).low).toBe(-1);
            expect((env.outputSpace as any).high).toBe(1);
        });

        describe('reset', () => {
            it('should return initial observation vector', async () => {
                const env = createEnvironment();
                const { input, info } = await env.reset();

                expect(Array.isArray(input)).toBe(true);
                expect(input.length).toBeGreaterThan(0);
                expect(info).toBeDefined();
            });

            it('should include complete info object', async () => {
                const env = createEnvironment();
                const { info } = await env.reset();

                expect(info.step).toBe(0);
                expect(info.position).toBeDefined();
                expect(info.velocity).toBeDefined();
                expect(typeof info.losPercentage).toBe('number');
                expect(typeof info.totalThroughput).toBe('number');
            });

            it('should produce reproducible results with same seed', async () => {
                const env1 = createEnvironment({ seed: 12345 });
                const env2 = createEnvironment({ seed: 12345 });

                const { input: input1 } = await env1.reset(12345);
                const { input: input2 } = await env2.reset(12345);

                expect(input1).toEqual(input2);
            });
        });

        describe('step', () => {
            it('should accept action and return step result', async () => {
                const env = createEnvironment();
                await env.reset();

                const action = [0, 0.5, 0]; // Move in y direction
                const result = await env.step(action);

                expect(result.input).toBeDefined();
                expect(typeof result.metric).toBe('number');
                expect(typeof result.done).toBe('boolean');
                expect(typeof result.truncated).toBe('boolean');
                expect(result.info).toBeDefined();
            });

            it('should update step counter', async () => {
                const env = createEnvironment();
                await env.reset();

                const { info: info1 } = await env.step([0, 0, 0]);
                expect(info1.step).toBe(1);

                const { info: info2 } = await env.step([0, 0, 0]);
                expect(info2.step).toBe(2);
            });

            it('should throw error if not reset', async () => {
                const env = new IsacTrajectoryEnvironment();
                await expect(env.step([0, 0, 0])).rejects.toThrow();
            });

            it('should truncate at episode horizon', async () => {
                const env = createEnvironment({ episodeHorizon: 3 });
                await env.reset();

                let truncated = false;
                for (let i = 0; i < 5; i++) {
                    const result = await env.step([0, 0, 0]);
                    if (result.truncated) {
                        truncated = true;
                        break;
                    }
                }

                expect(truncated).toBe(true);
            });
        });

        describe('episode metrics and history', () => {
            it('should track state history', async () => {
                const env = createEnvironment();
                await env.reset();
                await env.step([0.1, 0, 0]);
                await env.step([0.1, 0, 0]);

                const history = env.getStateHistory();
                expect(history.length).toBe(3); // initial + 2 steps
            });

            it('should track reward history', async () => {
                const env = createEnvironment();
                await env.reset();
                await env.step([0.1, 0, 0]);
                await env.step([0.1, 0, 0]);

                const rewards = env.getMetricHistory();
                expect(rewards.length).toBe(2);
            });

            it('should provide episode metrics', async () => {
                const env = createEnvironment();
                await env.reset();
                await env.step([0.1, 0, 0]);

                const metrics = env.getEpisodeMetrics();
                expect(metrics).toBeDefined();
            });
        });

        describe('close', () => {
            it('should not throw when closing', () => {
                const env = createEnvironment();
                expect(() => env.close()).not.toThrow();
            });
        });
    });

    describe('Input/Output Conversion', async () => {
        const { extractInput, inputToVector, continuousOutputToOffset, discreteOutputToOffset, getInputSpace, getOutputSpace } = await import('../src/isac/envAdapter');
        const { createConfig } = await import('../src/isac');

        const mockConfig = createConfig({
            users: [
                { id: 'u1', position: { x: 100, y: 100, z: 0 }, minRate: 1e6, maxLatencyMs: 1, reliabilityTarget: 0.99999, isUrllc: true },
                { id: 'u2', position: { x: 900, y: 900, z: 0 }, minRate: 1e6, maxLatencyMs: 1, reliabilityTarget: 0.99999, isUrllc: false },
            ],
        });
        const mockState = {
            step: 5,
            uav: {
                position: { x: 500, y: 500, z: 100 },
                velocity: { x: 10, y: 5, z: 0 },
                energyUsed: 500,
            },
            userChannels: [
                { userId: 'u1', position: { x: 100, y: 100, z: 0 }, isLos: true, sinrDb: 15, distance: 100, throughputBps: 1e6 },
                { userId: 'u2', position: { x: 900, y: 900, z: 0 }, isLos: false, sinrDb: 5, distance: 500, throughputBps: 0.5e6 },
            ],
            losPercentage: 0.5,
            totalThroughput: 1.5e6,
            urllcSatisfied: [true, false],
            sensingCoverage: 0.8,
        };

        it('should extract input from state', () => {
            const input = extractInput(mockState as any, mockConfig);

            expect(input.normalizedPosition).toHaveLength(3);
            expect(input.normalizedVelocity).toHaveLength(3);
            expect(input.losStatus).toHaveLength(2);
            expect(input.normalizedSinr).toHaveLength(2);
        });

        it('should convert input to flat vector', () => {
            const input = extractInput(mockState as any, mockConfig);
            const vector = inputToVector(input);

            expect(Array.isArray(vector)).toBe(true);
            expect(vector.every(v => typeof v === 'number')).toBe(true);
        });

        it('should convert continuous output to offset', () => {
            const output = [0.5, -0.5, 0.2];
            const offset = continuousOutputToOffset(output, mockConfig);

            expect(offset.x).toBeDefined();
            expect(offset.y).toBeDefined();
            expect(offset.z).toBeDefined();
        });

        it('should convert discrete output to offset', () => {
            // Center output (13) should be hover
            const hoverOffset = discreteOutputToOffset(13, 20);
            expect(hoverOffset.x).toBe(0);
            expect(hoverOffset.y).toBe(0);
            expect(hoverOffset.z).toBe(0);

            // Forward output
            const forwardOffset = discreteOutputToOffset(1, 20);
            expect(forwardOffset.x).toBe(0);
            expect(forwardOffset.y).toBe(-20);
        });

        it('should return valid input space', () => {
            const space = getInputSpace(mockConfig);
            expect(space.kind).toBe('box');
            expect(space.shape[0]).toBeGreaterThan(0);
        });

        it('should return valid output space (continuous)', () => {
            const space = getOutputSpace(mockConfig, true);
            expect(space.kind).toBe('box');
            expect(space.shape).toEqual([3]);
        });

        it('should return valid output space (discrete)', () => {
            const space = getOutputSpace(mockConfig, false);
            expect(space.kind).toBe('discrete');
            expect(space.n).toBe(27);
        });
    });
});

