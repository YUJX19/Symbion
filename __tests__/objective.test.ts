/**
 * Core Objective Module Tests
 * Tests for MetricSpec, metric transformations, composition, and MetricRegistry
 */

import { describe, it, expect, beforeEach } from 'vitest';
import {
    // Factory functions
    asMetricSpec,
    simpleMetric,
    // Metric transformations
    normalizeValue,
    normalizeMetric,
    clampMetric,
    createSmoothMetric,
    // Composition
    combineMetricsWithTracking,
    combineMetrics,
    // Registry
    MetricRegistry,
    createMetricRegistry,
    globalMetricRegistry,
    // Types
    type MetricSpec,
    type MetricMetadata,
    type OptimizationDirection,
} from '../src/core';

// ==================== Test State Type ====================

interface TestState {
    reward: number;
    energy: number;
    distance: number;
    speed: number;
    throughput: number;
    latency: number;
}

// ==================== asMetricSpec ====================

describe('asMetricSpec', () => {
    it('should create metric spec with evaluation function', () => {
        const metric = asMetricSpec<TestState>(
            (state) => state.reward,
            {
                id: 'reward',
                description: 'Cumulative reward',
                direction: 'maximize',
            }
        );

        expect(metric.metadata.id).toBe('reward');
        expect(metric.evaluate({ reward: 10, energy: 0, distance: 0, speed: 0, throughput: 0, latency: 0 })).toBe(10);
    });

    it('should set default weight to 1.0', () => {
        const metric = asMetricSpec<TestState>(
            (state) => state.reward,
            {
                id: 'reward',
                description: 'Reward',
                direction: 'maximize',
            }
        );

        expect(metric.metadata.weight).toBe(1.0);
    });

    it('should preserve custom weight', () => {
        const metric = asMetricSpec<TestState>(
            (state) => state.reward,
            {
                id: 'reward',
                description: 'Reward',
                direction: 'maximize',
                weight: 0.5,
            }
        );

        expect(metric.metadata.weight).toBe(0.5);
    });

    it('should set default version', () => {
        const metric = asMetricSpec<TestState>(
            (state) => state.reward,
            {
                id: 'reward',
                description: 'Reward',
                direction: 'maximize',
            }
        );

        expect(metric.metadata.version).toBe('1.0');
    });

    it('should support all metadata fields', () => {
        const metric = asMetricSpec<TestState>(
            (state) => state.throughput,
            {
                id: 'throughput',
                description: 'Network throughput',
                direction: 'maximize',
                unit: 'bps',
                category: 'communication',
                weight: 2.0,
                version: '2.0',
            }
        );

        expect(metric.metadata.unit).toBe('bps');
        expect(metric.metadata.category).toBe('communication');
        expect(metric.metadata.weight).toBe(2.0);
        expect(metric.metadata.version).toBe('2.0');
    });
});

// ==================== simpleMetric ====================

describe('simpleMetric', () => {
    it('should create simple maximize metric', () => {
        const metric = simpleMetric<TestState>('reward', (s) => s.reward, 'maximize');

        expect(metric.metadata.id).toBe('reward');
        expect(metric.metadata.direction).toBe('maximize');
        expect(metric.evaluate({ reward: 100, energy: 0, distance: 0, speed: 0, throughput: 0, latency: 0 })).toBe(100);
    });

    it('should create simple minimize metric', () => {
        const metric = simpleMetric<TestState>('energy', (s) => s.energy, 'minimize');

        expect(metric.metadata.id).toBe('energy');
        expect(metric.metadata.direction).toBe('minimize');
    });

    it('should default to maximize direction', () => {
        const metric = simpleMetric<TestState>('reward', (s) => s.reward);
        expect(metric.metadata.direction).toBe('maximize');
    });

    it('should set description to id', () => {
        const metric = simpleMetric<TestState>('my_metric', (s) => s.speed);
        expect(metric.metadata.description).toBe('my_metric');
    });
});

// ==================== normalizeValue ====================

describe('normalizeValue', () => {
    describe('maximize direction', () => {
        it('should normalize to 0 at min', () => {
            expect(normalizeValue(0, 0, 100, 'maximize')).toBe(0);
        });

        it('should normalize to 1 at max', () => {
            expect(normalizeValue(100, 0, 100, 'maximize')).toBe(1);
        });

        it('should normalize to 0.5 at midpoint', () => {
            expect(normalizeValue(50, 0, 100, 'maximize')).toBe(0.5);
        });

        it('should clamp values below min', () => {
            expect(normalizeValue(-10, 0, 100, 'maximize')).toBe(0);
        });

        it('should clamp values above max', () => {
            expect(normalizeValue(150, 0, 100, 'maximize')).toBe(1);
        });
    });

    describe('minimize direction', () => {
        it('should invert normalization (1 at min, 0 at max)', () => {
            expect(normalizeValue(0, 0, 100, 'minimize')).toBe(1);
            expect(normalizeValue(100, 0, 100, 'minimize')).toBe(0);
            expect(normalizeValue(50, 0, 100, 'minimize')).toBe(0.5);
        });
    });

    describe('edge cases', () => {
        it('should return 0.5 when min equals max', () => {
            expect(normalizeValue(50, 50, 50, 'maximize')).toBe(0.5);
        });

        it('should handle negative ranges', () => {
            expect(normalizeValue(-50, -100, 0, 'maximize')).toBe(0.5);
        });
    });
});

// ==================== normalizeMetric ====================

describe('normalizeMetric', () => {
    it('should create normalized version of metric', () => {
        const raw = simpleMetric<TestState>('sinr', (s) => s.reward, 'maximize');
        const normalized = normalizeMetric(raw, -20, 40);

        expect(normalized.metadata.id).toBe('sinr_normalized');
        expect(normalized.metadata.direction).toBe('maximize');
    });

    it('should evaluate to normalized values', () => {
        const raw = simpleMetric<TestState>('value', (s) => s.reward, 'maximize');
        const normalized = normalizeMetric(raw, 0, 100);

        const state = { reward: 50, energy: 0, distance: 0, speed: 0, throughput: 0, latency: 0 };
        expect(normalized.evaluate(state)).toBe(0.5);
    });

    it('should handle minimize metrics', () => {
        const raw = simpleMetric<TestState>('energy', (s) => s.energy, 'minimize');
        const normalized = normalizeMetric(raw, 0, 100);

        // At 0 energy (best), should be 1 after normalization
        expect(normalized.evaluate({ reward: 0, energy: 0, distance: 0, speed: 0, throughput: 0, latency: 0 })).toBe(1);
    });
});

// ==================== clampMetric ====================

describe('clampMetric', () => {
    it('should create clamped version of metric', () => {
        const raw = simpleMetric<TestState>('speed', (s) => s.speed, 'maximize');
        const clamped = clampMetric(raw, 0, 20);

        expect(clamped.metadata.id).toBe('speed_clamped');
    });

    it('should clamp values to range', () => {
        const raw = simpleMetric<TestState>('speed', (s) => s.speed, 'maximize');
        const clamped = clampMetric(raw, 0, 20);

        expect(clamped.evaluate({ reward: 0, energy: 0, distance: 0, speed: 25, throughput: 0, latency: 0 })).toBe(20);
        expect(clamped.evaluate({ reward: 0, energy: 0, distance: 0, speed: -5, throughput: 0, latency: 0 })).toBe(0);
        expect(clamped.evaluate({ reward: 0, energy: 0, distance: 0, speed: 15, throughput: 0, latency: 0 })).toBe(15);
    });
});

// ==================== createSmoothMetric ====================

describe('createSmoothMetric', () => {
    it('should create smoothed version of metric', () => {
        const raw = simpleMetric<TestState>('reward', (s) => s.reward, 'maximize');
        const { metric: smoothed, reset } = createSmoothMetric(raw, 0.9);

        expect(smoothed.metadata.id).toBe('reward_smooth');
    });

    it('should return first value unchanged', () => {
        const raw = simpleMetric<TestState>('reward', (s) => s.reward, 'maximize');
        const { metric: smoothed, reset } = createSmoothMetric(raw, 0.9);

        const firstValue = smoothed.evaluate({ reward: 10, energy: 0, distance: 0, speed: 0, throughput: 0, latency: 0 });
        expect(firstValue).toBe(10);
    });

    it('should apply exponential smoothing', () => {
        const raw = simpleMetric<TestState>('reward', (s) => s.reward, 'maximize');
        const { metric: smoothed, reset } = createSmoothMetric(raw, 0.9);

        smoothed.evaluate({ reward: 10, energy: 0, distance: 0, speed: 0, throughput: 0, latency: 0 }); // 10
        const second = smoothed.evaluate({ reward: 20, energy: 0, distance: 0, speed: 0, throughput: 0, latency: 0 }); // 0.9 * 10 + 0.1 * 20 = 11

        expect(second).toBeCloseTo(11, 5);
    });

    it('should reset smoothed state', () => {
        const raw = simpleMetric<TestState>('reward', (s) => s.reward, 'maximize');
        const { metric: smoothed, reset } = createSmoothMetric(raw, 0.9);

        smoothed.evaluate({ reward: 10, energy: 0, distance: 0, speed: 0, throughput: 0, latency: 0 });
        smoothed.evaluate({ reward: 20, energy: 0, distance: 0, speed: 0, throughput: 0, latency: 0 });

        reset();

        const afterReset = smoothed.evaluate({ reward: 100, energy: 0, distance: 0, speed: 0, throughput: 0, latency: 0 });
        expect(afterReset).toBe(100); // Should be raw value after reset
    });
});

// ==================== combineMetricsWithTracking ====================

describe('combineMetricsWithTracking', () => {
    const createTestMetrics = () => [
        simpleMetric<TestState>('throughput', (s) => s.throughput, 'maximize'),
        simpleMetric<TestState>('energy', (s) => s.energy, 'minimize'),
        simpleMetric<TestState>('latency', (s) => s.latency, 'minimize'),
    ];

    describe('weighted_sum method', () => {
        it('should combine metrics with weighted sum', () => {
            const metrics = createTestMetrics();
            const combined = combineMetricsWithTracking(metrics, 'weighted_sum');

            const result = combined.evaluate({
                reward: 0,
                energy: 50,
                distance: 0,
                speed: 0,
                throughput: 100,
                latency: 10,
            });

            // throughput: +100, energy: -50, latency: -10
            expect(result.total).toBe(100 - 50 - 10);
        });

        it('should provide per-metric breakdown', () => {
            const metrics = createTestMetrics();
            const combined = combineMetricsWithTracking(metrics, 'weighted_sum');

            const result = combined.evaluate({
                reward: 0,
                energy: 50,
                distance: 0,
                speed: 0,
                throughput: 100,
                latency: 10,
            });

            expect(result.breakdown.throughput.raw).toBe(100);
            expect(result.breakdown.throughput.weighted).toBe(100);
            expect(result.breakdown.throughput.direction).toBe('maximize');

            expect(result.breakdown.energy.raw).toBe(50);
            expect(result.breakdown.energy.weighted).toBe(-50);
            expect(result.breakdown.energy.direction).toBe('minimize');
        });

        it('should include timestamp', () => {
            const metrics = createTestMetrics();
            const combined = combineMetricsWithTracking(metrics, 'weighted_sum');
            const before = Date.now();
            const result = combined.evaluate({
                reward: 0,
                energy: 0,
                distance: 0,
                speed: 0,
                throughput: 0,
                latency: 0,
            });
            const after = Date.now();

            expect(result.timestamp).toBeGreaterThanOrEqual(before);
            expect(result.timestamp).toBeLessThanOrEqual(after);
        });
    });

    describe('product method', () => {
        it('should combine metrics with product', () => {
            const metrics = [
                simpleMetric<TestState>('a', (s) => s.reward, 'maximize'),
                simpleMetric<TestState>('b', (s) => s.energy, 'maximize'),
            ];
            const combined = combineMetricsWithTracking(metrics, 'product');

            const result = combined.evaluate({
                reward: 2,
                energy: 3,
                distance: 0,
                speed: 0,
                throughput: 0,
                latency: 0,
            });

            expect(result.total).toBe(6);
        });
    });

    describe('min method', () => {
        it('should return minimum weighted value', () => {
            const metrics = [
                simpleMetric<TestState>('a', (s) => s.reward, 'maximize'),
                simpleMetric<TestState>('b', (s) => s.energy, 'maximize'),
            ];
            const combined = combineMetricsWithTracking(metrics, 'min');

            const result = combined.evaluate({
                reward: 10,
                energy: 5,
                distance: 0,
                speed: 0,
                throughput: 0,
                latency: 0,
            });

            expect(result.total).toBe(5);
        });
    });

    describe('max method', () => {
        it('should return maximum weighted value', () => {
            const metrics = [
                simpleMetric<TestState>('a', (s) => s.reward, 'maximize'),
                simpleMetric<TestState>('b', (s) => s.energy, 'maximize'),
            ];
            const combined = combineMetricsWithTracking(metrics, 'max');

            const result = combined.evaluate({
                reward: 10,
                energy: 5,
                distance: 0,
                speed: 0,
                throughput: 0,
                latency: 0,
            });

            expect(result.total).toBe(10);
        });
    });

    describe('getMetricIds', () => {
        it('should return list of metric IDs', () => {
            const metrics = createTestMetrics();
            const combined = combineMetricsWithTracking(metrics, 'weighted_sum');

            expect(combined.getMetricIds()).toEqual(['throughput', 'energy', 'latency']);
        });
    });

    describe('custom weights', () => {
        it('should respect metric weights', () => {
            const metrics = [
                asMetricSpec<TestState>(
                    (s) => s.throughput,
                    { id: 'throughput', description: 'Throughput', direction: 'maximize', weight: 2.0 }
                ),
                asMetricSpec<TestState>(
                    (s) => s.energy,
                    { id: 'energy', description: 'Energy', direction: 'minimize', weight: 0.5 }
                ),
            ];
            const combined = combineMetricsWithTracking(metrics, 'weighted_sum');

            const result = combined.evaluate({
                reward: 0,
                energy: 100,
                distance: 0,
                speed: 0,
                throughput: 100,
                latency: 0,
            });

            // throughput: 100 * 2.0 = 200, energy: -100 * 0.5 = -50
            expect(result.total).toBe(150);
            expect(result.breakdown.throughput.weighted).toBe(200);
            expect(result.breakdown.energy.weighted).toBe(-50);
        });
    });
});

// ==================== combineMetrics ====================

describe('combineMetrics', () => {
    it('should return combined value directly', () => {
        const metrics = [
            simpleMetric<TestState>('a', (s) => s.reward, 'maximize'),
            simpleMetric<TestState>('b', (s) => s.energy, 'maximize'),
        ];
        const combined = combineMetrics(metrics, 'weighted_sum');

        const value = combined({
            reward: 10,
            energy: 20,
            distance: 0,
            speed: 0,
            throughput: 0,
            latency: 0,
        });

        expect(value).toBe(30);
    });
});

// ==================== MetricRegistry ====================

describe('MetricRegistry', () => {
    let registry: MetricRegistry<TestState>;

    beforeEach(() => {
        registry = new MetricRegistry<TestState>();
    });

    describe('register', () => {
        it('should register metrics', () => {
            const metric = simpleMetric<TestState>('reward', (s) => s.reward);
            registry.register(metric);

            expect(registry.has('reward')).toBe(true);
        });

        it('should allow chaining', () => {
            registry
                .register(simpleMetric<TestState>('a', (s) => s.reward))
                .register(simpleMetric<TestState>('b', (s) => s.energy));

            expect(registry.has('a')).toBe(true);
            expect(registry.has('b')).toBe(true);
        });
    });

    describe('get', () => {
        it('should return registered metric', () => {
            const metric = simpleMetric<TestState>('reward', (s) => s.reward);
            registry.register(metric);

            const retrieved = registry.get('reward');
            expect(retrieved).toBe(metric);
        });

        it('should return undefined for unknown metric', () => {
            expect(registry.get('unknown')).toBeUndefined();
        });
    });

    describe('has', () => {
        it('should return true for registered metric', () => {
            registry.register(simpleMetric<TestState>('test', (s) => s.reward));
            expect(registry.has('test')).toBe(true);
        });

        it('should return false for unregistered metric', () => {
            expect(registry.has('unknown')).toBe(false);
        });
    });

    describe('list', () => {
        it('should return all metric IDs', () => {
            registry
                .register(simpleMetric<TestState>('a', (s) => s.reward))
                .register(simpleMetric<TestState>('b', (s) => s.energy))
                .register(simpleMetric<TestState>('c', (s) => s.speed));

            const ids = registry.list();
            expect(ids).toContain('a');
            expect(ids).toContain('b');
            expect(ids).toContain('c');
            expect(ids.length).toBe(3);
        });

        it('should return empty array for empty registry', () => {
            expect(registry.list()).toEqual([]);
        });
    });

    describe('listByCategory', () => {
        it('should filter metrics by category', () => {
            registry
                .register(asMetricSpec<TestState>(
                    (s) => s.throughput,
                    { id: 'throughput', description: 'Throughput', direction: 'maximize', category: 'communication' }
                ))
                .register(asMetricSpec<TestState>(
                    (s) => s.energy,
                    { id: 'energy', description: 'Energy', direction: 'minimize', category: 'energy' }
                ))
                .register(asMetricSpec<TestState>(
                    (s) => s.latency,
                    { id: 'latency', description: 'Latency', direction: 'minimize', category: 'reliability' }
                ));

            const commMetrics = registry.listByCategory('communication');
            expect(commMetrics).toHaveLength(1);
            expect(commMetrics[0].metadata.id).toBe('throughput');
        });
    });

    describe('compose', () => {
        it('should compose registered metrics', () => {
            registry
                .register(simpleMetric<TestState>('a', (s) => s.reward, 'maximize'))
                .register(simpleMetric<TestState>('b', (s) => s.energy, 'maximize'));

            const composed = registry.compose(['a', 'b'], 'weighted_sum');
            const result = composed.evaluate({
                reward: 10,
                energy: 20,
                distance: 0,
                speed: 0,
                throughput: 0,
                latency: 0,
            });

            expect(result.total).toBe(30);
        });

        it('should throw for unknown metric', () => {
            registry.register(simpleMetric<TestState>('a', (s) => s.reward));

            expect(() => registry.compose(['a', 'unknown'])).toThrow('Metric not found: unknown');
        });
    });

    describe('toJSON', () => {
        it('should export registry to JSON', () => {
            registry
                .register(asMetricSpec<TestState>(
                    (s) => s.throughput,
                    { id: 'throughput', description: 'Throughput', direction: 'maximize' }
                ))
                .register(asMetricSpec<TestState>(
                    (s) => s.energy,
                    { id: 'energy', description: 'Energy', direction: 'minimize' }
                ));

            const json = registry.toJSON();

            expect(json.throughput.id).toBe('throughput');
            expect(json.throughput.direction).toBe('maximize');
            expect(json.energy.id).toBe('energy');
            expect(json.energy.direction).toBe('minimize');
        });
    });
});

// ==================== createMetricRegistry ====================

describe('createMetricRegistry', () => {
    it('should create new registry instance', () => {
        const registry = createMetricRegistry<TestState>();
        expect(registry).toBeInstanceOf(MetricRegistry);
    });
});

// ==================== globalMetricRegistry ====================

describe('globalMetricRegistry', () => {
    it('should be a MetricRegistry instance', () => {
        expect(globalMetricRegistry).toBeInstanceOf(MetricRegistry);
    });
});
