/**
 * Core Optimization Framework Tests
 * 
 * Tests for core optimization modules (constraint, objective, space).
 * Domain-specific constraints are tested in isac-constraints.test.ts.
 */

import { describe, it, expect } from 'vitest';
import {
    // Constraint exports
    leConstraint,
    geConstraint,
    eqConstraint,
    asConstraintSpec,
    evaluateConstraint,
    evaluateConstraints,
    constraintsToPenalty,
} from '../src/core/constraint';
import {
    // Objective exports
    asMetricSpec,
    simpleMetric,
    normalizeValue,
    normalizeMetric,
    clampMetric,
    combineMetricsWithTracking,
    combineMetrics,
} from '../src/core/objective';
import {
    // Space exports
    discrete,
    box,
    multiDiscrete,
    dict,
    tuple,
    sample,
    contains,
    getDimension,
    serialize,
} from '../src/core/space';
import { isClose, createMockRNG } from './test-utils';

// ==================== Constraint Tests ====================

describe('Constraint System', () => {
    // Test state interface
    interface TestState {
        position: { x: number; y: number };
        velocity: { vx: number; vy: number };
        speed: number;
    }

    const createTestState = (speed: number, x = 0, y = 0): TestState => ({
        position: { x, y },
        velocity: { vx: speed * 0.707, vy: speed * 0.707 },
        speed,
    });

    describe('leConstraint (less than or equal)', () => {
        it('should create constraint with correct metadata', () => {
            const constraint = leConstraint<TestState>(
                'max_speed',
                (s) => s.speed,
                10,
                'hard'
            );
            expect(constraint.metadata.id).toBe('max_speed');
            expect(constraint.metadata.operator).toBe('le');
            expect(constraint.metadata.threshold).toBe(10);
            expect(constraint.metadata.severity).toBe('hard');
        });

        it('should evaluate satisfied constraint correctly', () => {
            const constraint = leConstraint<TestState>(
                'max_speed',
                (s) => s.speed,
                10
            );
            const result = evaluateConstraint(constraint, createTestState(5));
            expect(result.satisfied).toBe(true);
            expect(result.violation).toBe(0);
            expect(result.penalty).toBe(0);
        });

        it('should evaluate violated constraint correctly', () => {
            const constraint = leConstraint<TestState>(
                'max_speed',
                (s) => s.speed,
                10
            );
            const result = evaluateConstraint(constraint, createTestState(15));
            expect(result.satisfied).toBe(false);
            expect(result.violation).toBe(5); // 15 - 10 = 5
            // Hard constraints have 0 penalty (they make solution infeasible instead)
            expect(result.penalty).toBe(0);
        });

        it('should handle boundary value', () => {
            const constraint = leConstraint<TestState>(
                'max_speed',
                (s) => s.speed,
                10
            );
            const result = evaluateConstraint(constraint, createTestState(10));
            expect(result.satisfied).toBe(true);
        });

        it('should respect soft constraint penalty weight', () => {
            const constraint = leConstraint<TestState>(
                'max_speed',
                (s) => s.speed,
                10,
                'soft',
                { penaltyWeight: 2.0 }
            );
            const result = evaluateConstraint(constraint, createTestState(15));
            expect(result.severity).toBe('soft');
            expect(result.penalty).toBe(10); // 5 * 2.0
        });
    });

    describe('geConstraint (greater than or equal)', () => {
        it('should evaluate minimum distance constraint', () => {
            const constraint = geConstraint<TestState>(
                'min_distance',
                (s) => Math.sqrt(s.position.x ** 2 + s.position.y ** 2),
                5
            );

            const nearState = createTestState(0, 2, 2); // distance = 2.83
            const farState = createTestState(0, 4, 4);  // distance = 5.66

            expect(evaluateConstraint(constraint, nearState).satisfied).toBe(false);
            expect(evaluateConstraint(constraint, farState).satisfied).toBe(true);
        });

        it('should calculate correct violation amount', () => {
            const constraint = geConstraint<TestState>(
                'min_speed',
                (s) => s.speed,
                5
            );
            const result = evaluateConstraint(constraint, createTestState(2));
            expect(result.violation).toBe(3); // 5 - 2 = 3
        });
    });

    describe('eqConstraint (equality)', () => {
        it('should satisfy within tolerance', () => {
            const constraint = eqConstraint<TestState>(
                'target_speed',
                (s) => s.speed,
                10,
                0.5
            );

            expect(evaluateConstraint(constraint, createTestState(10.2)).satisfied).toBe(true);
            expect(evaluateConstraint(constraint, createTestState(10.6)).satisfied).toBe(false);
        });

        it('should calculate symmetric violation', () => {
            const constraint = eqConstraint<TestState>(
                'target_speed',
                (s) => s.speed,
                10,
                0.1
            );

            const aboveResult = evaluateConstraint(constraint, createTestState(12));
            const belowResult = evaluateConstraint(constraint, createTestState(8));

            // Both should have same violation magnitude (within floating point)
            expect(isClose(aboveResult.violation, belowResult.violation, 0.01)).toBe(true);
        });
    });

    describe('evaluateConstraints', () => {
        it('should evaluate multiple constraints', () => {
            const constraints = [
                leConstraint<TestState>('max_speed', (s) => s.speed, 10),
                geConstraint<TestState>('min_speed', (s) => s.speed, 2),
            ];

            const report = evaluateConstraints(constraints, createTestState(5));
            expect(report.results.length).toBe(2);
            expect(report.feasible).toBe(true);
            expect(report.totalPenalty).toBe(0);
        });

        it('should detect infeasibility from hard constraint', () => {
            const constraints = [
                leConstraint<TestState>('max_speed', (s) => s.speed, 10, 'hard'),
            ];

            const report = evaluateConstraints(constraints, createTestState(15));
            expect(report.feasible).toBe(false);
            expect(report.violations.length).toBe(1);
        });

        it('should accumulate penalties from soft constraints', () => {
            const constraints = [
                leConstraint<TestState>('soft1', (s) => s.speed, 5, 'soft', { penaltyWeight: 1 }),
                leConstraint<TestState>('soft2', (s) => s.speed, 8, 'soft', { penaltyWeight: 2 }),
            ];

            const report = evaluateConstraints(constraints, createTestState(10));
            expect(report.feasible).toBe(true); // Soft constraints don't cause infeasibility
            expect(report.totalPenalty).toBeGreaterThan(0);
        });
    });

    describe('constraintsToPenalty', () => {
        it('should convert violations to optimization penalty', () => {
            const constraints = [
                leConstraint<TestState>('max_speed', (s) => s.speed, 10, 'soft'),
            ];

            const penalty = constraintsToPenalty(constraints, createTestState(5));
            expect(penalty).toBe(0);

            const violatedPenalty = constraintsToPenalty(constraints, createTestState(15));
            expect(violatedPenalty).toBeGreaterThan(0);
        });

        it('should apply large penalty for hard constraint violations', () => {
            const hardConstraint = leConstraint<TestState>('hard', (s) => s.speed, 10, 'hard');
            const softConstraint = leConstraint<TestState>('soft', (s) => s.speed, 10, 'soft');

            const hardPenalty = constraintsToPenalty([hardConstraint], createTestState(15));
            const softPenalty = constraintsToPenalty([softConstraint], createTestState(15));

            expect(hardPenalty).toBeGreaterThan(softPenalty);
        });
    });

    // Note: Domain-specific constraints (maxSpeedConstraint, minDistanceConstraint, etc.)
    // are now tested in isac-constraints.test.ts
});

// ==================== Objective/Metric Tests ====================

describe('Objective System', () => {
    interface OptState {
        x: number;
        y: number;
    }

    describe('simpleMetric', () => {
        it('should create metric with correct metadata', () => {
            const metric = simpleMetric<OptState>('distance', (s) => Math.sqrt(s.x ** 2 + s.y ** 2), 'minimize');
            expect(metric.metadata.id).toBe('distance');
            expect(metric.metadata.direction).toBe('minimize');
        });

        it('should evaluate correctly', () => {
            const metric = simpleMetric<OptState>('sum', (s) => s.x + s.y);
            expect(metric.evaluate({ x: 3, y: 4 })).toBe(7);
        });
    });

    describe('asMetricSpec', () => {
        it('should create full metric specification', () => {
            const metric = asMetricSpec<OptState>(
                (s) => s.x * s.y,
                {
                    id: 'product',
                    description: 'Product of coordinates',
                    direction: 'maximize',
                    unit: 'units²',
                    weight: 1.5,
                }
            );
            expect(metric.metadata.description).toBe('Product of coordinates');
            expect(metric.metadata.weight).toBe(1.5);
            expect(metric.evaluate({ x: 3, y: 4 })).toBe(12);
        });
    });

    describe('normalizeValue', () => {
        it('should normalize to [0, 1] for maximize', () => {
            expect(normalizeValue(50, 0, 100, 'maximize')).toBe(0.5);
            expect(normalizeValue(0, 0, 100, 'maximize')).toBe(0);
            expect(normalizeValue(100, 0, 100, 'maximize')).toBe(1);
        });

        it('should invert for minimize direction', () => {
            expect(normalizeValue(50, 0, 100, 'minimize')).toBe(0.5);
            expect(normalizeValue(0, 0, 100, 'minimize')).toBe(1); // Low value = good = 1
            expect(normalizeValue(100, 0, 100, 'minimize')).toBe(0);
        });

        it('should clamp out-of-range values', () => {
            expect(normalizeValue(-10, 0, 100, 'maximize')).toBe(0);
            expect(normalizeValue(150, 0, 100, 'maximize')).toBe(1);
        });
    });

    describe('normalizeMetric', () => {
        it('should create normalized version of metric', () => {
            const original = simpleMetric<OptState>('value', (s) => s.x, 'maximize');
            const normalized = normalizeMetric(original, 0, 10);

            expect(normalized.evaluate({ x: 5, y: 0 })).toBe(0.5);
            expect(normalized.evaluate({ x: 0, y: 0 })).toBe(0);
            expect(normalized.evaluate({ x: 10, y: 0 })).toBe(1);
        });
    });

    describe('clampMetric', () => {
        it('should clamp values to range', () => {
            const original = simpleMetric<OptState>('value', (s) => s.x);
            const clamped = clampMetric(original, 0, 10);

            expect(clamped.evaluate({ x: 5, y: 0 })).toBe(5);
            expect(clamped.evaluate({ x: -5, y: 0 })).toBe(0);
            expect(clamped.evaluate({ x: 15, y: 0 })).toBe(10);
        });
    });

    describe('combineMetrics', () => {
        it('should combine with weighted sum', () => {
            const metrics = [
                asMetricSpec<OptState>((s) => s.x, { id: 'm1', description: '', direction: 'maximize', weight: 1 }),
                asMetricSpec<OptState>((s) => s.y, { id: 'm2', description: '', direction: 'maximize', weight: 2 }),
            ];

            const combined = combineMetrics(metrics, 'weighted_sum');
            // Default weights: 1, 2; values: 3, 4 => 1*3 + 2*4 = 11
            expect(combined({ x: 3, y: 4 })).toBe(11);
        });

        it('should combine with product', () => {
            const metrics = [
                simpleMetric<OptState>('m1', (s) => s.x),
                simpleMetric<OptState>('m2', (s) => s.y),
            ];

            const combined = combineMetrics(metrics, 'product');
            expect(combined({ x: 3, y: 4 })).toBe(12);
        });

        it('should combine with min', () => {
            const metrics = [
                simpleMetric<OptState>('m1', (s) => s.x),
                simpleMetric<OptState>('m2', (s) => s.y),
            ];

            const combined = combineMetrics(metrics, 'min');
            expect(combined({ x: 3, y: 4 })).toBe(3);
        });

        it('should combine with max', () => {
            const metrics = [
                simpleMetric<OptState>('m1', (s) => s.x),
                simpleMetric<OptState>('m2', (s) => s.y),
            ];

            const combined = combineMetrics(metrics, 'max');
            expect(combined({ x: 3, y: 4 })).toBe(4);
        });
    });

    describe('combineMetricsWithTracking', () => {
        it('should provide detailed breakdown', () => {
            const metrics = [
                asMetricSpec<OptState>((s) => s.x, { id: 'x_val', description: '', direction: 'maximize', weight: 1 }),
                asMetricSpec<OptState>((s) => s.y, { id: 'y_val', description: '', direction: 'maximize', weight: 2 }),
            ];

            const { evaluate, getMetricIds } = combineMetricsWithTracking(metrics);
            const result = evaluate({ x: 3, y: 4 });

            expect(getMetricIds()).toEqual(['x_val', 'y_val']);
            expect(result.total).toBe(11);
            expect(result.breakdown['x_val'].raw).toBe(3);
            expect(result.breakdown['y_val'].raw).toBe(4);
            expect(result.breakdown['x_val'].weighted).toBe(3);
            expect(result.breakdown['y_val'].weighted).toBe(8);
        });

        it('should include timestamp', () => {
            const metrics = [simpleMetric<OptState>('m', (s) => s.x)];
            const { evaluate } = combineMetricsWithTracking(metrics);

            const before = Date.now();
            const result = evaluate({ x: 1, y: 2 });
            const after = Date.now();

            expect(result.timestamp).toBeGreaterThanOrEqual(before);
            expect(result.timestamp).toBeLessThanOrEqual(after);
        });
    });
});

// ==================== Space Tests ====================

describe('Space System', () => {
    describe('Discrete Space', () => {
        it('should create discrete space with n values', () => {
            const space = discrete(5);
            expect(space.kind).toBe('discrete');
            expect(space.n).toBe(5);
        });

        it('should support labels', () => {
            const space = discrete(3, ['low', 'medium', 'high']);
            expect(space.labels).toEqual(['low', 'medium', 'high']);
        });

        it('should sample valid values', () => {
            const space = discrete(10);
            const rng = createMockRNG(42);

            for (let i = 0; i < 100; i++) {
                const value = sample(space, () => rng.next());
                expect(Number.isInteger(value)).toBe(true);
                expect(value).toBeGreaterThanOrEqual(0);
                expect(value).toBeLessThan(10);
            }
        });

        it('should check containment correctly', () => {
            const space = discrete(5);
            expect(contains(space, 0)).toBe(true);
            expect(contains(space, 4)).toBe(true);
            expect(contains(space, 5)).toBe(false);
            expect(contains(space, -1)).toBe(false);
            expect(contains(space, 2.5)).toBe(false);
        });

        it('should report dimension as 1', () => {
            expect(getDimension(discrete(100))).toBe(1);
        });
    });

    describe('Box Space', () => {
        it('should create box space with shape and bounds', () => {
            const space = box([3, 4], -1, 1);
            expect(space.kind).toBe('box');
            expect(space.shape).toEqual([3, 4]);
            expect(space.low).toBe(-1);
            expect(space.high).toBe(1);
        });

        it('should support per-dimension bounds', () => {
            const space = box([2], [0, -10], [1, 10]);
            expect(space.low).toEqual([0, -10]);
            expect(space.high).toEqual([1, 10]);
        });

        it('should sample within bounds', () => {
            const space = box([3], -1, 1);
            const rng = createMockRNG(42);

            for (let i = 0; i < 50; i++) {
                const value = sample(space, () => rng.next()) as number[];
                expect(value.length).toBe(3);
                for (const v of value) {
                    expect(v).toBeGreaterThanOrEqual(-1);
                    expect(v).toBeLessThanOrEqual(1);
                }
            }
        });

        it('should check containment with bounds', () => {
            const space = box([2], 0, 10);
            expect(contains(space, [5, 5])).toBe(true);
            expect(contains(space, [0, 10])).toBe(true);
            expect(contains(space, [-1, 5])).toBe(false);
            expect(contains(space, [5, 11])).toBe(false);
            expect(contains(space, [5])).toBe(false); // Wrong shape
        });

        it('should calculate dimension correctly', () => {
            expect(getDimension(box([3]))).toBe(3);
            expect(getDimension(box([2, 3]))).toBe(6);
            expect(getDimension(box([2, 3, 4]))).toBe(24);
        });
    });

    describe('MultiDiscrete Space', () => {
        it('should create multi-discrete space', () => {
            const space = multiDiscrete([3, 4, 5]);
            expect(space.kind).toBe('multiDiscrete');
            expect(space.nvec).toEqual([3, 4, 5]);
        });

        it('should sample valid multi-discrete values', () => {
            const space = multiDiscrete([2, 3, 4]);
            const rng = createMockRNG(42);

            for (let i = 0; i < 50; i++) {
                const value = sample(space, () => rng.next()) as number[];
                expect(value.length).toBe(3);
                expect(value[0]).toBeGreaterThanOrEqual(0);
                expect(value[0]).toBeLessThan(2);
                expect(value[1]).toBeGreaterThanOrEqual(0);
                expect(value[1]).toBeLessThan(3);
                expect(value[2]).toBeGreaterThanOrEqual(0);
                expect(value[2]).toBeLessThan(4);
            }
        });

        it('should calculate dimension as sum of nvec', () => {
            expect(getDimension(multiDiscrete([2, 3, 4]))).toBe(3);
        });
    });

    describe('Dict Space', () => {
        it('should create dict space from named spaces', () => {
            const space = dict({
                action: discrete(4),
                position: box([2], -10, 10),
            });
            expect(space.kind).toBe('dict');
            expect('action' in space.spaces).toBe(true);
            expect('position' in space.spaces).toBe(true);
        });

        it('should sample all components', () => {
            const space = dict({
                d: discrete(5),
                b: box([2], 0, 1),
            });
            const rng = createMockRNG(42);

            const value = sample(space, () => rng.next()) as Record<string, unknown>;
            expect('d' in value).toBe(true);
            expect('b' in value).toBe(true);
            expect(typeof value.d).toBe('number');
            expect(Array.isArray(value.b)).toBe(true);
        });

        it('should check containment for all components', () => {
            const space = dict({
                x: discrete(3),
                y: box([1], 0, 10),
            });

            expect(contains(space, { x: 1, y: [5] })).toBe(true);
            expect(contains(space, { x: 5, y: [5] })).toBe(false); // x out of range
            expect(contains(space, { x: 1, y: [15] })).toBe(false); // y out of range
            expect(contains(space, { x: 1 })).toBe(false); // missing y
        });

        it('should calculate total dimension', () => {
            const space = dict({
                a: discrete(10),
                b: box([3]),
                c: multiDiscrete([2, 2, 2]),
            });
            expect(getDimension(space)).toBe(1 + 3 + 3); // 7
        });
    });

    describe('Tuple Space', () => {
        it('should create tuple from ordered spaces', () => {
            const space = tuple([discrete(3), box([2], 0, 1)]);
            expect(space.kind).toBe('tuple');
            expect(space.spaces.length).toBe(2);
        });

        it('should sample all components in order', () => {
            const space = tuple([discrete(5), box([2])]);
            const rng = createMockRNG(42);

            const value = sample(space, () => rng.next()) as unknown[];
            expect(value.length).toBe(2);
            expect(typeof value[0]).toBe('number');
            expect(Array.isArray(value[1])).toBe(true);
        });

        it('should calculate total dimension', () => {
            const space = tuple([discrete(5), box([3]), discrete(2)]);
            expect(getDimension(space)).toBe(1 + 3 + 1); // 5
        });
    });

    describe('Space Serialization', () => {
        it('should serialize consistently', () => {
            const space = box([3], -1, 1);
            const schema1 = serialize(space);
            const schema2 = serialize(space);
            expect(schema1).toEqual(schema2);
        });

        it('should serialize different spaces differently', () => {
            const space1 = box([3], -1, 1);
            const space2 = box([4], -1, 1);
            expect(serialize(space1)).not.toEqual(serialize(space2));
        });

        it('should serialize complex spaces', () => {
            const space = dict({
                obs: box([4], -10, 10),
                act: discrete(3),
            });

            const serialized = serialize(space);
            expect(typeof serialized).toBe('string');
            expect(serialized.length).toBeGreaterThan(0);
            // Should contain space kind
            expect(serialized).toContain('dict');
        });
    });

    describe('Edge Cases', () => {
        it('should handle empty box shape', () => {
            const space = box([]);
            expect(getDimension(space)).toBe(1); // Scalar
        });

        it('should handle single-element spaces', () => {
            const d = discrete(1);
            expect(contains(d, 0)).toBe(true);
            expect(sample(d)).toBe(0);
        });

        it('should handle deeply nested dict/tuple', () => {
            const space = dict({
                outer: tuple([
                    dict({
                        inner: box([2]),
                    }),
                    discrete(3),
                ]),
            });

            const dim = getDimension(space);
            expect(dim).toBe(2 + 1); // box[2] + discrete(3)
        });
    });
});

// ==================== Integration Tests ====================

describe('Core Integration', () => {
    it('should combine constraints and objectives for optimization', () => {
        interface OptProblem {
            x: number;
            y: number;
        }

        // Objective: minimize distance from origin
        const objective = simpleMetric<OptProblem>(
            'distance',
            (s) => Math.sqrt(s.x ** 2 + s.y ** 2),
            'minimize'
        );

        // Constraints: stay within bounds
        const constraints = [
            leConstraint<OptProblem>('x_max', (s) => s.x, 10),
            geConstraint<OptProblem>('x_min', (s) => s.x, -10),
            leConstraint<OptProblem>('y_max', (s) => s.y, 10),
            geConstraint<OptProblem>('y_min', (s) => s.y, -10),
        ];

        // Test feasible point
        const feasible = { x: 5, y: 5 };
        const report = evaluateConstraints(constraints, feasible);
        expect(report.feasible).toBe(true);

        const objValue = objective.evaluate(feasible);
        expect(objValue).toBeCloseTo(Math.sqrt(50), 5);

        // Test infeasible point
        const infeasible = { x: 15, y: 5 };
        const infReport = evaluateConstraints(constraints, infeasible);
        expect(infReport.feasible).toBe(false);
    });

    it('should use space for defining valid search regions', () => {
        // Define observation and action spaces
        const obsSpace = box([4], -1, 1);
        const actSpace = discrete(3, ['left', 'forward', 'right']);

        const rng = createMockRNG(123);

        // Sample valid observations and actions
        const obs = sample(obsSpace, () => rng.next()) as number[];
        const act = sample(actSpace, () => rng.next()) as number;

        expect(contains(obsSpace, obs)).toBe(true);
        expect(contains(actSpace, act)).toBe(true);

        // Verify dimensions
        expect(obs.length).toBe(4);
        expect(typeof act).toBe('number');
    });
});
