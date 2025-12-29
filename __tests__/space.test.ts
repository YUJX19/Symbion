/**
 * Core Space Module Tests
 * Tests for AI environment space definitions and operations
 */

import { describe, it, expect } from 'vitest';
import {
    // Space factories
    discrete,
    box,
    multiDiscrete,
    dict,
    tuple,
    // Space operations
    sample,
    contains,
    getDimension,
    serialize,
    computeSchemaHash,
    flatten,
    unflatten,
    // RNG
    SeededRandom,
    createRng,
    // Types
    type Space,
    type DiscreteSpace,
    type BoxSpace,
    type MultiDiscreteSpace,
    type DictSpace,
    type TupleSpace,
} from '../src/core';

// ==================== Space Factories ====================

describe('Space Factories', () => {
    describe('discrete', () => {
        it('should create discrete space with n values', () => {
            const space = discrete(5);
            expect(space.kind).toBe('discrete');
            expect(space.n).toBe(5);
        });

        it('should create discrete space with labels', () => {
            const labels = ['action1', 'action2', 'action3'];
            const space = discrete(3, labels);
            expect(space.labels).toEqual(labels);
        });

        it('should create MCS selection space', () => {
            const mcsSpace = discrete(29);
            expect(mcsSpace.n).toBe(29);
        });
    });

    describe('box', () => {
        it('should create box space with shape', () => {
            const space = box([3]);
            expect(space.kind).toBe('box');
            expect(space.shape).toEqual([3]);
        });

        it('should create box space with scalar bounds', () => {
            const space = box([4], -1, 1);
            expect(space.low).toBe(-1);
            expect(space.high).toBe(1);
        });

        it('should create box space with per-dimension bounds', () => {
            const space = box([3], [0, 0, 30], [1000, 1000, 150]);
            expect(space.low).toEqual([0, 0, 30]);
            expect(space.high).toEqual([1000, 1000, 150]);
        });

        it('should create box space with default infinite bounds', () => {
            const space = box([5]);
            expect(space.low).toBe(-Infinity);
            expect(space.high).toBe(Infinity);
        });

        it('should create box space with dtype', () => {
            const space = box([3], 0, 1, 'float64');
            expect(space.dtype).toBe('float64');
        });

        it('should default to float32 dtype', () => {
            const space = box([3], 0, 1);
            expect(space.dtype).toBe('float32');
        });

        it('should create 2D observation space', () => {
            const space = box([10, 10], 0, 255, 'float32');
            expect(space.shape).toEqual([10, 10]);
        });
    });

    describe('multiDiscrete', () => {
        it('should create multi-discrete space', () => {
            const space = multiDiscrete([9, 9, 5]);
            expect(space.kind).toBe('multiDiscrete');
            expect(space.nvec).toEqual([9, 9, 5]);
        });

        it('should create multi-discrete space with labels', () => {
            const labels = [
                ['left', 'stay', 'right'],
                ['back', 'stay', 'forward'],
            ];
            const space = multiDiscrete([3, 3], labels);
            expect(space.labels).toEqual(labels);
        });

        it('should create grid action space', () => {
            const space = multiDiscrete([9, 9, 5, 29]); // x, y, z, MCS
            expect(space.nvec.length).toBe(4);
        });
    });

    describe('dict', () => {
        it('should create dict space', () => {
            const space = dict({
                position: box([3], 0, 1000),
                velocity: box([3], -20, 20),
            });
            expect(space.kind).toBe('dict');
            expect(Object.keys(space.spaces)).toEqual(['position', 'velocity']);
        });

        it('should create nested dict space', () => {
            const space = dict({
                motion: dict({
                    position: box([3]),
                    velocity: box([3]),
                }),
                communication: dict({
                    mcs: discrete(29),
                    power: box([1], 0, 23),
                }),
            });
            expect(space.kind).toBe('dict');
            expect((space.spaces.motion as DictSpace).kind).toBe('dict');
        });

        it('should create observation space with mixed types', () => {
            const space = dict({
                position: box([3], 0, 1000),
                mcsIndex: discrete(29),
                txPower: box([1], 0, 23),
            });
            expect((space.spaces.position as BoxSpace).kind).toBe('box');
            expect((space.spaces.mcsIndex as DiscreteSpace).kind).toBe('discrete');
        });
    });

    describe('tuple', () => {
        it('should create tuple space', () => {
            const space = tuple([
                box([3], -1, 1),
                discrete(29),
            ]);
            expect(space.kind).toBe('tuple');
            expect(space.spaces.length).toBe(2);
        });

        it('should create action space with continuous and discrete parts', () => {
            const space = tuple([
                box([3], -1, 1),    // continuous movement
                discrete(29),       // MCS selection
                discrete(24),       // power level
            ]);
            expect(space.spaces.length).toBe(3);
        });
    });
});

// ==================== sample ====================

describe('sample', () => {
    describe('deterministic sampling', () => {
        it('should sample from discrete space', () => {
            const space = discrete(5);
            const rng = createRng(42);
            const value = sample(space, rng.random.bind(rng));
            expect(typeof value).toBe('number');
            expect(value).toBeGreaterThanOrEqual(0);
            expect(value).toBeLessThan(5);
        });

        it('should sample from box space with finite bounds', () => {
            const space = box([3], 0, 1);
            const rng = createRng(42);
            const value = sample(space, rng.random.bind(rng)) as number[];
            expect(value).toHaveLength(3);
            value.forEach(v => {
                expect(v).toBeGreaterThanOrEqual(0);
                expect(v).toBeLessThanOrEqual(1);
            });
        });

        it('should sample from box space with infinite bounds', () => {
            const space = box([2]);
            const rng = createRng(42);
            const value = sample(space, rng.random.bind(rng)) as number[];
            expect(value).toHaveLength(2);
            // Should use fallback bounds
            value.forEach(v => {
                expect(v).toBeGreaterThanOrEqual(-1e6);
                expect(v).toBeLessThanOrEqual(1e6);
            });
        });

        it('should sample from box space with per-dimension bounds', () => {
            const space = box([3], [0, 10, 100], [5, 20, 200]);
            const rng = createRng(42);
            const value = sample(space, rng.random.bind(rng)) as number[];
            expect(value[0]).toBeGreaterThanOrEqual(0);
            expect(value[0]).toBeLessThanOrEqual(5);
            expect(value[1]).toBeGreaterThanOrEqual(10);
            expect(value[1]).toBeLessThanOrEqual(20);
            expect(value[2]).toBeGreaterThanOrEqual(100);
            expect(value[2]).toBeLessThanOrEqual(200);
        });

        it('should sample from multiDiscrete space', () => {
            const space = multiDiscrete([3, 5, 7]);
            const rng = createRng(42);
            const value = sample(space, rng.random.bind(rng)) as number[];
            expect(value).toHaveLength(3);
            expect(value[0]).toBeGreaterThanOrEqual(0);
            expect(value[0]).toBeLessThan(3);
            expect(value[1]).toBeGreaterThanOrEqual(0);
            expect(value[1]).toBeLessThan(5);
            expect(value[2]).toBeGreaterThanOrEqual(0);
            expect(value[2]).toBeLessThan(7);
        });

        it('should sample from dict space', () => {
            const space = dict({
                position: box([3], 0, 100),
                action: discrete(5),
            });
            const rng = createRng(42);
            const value = sample(space, rng.random.bind(rng)) as Record<string, unknown>;
            expect(value).toHaveProperty('position');
            expect(value).toHaveProperty('action');
            expect((value.position as number[]).length).toBe(3);
            expect(typeof value.action).toBe('number');
        });

        it('should sample from tuple space', () => {
            const space = tuple([
                box([2], -1, 1),
                discrete(10),
            ]);
            const rng = createRng(42);
            const value = sample(space, rng.random.bind(rng)) as unknown[];
            expect(value).toHaveLength(2);
            expect((value[0] as number[]).length).toBe(2);
            expect(typeof value[1]).toBe('number');
        });
    });

    describe('reproducibility', () => {
        it('should produce same values with same seed', () => {
            const space = box([5], 0, 1);
            const rng1 = createRng(12345);
            const rng2 = createRng(12345);

            const value1 = sample(space, rng1.random.bind(rng1));
            const value2 = sample(space, rng2.random.bind(rng2));

            expect(value1).toEqual(value2);
        });

        it('should produce different values with different seeds', () => {
            const space = discrete(100);
            const rng1 = createRng(11111);
            const rng2 = createRng(22222);

            const values1: number[] = [];
            const values2: number[] = [];
            for (let i = 0; i < 10; i++) {
                values1.push(sample(space, rng1.random.bind(rng1)) as number);
                values2.push(sample(space, rng2.random.bind(rng2)) as number);
            }

            expect(values1).not.toEqual(values2);
        });
    });
});

// ==================== contains ====================

describe('contains', () => {
    describe('discrete space', () => {
        const space = discrete(5);

        it('should accept valid discrete values', () => {
            expect(contains(space, 0)).toBe(true);
            expect(contains(space, 4)).toBe(true);
            expect(contains(space, 2)).toBe(true);
        });

        it('should reject out of range values', () => {
            expect(contains(space, -1)).toBe(false);
            expect(contains(space, 5)).toBe(false);
            expect(contains(space, 100)).toBe(false);
        });

        it('should reject non-integer values', () => {
            expect(contains(space, 1.5)).toBe(false);
            expect(contains(space, 2.999)).toBe(false);
        });

        it('should reject non-number values', () => {
            expect(contains(space, 'string')).toBe(false);
            expect(contains(space, null)).toBe(false);
            expect(contains(space, undefined)).toBe(false);
        });
    });

    describe('box space', () => {
        it('should accept values within scalar bounds', () => {
            const space = box([3], -1, 1);
            expect(contains(space, [0, 0, 0])).toBe(true);
            expect(contains(space, [-1, 0, 1])).toBe(true);
            expect(contains(space, [0.5, -0.5, 0.9])).toBe(true);
        });

        it('should reject values outside scalar bounds', () => {
            const space = box([3], -1, 1);
            expect(contains(space, [0, 0, 1.1])).toBe(false);
            expect(contains(space, [-1.5, 0, 0])).toBe(false);
        });

        it('should accept values within per-dimension bounds', () => {
            const space = box([3], [0, 0, 30], [1000, 1000, 150]);
            expect(contains(space, [500, 500, 100])).toBe(true);
            expect(contains(space, [0, 0, 30])).toBe(true);
            expect(contains(space, [1000, 1000, 150])).toBe(true);
        });

        it('should reject values outside per-dimension bounds', () => {
            const space = box([3], [0, 0, 30], [1000, 1000, 150]);
            expect(contains(space, [500, 500, 25])).toBe(false);  // z too low
            expect(contains(space, [1001, 500, 100])).toBe(false); // x too high
        });

        it('should reject wrong length arrays', () => {
            const space = box([3], -1, 1);
            expect(contains(space, [0, 0])).toBe(false);
            expect(contains(space, [0, 0, 0, 0])).toBe(false);
        });

        it('should reject non-array values', () => {
            const space = box([3], -1, 1);
            expect(contains(space, 0)).toBe(false);
            expect(contains(space, 'string')).toBe(false);
        });
    });

    describe('multiDiscrete space', () => {
        const space = multiDiscrete([3, 5, 7]);

        it('should accept valid multi-discrete values', () => {
            expect(contains(space, [0, 0, 0])).toBe(true);
            expect(contains(space, [2, 4, 6])).toBe(true);
            expect(contains(space, [1, 2, 3])).toBe(true);
        });

        it('should reject out of range values', () => {
            expect(contains(space, [3, 0, 0])).toBe(false);  // first dim >= 3
            expect(contains(space, [0, 5, 0])).toBe(false);  // second dim >= 5
            expect(contains(space, [0, 0, 7])).toBe(false);  // third dim >= 7
        });

        it('should reject negative values', () => {
            expect(contains(space, [-1, 0, 0])).toBe(false);
        });

        it('should reject wrong length arrays', () => {
            expect(contains(space, [0, 0])).toBe(false);
            expect(contains(space, [0, 0, 0, 0])).toBe(false);
        });
    });

    describe('dict space', () => {
        const space = dict({
            position: box([3], 0, 100),
            action: discrete(5),
        });

        it('should accept valid dict values', () => {
            expect(contains(space, { position: [50, 50, 50], action: 2 })).toBe(true);
        });

        it('should reject missing keys', () => {
            expect(contains(space, { position: [50, 50, 50] })).toBe(false);
            expect(contains(space, { action: 2 })).toBe(false);
        });

        it('should reject invalid sub-values', () => {
            expect(contains(space, { position: [50, 50, 150], action: 2 })).toBe(false);
            expect(contains(space, { position: [50, 50, 50], action: 10 })).toBe(false);
        });

        it('should reject non-object values', () => {
            expect(contains(space, null)).toBe(false);
            expect(contains(space, [50, 50, 50])).toBe(false);
        });
    });

    describe('tuple space', () => {
        const space = tuple([
            box([2], -1, 1),
            discrete(5),
        ]);

        it('should accept valid tuple values', () => {
            expect(contains(space, [[0, 0], 2])).toBe(true);
            expect(contains(space, [[-1, 1], 0])).toBe(true);
        });

        it('should reject invalid sub-values', () => {
            expect(contains(space, [[0, 2], 2])).toBe(false);  // box value out of range
            expect(contains(space, [[0, 0], 10])).toBe(false); // discrete value out of range
        });

        it('should reject wrong length tuples', () => {
            expect(contains(space, [[0, 0]])).toBe(false);
            expect(contains(space, [[0, 0], 2, 3])).toBe(false);
        });
    });
});

// ==================== getDimension ====================

describe('getDimension', () => {
    it('should return 1 for discrete space', () => {
        expect(getDimension(discrete(10))).toBe(1);
        expect(getDimension(discrete(100))).toBe(1);
    });

    it('should return product of shape for box space', () => {
        expect(getDimension(box([3]))).toBe(3);
        expect(getDimension(box([4, 4]))).toBe(16);
        expect(getDimension(box([2, 3, 4]))).toBe(24);
    });

    it('should return nvec length for multiDiscrete space', () => {
        expect(getDimension(multiDiscrete([3, 5, 7]))).toBe(3);
        expect(getDimension(multiDiscrete([10]))).toBe(1);
    });

    it('should return sum of sub-dimensions for dict space', () => {
        const space = dict({
            position: box([3]),
            velocity: box([3]),
            mcs: discrete(29),
        });
        expect(getDimension(space)).toBe(3 + 3 + 1);
    });

    it('should return sum of sub-dimensions for tuple space', () => {
        const space = tuple([
            box([3]),
            discrete(5),
            multiDiscrete([9, 9]),
        ]);
        expect(getDimension(space)).toBe(3 + 1 + 2);
    });

    it('should handle nested dict spaces', () => {
        const space = dict({
            motion: dict({
                position: box([3]),
                velocity: box([3]),
            }),
            communication: dict({
                mcs: discrete(29),
                power: box([1]),
            }),
        });
        expect(getDimension(space)).toBe(3 + 3 + 1 + 1);
    });
});

// ==================== serialize ====================

describe('serialize', () => {
    it('should serialize discrete space', () => {
        const space = discrete(5);
        const json = serialize(space);
        expect(JSON.parse(json)).toEqual(space);
    });

    it('should serialize box space', () => {
        const space = box([3], 0, 1);
        const json = serialize(space);
        expect(JSON.parse(json).kind).toBe('box');
    });

    it('should produce consistent output for same space', () => {
        const space1 = dict({ a: discrete(3), b: box([2]) });
        const space2 = dict({ a: discrete(3), b: box([2]) });
        expect(serialize(space1)).toBe(serialize(space2));
    });
});

// ==================== computeSchemaHash ====================

describe('computeSchemaHash', () => {
    it('should compute hash for observation and action spaces', () => {
        const obsSpace = box([10]);
        const actSpace = discrete(5);
        const hash = computeSchemaHash(obsSpace, actSpace);
        expect(typeof hash).toBe('string');
        expect(hash.length).toBe(32);
    });

    it('should produce same hash for same spaces', () => {
        const obs1 = box([3], 0, 1);
        const act1 = discrete(5);
        const hash1 = computeSchemaHash(obs1, act1);

        const obs2 = box([3], 0, 1);
        const act2 = discrete(5);
        const hash2 = computeSchemaHash(obs2, act2);

        expect(hash1).toBe(hash2);
    });

    it('should produce different hash for different spaces', () => {
        const obs = box([3]);
        const hash1 = computeSchemaHash(obs, discrete(5));
        const hash2 = computeSchemaHash(obs, discrete(10));
        expect(hash1).not.toBe(hash2);
    });
});

// ==================== flatten ====================

describe('flatten', () => {
    it('should flatten discrete value', () => {
        const space = discrete(5);
        expect(flatten(space, 3)).toEqual([3]);
    });

    it('should flatten box value', () => {
        const space = box([3]);
        expect(flatten(space, [1, 2, 3])).toEqual([1, 2, 3]);
    });

    it('should flatten multiDiscrete value', () => {
        const space = multiDiscrete([3, 5, 7]);
        expect(flatten(space, [1, 2, 3])).toEqual([1, 2, 3]);
    });

    it('should flatten dict value in sorted key order', () => {
        const space = dict({
            b: box([2]),
            a: discrete(5),
        });
        const value = { a: 3, b: [1, 2] };
        // Keys sorted: a, b
        expect(flatten(space, value)).toEqual([3, 1, 2]);
    });

    it('should flatten tuple value', () => {
        const space = tuple([
            discrete(5),
            box([2]),
        ]);
        const value = [3, [1, 2]];
        expect(flatten(space, value)).toEqual([3, 1, 2]);
    });

    it('should flatten nested dict', () => {
        const space = dict({
            motion: dict({
                position: box([2]),
                velocity: box([2]),
            }),
            action: discrete(5),
        });
        const value = {
            motion: { position: [1, 2], velocity: [3, 4] },
            action: 2,
        };
        // Keys sorted: action, motion.position, motion.velocity
        expect(flatten(space, value)).toEqual([2, 1, 2, 3, 4]);
    });
});

// ==================== unflatten ====================

describe('unflatten', () => {
    it('should unflatten to discrete value', () => {
        const space = discrete(5);
        const { value, consumed } = unflatten(space, [3]);
        expect(value).toBe(3);
        expect(consumed).toBe(1);
    });

    it('should unflatten to box value', () => {
        const space = box([3]);
        const { value, consumed } = unflatten(space, [1, 2, 3]);
        expect(value).toEqual([1, 2, 3]);
        expect(consumed).toBe(3);
    });

    it('should unflatten to multiDiscrete value', () => {
        const space = multiDiscrete([3, 5, 7]);
        const { value, consumed } = unflatten(space, [1, 2, 3]);
        expect(value).toEqual([1, 2, 3]);
        expect(consumed).toBe(3);
    });

    it('should unflatten to dict value', () => {
        const space = dict({
            b: box([2]),
            a: discrete(5),
        });
        // Keys sorted: a, b
        const { value, consumed } = unflatten(space, [3, 1, 2]);
        expect(value).toEqual({ a: 3, b: [1, 2] });
        expect(consumed).toBe(3);
    });

    it('should unflatten to tuple value', () => {
        const space = tuple([
            discrete(5),
            box([2]),
        ]);
        const { value, consumed } = unflatten(space, [3, 1, 2]);
        expect(value).toEqual([3, [1, 2]]);
        expect(consumed).toBe(3);
    });

    it('should use offset correctly', () => {
        const space = box([2]);
        const { value, consumed } = unflatten(space, [0, 0, 1, 2, 0, 0], 2);
        expect(value).toEqual([1, 2]);
        expect(consumed).toBe(2);
    });

    it('should round-trip flatten/unflatten', () => {
        const space = dict({
            position: box([3]),
            velocity: box([3]),
            mcs: discrete(29),
        });
        const original = {
            position: [100, 200, 50],
            velocity: [1, 2, 3],
            mcs: 15,
        };

        const flat = flatten(space, original);
        const { value } = unflatten(space, flat);
        expect(value).toEqual(original);
    });
});

// ==================== Integration Tests ====================

describe('Space Integration', () => {
    it('should work with UAV observation space', () => {
        const obsSpace = dict({
            position: box([3], 0, 1000),
            velocity: box([3], -20, 20),
            acceleration: box([3], -5, 5),
            mcsIndex: discrete(29),
            txPower: box([1], 0, 23),
            sinrDb: box([1], -20, 40),
        });

        expect(getDimension(obsSpace)).toBe(3 + 3 + 3 + 1 + 1 + 1);

        const rng = createRng(42);
        const value = sample(obsSpace, rng.random.bind(rng));
        expect(contains(obsSpace, value)).toBe(true);

        const flat = flatten(obsSpace, value);
        expect(flat.length).toBe(12);

        const { value: restored } = unflatten(obsSpace, flat);
        expect(contains(obsSpace, restored)).toBe(true);
    });

    it('should work with UAV action space', () => {
        const actSpace = tuple([
            box([3], -1, 1),  // normalized movement
            discrete(29),     // MCS selection
        ]);

        expect(getDimension(actSpace)).toBe(4);

        const rng = createRng(42);
        const action = sample(actSpace, rng.random.bind(rng));
        expect(contains(actSpace, action)).toBe(true);
    });

    it('should validate schema hash for reproducibility', () => {
        const obs1 = dict({
            position: box([3], 0, 1000),
            mcs: discrete(29),
        });
        const act1 = box([3], -1, 1);

        const obs2 = dict({
            position: box([3], 0, 1000),
            mcs: discrete(29),
        });
        const act2 = box([3], -1, 1);

        const hash1 = computeSchemaHash(obs1, act1);
        const hash2 = computeSchemaHash(obs2, act2);

        expect(hash1).toBe(hash2);
    });
});
