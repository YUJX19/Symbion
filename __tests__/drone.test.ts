/**
 * Drone Module Tests
 * Tests for drone utility functions
 */

import { describe, it, expect } from 'vitest';
import {
    normalizeAngle,
    distance,
    goalReached,
    gaussianRandom,
} from '../src/models/robotics/drone/utils';
import { isClose, mean, std } from './test-utils';

// ==================== normalizeAngle Tests ====================

describe('normalizeAngle', () => {
    it('should keep angles within [-π, π] unchanged', () => {
        expect(normalizeAngle(0)).toBe(0);
        expect(normalizeAngle(1)).toBe(1);
        expect(normalizeAngle(-1)).toBe(-1);
    });

    it('should normalize angles greater than π', () => {
        // 2π should wrap to 0
        expect(isClose(normalizeAngle(2 * Math.PI), 0, 1e-10)).toBe(true);
        // 3π should wrap to π (or -π)
        const result = normalizeAngle(3 * Math.PI);
        expect(Math.abs(result) - Math.PI).toBeLessThan(1e-10);
    });

    it('should normalize angles less than -π', () => {
        // -2π should wrap to 0
        expect(isClose(normalizeAngle(-2 * Math.PI), 0, 1e-10)).toBe(true);
        // -3π should wrap to π (or -π)
        const result = normalizeAngle(-3 * Math.PI);
        expect(Math.abs(result) - Math.PI).toBeLessThan(1e-10);
    });

    it('should handle multiple rotations', () => {
        // 5π = 2*2π + π → wraps to π (or -π)
        const result5pi = normalizeAngle(5 * Math.PI);
        expect(Math.abs(result5pi) - Math.PI).toBeLessThan(1e-10);

        // 10π → wraps to 0
        expect(isClose(normalizeAngle(10 * Math.PI), 0, 1e-10)).toBe(true);
    });

    it('should handle boundary values', () => {
        // Exactly π should stay (or become -π)
        expect(Math.abs(normalizeAngle(Math.PI))).toBeCloseTo(Math.PI, 10);
        // Exactly -π should stay
        expect(normalizeAngle(-Math.PI)).toBeCloseTo(-Math.PI, 10);
    });

    it('should handle very large angles', () => {
        const largeAngle = 1000 * Math.PI;
        const normalized = normalizeAngle(largeAngle);
        expect(normalized).toBeGreaterThanOrEqual(-Math.PI);
        expect(normalized).toBeLessThanOrEqual(Math.PI);
    });
});

// ==================== distance Tests ====================

describe('distance', () => {
    it('should compute Euclidean distance between two points', () => {
        expect(distance({ x: 0, y: 0 }, { x: 3, y: 4 })).toBe(5);
    });

    it('should return 0 for same point', () => {
        expect(distance({ x: 5, y: 5 }, { x: 5, y: 5 })).toBe(0);
    });

    it('should handle negative coordinates', () => {
        expect(distance({ x: -3, y: -4 }, { x: 0, y: 0 })).toBe(5);
    });

    it('should be symmetric', () => {
        const p1 = { x: 1, y: 2 };
        const p2 = { x: 5, y: 7 };
        expect(distance(p1, p2)).toBe(distance(p2, p1));
    });

    it('should handle large values', () => {
        const d = distance({ x: 0, y: 0 }, { x: 1000, y: 1000 });
        expect(isClose(d, Math.sqrt(2) * 1000, 1e-6)).toBe(true);
    });

    it('should handle small values', () => {
        const d = distance({ x: 0, y: 0 }, { x: 0.001, y: 0.001 });
        expect(isClose(d, Math.sqrt(2) * 0.001, 1e-10)).toBe(true);
    });
});

// ==================== goalReached Tests ====================

describe('goalReached', () => {
    it('should return true when at goal', () => {
        const pose = { x: 5, y: 5, theta: 0 };
        const goal = { x: 5, y: 5, theta: Math.PI };
        expect(goalReached(pose, goal)).toBe(true);
    });

    it('should return true when within default threshold', () => {
        const pose = { x: 5, y: 5, theta: 0 };
        const goal = { x: 5.05, y: 5.05, theta: 0 };
        // Distance = sqrt(0.05^2 + 0.05^2) ≈ 0.07 < 0.1
        expect(goalReached(pose, goal)).toBe(true);
    });

    it('should return false when outside threshold', () => {
        const pose = { x: 0, y: 0, theta: 0 };
        const goal = { x: 1, y: 1, theta: 0 };
        expect(goalReached(pose, goal, 0.1)).toBe(false);
    });

    it('should respect custom threshold', () => {
        const pose = { x: 0, y: 0, theta: 0 };
        const goal = { x: 0.5, y: 0, theta: 0 };

        expect(goalReached(pose, goal, 0.4)).toBe(false);
        expect(goalReached(pose, goal, 0.6)).toBe(true);
    });

    it('should ignore theta (heading) difference', () => {
        const pose = { x: 5, y: 5, theta: 0 };
        const goal = { x: 5, y: 5, theta: Math.PI };
        // Same position, different heading
        expect(goalReached(pose, goal)).toBe(true);
    });

    it('should handle edge case at exact threshold', () => {
        const pose = { x: 0, y: 0, theta: 0 };
        const goal = { x: 0.1, y: 0, theta: 0 };
        // Distance = 0.1, threshold = 0.1 → just at boundary
        expect(goalReached(pose, goal, 0.1)).toBe(false); // < not <=
    });
});

// ==================== gaussianRandom Tests ====================

describe('gaussianRandom', () => {
    it('should return a finite number', () => {
        for (let i = 0; i < 100; i++) {
            const val = gaussianRandom();
            expect(Number.isFinite(val)).toBe(true);
        }
    });

    it('should have mean approximately 0 (statistical test)', () => {
        const samples = 10000;
        const values: number[] = [];

        for (let i = 0; i < samples; i++) {
            values.push(gaussianRandom());
        }

        const avg = mean(values);
        // Mean should be close to 0 with high probability
        expect(Math.abs(avg)).toBeLessThan(0.05);
    });

    it('should have standard deviation approximately 1 (statistical test)', () => {
        const samples = 10000;
        const values: number[] = [];

        for (let i = 0; i < samples; i++) {
            values.push(gaussianRandom());
        }

        const stdDev = std(values);
        // Std dev should be close to 1
        expect(stdDev).toBeGreaterThan(0.9);
        expect(stdDev).toBeLessThan(1.1);
    });

    it('should produce values in typical Gaussian range', () => {
        const samples = 1000;
        let countOutside3Sigma = 0;

        for (let i = 0; i < samples; i++) {
            const val = gaussianRandom();
            if (Math.abs(val) > 3) {
                countOutside3Sigma++;
            }
        }

        // ~99.7% should be within 3 sigma, so <1% outside
        expect(countOutside3Sigma / samples).toBeLessThan(0.01);
    });

    it('should produce different values on successive calls', () => {
        const values = new Set<number>();
        for (let i = 0; i < 100; i++) {
            values.add(gaussianRandom());
        }
        // Should have many unique values
        expect(values.size).toBeGreaterThan(90);
    });

    it('should follow Gaussian distribution (Kolmogorov-Smirnov approximation)', () => {
        const samples = 5000;
        const values: number[] = [];

        for (let i = 0; i < samples; i++) {
            values.push(gaussianRandom());
        }

        // Count proportion in each range
        const inMinus1To1 = values.filter(v => v >= -1 && v <= 1).length / samples;
        const inMinus2To2 = values.filter(v => v >= -2 && v <= 2).length / samples;

        // For standard normal: ~68.27% within [-1,1], ~95.45% within [-2,2]
        expect(inMinus1To1).toBeGreaterThan(0.63);
        expect(inMinus1To1).toBeLessThan(0.73);
        expect(inMinus2To2).toBeGreaterThan(0.93);
        expect(inMinus2To2).toBeLessThan(0.97);
    });
});
