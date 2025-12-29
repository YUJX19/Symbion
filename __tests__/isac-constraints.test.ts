/**
 * ISAC Domain Constraints Tests
 * 
 * Tests for ISAC-specific ConstraintSpec factory functions.
 * These constraints are defined in isac/constraints/ and provide
 * domain-specific constraint implementations for UAV and communication scenarios.
 * 
 * Coverage:
 * - Flight constraints (speed, acceleration, altitude, no-fly zone, energy)
 * - Communication constraints (SINR, distance)
 * - Integration with core.evaluateConstraint/evaluateConstraints
 */

import { describe, it, expect } from 'vitest';
import {
    evaluateConstraint,
    evaluateConstraints,
} from '../src/core';
import {
    // Flight constraints
    maxSpeedConstraint,
    maxAccelerationConstraint,
    altitudeConstraint,
    noFlyZoneConstraint,
    energyBudgetConstraint,
    // Communication constraints
    minSinrConstraint,
    minDistanceConstraint,
    // Types
    type BoundingBox,
} from '../src/isac/constraints';

// ==================== Test State Types ====================

interface UAVState {
    position: number[];
    velocity: number[];
    acceleration: number[];
    sinrDb: number;
    energyUsed: number;
}

function createUAVState(overrides: Partial<UAVState> = {}): UAVState {
    return {
        position: [100, 200, 50],
        velocity: [5, 5, 0],
        acceleration: [1, 1, 0],
        sinrDb: 10,
        energyUsed: 500,
        ...overrides,
    };
}

interface DistState {
    distance: number;
}

// ==================== Core Interface Compatibility ====================

describe('ISAC Constraints - Core Interface Compatibility', () => {
    it('should produce valid ConstraintSpec with required metadata', () => {
        const constraint = maxSpeedConstraint<UAVState>(20);

        // Verify interface completeness
        expect(constraint.metadata).toBeDefined();
        expect(constraint.metadata.id).toBe('max_speed');
        expect(constraint.metadata.operator).toBe('le');
        expect(constraint.metadata.threshold).toBe(20);
        expect(constraint.metadata.severity).toBe('hard');
        expect(typeof constraint.evaluate).toBe('function');
    });

    it('should work with core evaluateConstraint function', () => {
        const [minConstraint] = altitudeConstraint<UAVState>(30, 150);
        const state = createUAVState({ position: [0, 0, 100] });

        // Core evaluateConstraint should work correctly
        const result = evaluateConstraint(minConstraint, state);
        expect(result).toHaveProperty('satisfied');
        expect(result).toHaveProperty('violation');
        expect(result).toHaveProperty('penalty');
        expect(result).toHaveProperty('value');
    });

    it('should work with core evaluateConstraints batch function', () => {
        const constraints = [
            maxSpeedConstraint<UAVState>(20),
            ...altitudeConstraint<UAVState>(30, 150),
        ];
        const state = createUAVState({
            position: [0, 0, 100],
            velocity: [10, 10, 0],
        });

        const report = evaluateConstraints(constraints, state);
        expect(report.feasible).toBeDefined();
        expect(report.totalPenalty).toBeDefined();
        expect(report.results).toHaveLength(3); // 1 speed + 2 altitude
    });
});

// ==================== Flight Constraints ====================

describe('ISAC Flight Constraints', () => {
    describe('maxSpeedConstraint', () => {
        it('should create max speed constraint', () => {
            const constraint = maxSpeedConstraint<UAVState>(20);
            expect(constraint.metadata.id).toBe('max_speed');
            expect(constraint.metadata.unit).toBe('m/s');
        });

        it('should calculate speed magnitude from velocity vector', () => {
            const constraint = maxSpeedConstraint<UAVState>(20);
            const state = createUAVState({ velocity: [3, 4, 0] }); // |v| = 5
            const result = evaluateConstraint(constraint, state);
            expect(result.value).toBeCloseTo(5, 5);
            expect(result.satisfied).toBe(true);
        });

        it('should detect speed violation', () => {
            const constraint = maxSpeedConstraint<UAVState>(5);
            const state = createUAVState({ velocity: [10, 10, 0] }); // |v| ≈ 14.14
            const result = evaluateConstraint(constraint, state);
            expect(result.satisfied).toBe(false);
            expect(result.violation).toBeGreaterThan(0);
        });

        it('should handle 3D velocity', () => {
            const constraint = maxSpeedConstraint<UAVState>(10);
            const state = createUAVState({ velocity: [3, 4, 0] }); // |v| = 5
            const result = evaluateConstraint(constraint, state);
            expect(result.value).toBeCloseTo(5, 5);
        });

        it('should support soft severity', () => {
            const constraint = maxSpeedConstraint<UAVState>(10, 'soft');
            expect(constraint.metadata.severity).toBe('soft');
        });
    });

    describe('maxAccelerationConstraint', () => {
        it('should create max acceleration constraint', () => {
            const constraint = maxAccelerationConstraint<UAVState>(5);
            expect(constraint.metadata.id).toBe('max_acceleration');
            expect(constraint.metadata.unit).toBe('m/s²');
        });

        it('should calculate acceleration magnitude', () => {
            const constraint = maxAccelerationConstraint<UAVState>(10);
            const state = createUAVState({ acceleration: [3, 4, 0] }); // |a| = 5
            const result = evaluateConstraint(constraint, state);
            expect(result.value).toBeCloseTo(5, 5);
            expect(result.satisfied).toBe(true);
        });

        it('should detect acceleration violation', () => {
            const constraint = maxAccelerationConstraint<UAVState>(2);
            const state = createUAVState({ acceleration: [3, 4, 0] }); // |a| = 5
            const result = evaluateConstraint(constraint, state);
            expect(result.satisfied).toBe(false);
        });
    });

    describe('altitudeConstraint', () => {
        it('should create min and max altitude constraints', () => {
            const constraints = altitudeConstraint<UAVState>(30, 150);
            expect(constraints).toHaveLength(2);
            expect(constraints[0].metadata.id).toBe('min_altitude');
            expect(constraints[1].metadata.id).toBe('max_altitude');
        });

        it('should check altitude bounds correctly', () => {
            const constraints = altitudeConstraint<UAVState>(30, 150);
            const [minConstr, maxConstr] = constraints;

            // Valid altitude
            const validState = createUAVState({ position: [0, 0, 100] });
            expect(evaluateConstraint(minConstr, validState).satisfied).toBe(true);
            expect(evaluateConstraint(maxConstr, validState).satisfied).toBe(true);

            // Too low
            const lowState = createUAVState({ position: [0, 0, 20] });
            expect(evaluateConstraint(minConstr, lowState).satisfied).toBe(false);

            // Too high
            const highState = createUAVState({ position: [0, 0, 200] });
            expect(evaluateConstraint(maxConstr, highState).satisfied).toBe(false);
        });

        it('should handle boundary values', () => {
            const [minConstr, maxConstr] = altitudeConstraint<UAVState>(30, 150);

            const atMin = createUAVState({ position: [0, 0, 30] });
            const atMax = createUAVState({ position: [0, 0, 150] });

            expect(evaluateConstraint(minConstr, atMin).satisfied).toBe(true);
            expect(evaluateConstraint(maxConstr, atMax).satisfied).toBe(true);
        });

        it('should use z-coordinate from position array', () => {
            const [minConstr] = altitudeConstraint<UAVState>(50, 200);
            const state = createUAVState({ position: [100, 200, 75] });
            const result = evaluateConstraint(minConstr, state);
            expect(result.value).toBe(75);
        });
    });

    describe('noFlyZoneConstraint', () => {
        it('should create no-fly zone constraint', () => {
            const zone: BoundingBox = { min: [100, 100, 0], max: [200, 200, 100] };
            const constraint = noFlyZoneConstraint<UAVState>(zone);

            expect(constraint.metadata.id).toBe('no_fly_zone');
        });

        it('should return positive distance when outside zone', () => {
            const zone: BoundingBox = { min: [100, 100, 0], max: [200, 200, 100] };
            const constraint = noFlyZoneConstraint<UAVState>(zone);

            const outsideState = createUAVState({ position: [50, 50, 50] });
            const result = evaluateConstraint(constraint, outsideState);
            expect(result.value).toBeGreaterThan(0);
            expect(result.satisfied).toBe(true);
        });

        it('should return zero distance when inside zone', () => {
            const zone: BoundingBox = { min: [100, 100, 0], max: [200, 200, 100] };
            const constraint = noFlyZoneConstraint<UAVState>(zone);

            const insideState = createUAVState({ position: [150, 150, 50] });
            const result = evaluateConstraint(constraint, insideState);
            // Inside the box, distance to boundary is 0
            expect(result.value).toBe(0);
        });

        it('should calculate distance to zone boundary correctly', () => {
            const zone: BoundingBox = { min: [100, 100, 0], max: [200, 200, 100] };
            const constraint = noFlyZoneConstraint<UAVState>(zone);

            // 50 units away from the zone on x-axis
            const farState = createUAVState({ position: [50, 150, 50] });
            const result = evaluateConstraint(constraint, farState);
            expect(result.value).toBeCloseTo(50, 5); // 100 - 50 = 50 distance
        });
    });

    describe('energyBudgetConstraint', () => {
        it('should create energy budget constraint', () => {
            const constraint = energyBudgetConstraint<UAVState>(1000);

            expect(constraint.metadata.id).toBe('energy_budget');
            expect(constraint.metadata.unit).toBe('J');
            expect(constraint.metadata.severity).toBe('soft');
        });

        it('should check energy usage', () => {
            const constraint = energyBudgetConstraint<UAVState>(1000);

            const underBudget = createUAVState({ energyUsed: 500 });
            expect(evaluateConstraint(constraint, underBudget).satisfied).toBe(true);

            const overBudget = createUAVState({ energyUsed: 1500 });
            expect(evaluateConstraint(constraint, overBudget).satisfied).toBe(false);
        });

        it('should support hard severity', () => {
            const constraint = energyBudgetConstraint<UAVState>(1000, 'hard');
            expect(constraint.metadata.severity).toBe('hard');
        });
    });
});

// ==================== Communication Constraints ====================

describe('ISAC Communication Constraints', () => {
    describe('minSinrConstraint', () => {
        it('should create min SINR constraint', () => {
            const constraint = minSinrConstraint<UAVState>(0);
            expect(constraint.metadata.id).toBe('min_sinr');
            expect(constraint.metadata.unit).toBe('dB');
        });

        it('should default to soft severity', () => {
            const constraint = minSinrConstraint<UAVState>(0);
            expect(constraint.metadata.severity).toBe('soft');
        });

        it('should check SINR threshold', () => {
            const constraint = minSinrConstraint<UAVState>(5);

            const goodState = createUAVState({ sinrDb: 10 });
            expect(evaluateConstraint(constraint, goodState).satisfied).toBe(true);

            const badState = createUAVState({ sinrDb: 2 });
            expect(evaluateConstraint(constraint, badState).satisfied).toBe(false);
        });

        it('should handle negative SINR values', () => {
            const constraint = minSinrConstraint<UAVState>(-5);

            const aboveThreshold = createUAVState({ sinrDb: -3 });
            const belowThreshold = createUAVState({ sinrDb: -8 });

            expect(evaluateConstraint(constraint, aboveThreshold).satisfied).toBe(true);
            expect(evaluateConstraint(constraint, belowThreshold).satisfied).toBe(false);
        });
    });

    describe('minDistanceConstraint', () => {
        it('should create min distance constraint with custom evaluator', () => {
            const constraint = minDistanceConstraint<DistState>(
                'obstacle_dist',
                (s) => s.distance,
                5
            );

            expect(constraint.metadata.id).toBe('obstacle_dist');
            expect(evaluateConstraint(constraint, { distance: 10 }).satisfied).toBe(true);
            expect(evaluateConstraint(constraint, { distance: 3 }).satisfied).toBe(false);
        });

        it('should support complex distance evaluators', () => {
            interface PositionState {
                position: { x: number; y: number };
                target: { x: number; y: number };
            }

            const constraint = minDistanceConstraint<PositionState>(
                'target_distance',
                (s) => Math.sqrt(
                    (s.position.x - s.target.x) ** 2 +
                    (s.position.y - s.target.y) ** 2
                ),
                5
            );

            const close = { position: { x: 0, y: 0 }, target: { x: 2, y: 2 } }; // dist ≈ 2.83
            const far = { position: { x: 0, y: 0 }, target: { x: 10, y: 10 } }; // dist ≈ 14.14

            expect(evaluateConstraint(constraint, close).satisfied).toBe(false);
            expect(evaluateConstraint(constraint, far).satisfied).toBe(true);
        });
    });
});

// ==================== Integration Tests ====================

describe('ISAC Constraint Integration', () => {
    it('should work with complete UAV state validation', () => {
        const constraints = [
            maxSpeedConstraint<UAVState>(20, 'hard'),
            ...altitudeConstraint<UAVState>(30, 150, 'hard'),
            minSinrConstraint<UAVState>(0, 'soft'),
            energyBudgetConstraint<UAVState>(1000, 'soft'),
        ];

        // Valid state
        const validState = createUAVState({
            position: [100, 100, 80],
            velocity: [10, 10, 0],
            sinrDb: 15,
            energyUsed: 500,
        });

        const report = evaluateConstraints(constraints, validState);
        expect(report.feasible).toBe(true);
        expect(report.totalPenalty).toBe(0);
    });

    it('should handle mixed hard/soft violations', () => {
        const constraints = [
            maxSpeedConstraint<UAVState>(10, 'hard'),
            minSinrConstraint<UAVState>(5, 'soft'),
        ];

        const state = createUAVState({
            velocity: [15, 0, 0],  // Speed = 15 > 10 (hard violation)
            sinrDb: 2,            // SINR < 5 (soft violation)
        });

        const report = evaluateConstraints(constraints, state);
        expect(report.feasible).toBe(false); // Hard constraint violated
        expect(report.violations).toHaveLength(2);
    });

    it('should calculate correct penalty for soft violations only', () => {
        const constraints = [
            maxSpeedConstraint<UAVState>(20, 'hard'),
            energyBudgetConstraint<UAVState>(500, 'soft'),
        ];

        const state = createUAVState({
            velocity: [5, 0, 0],   // Speed = 5 < 20 (satisfied)
            energyUsed: 600,       // > 500 (soft violation)
        });

        const report = evaluateConstraints(constraints, state);
        expect(report.feasible).toBe(true);
        expect(report.totalPenalty).toBeGreaterThan(0);
    });

    it('should handle all constraints satisfied', () => {
        const zone: BoundingBox = { min: [500, 500, 0], max: [600, 600, 100] };

        const constraints = [
            maxSpeedConstraint<UAVState>(50),
            maxAccelerationConstraint<UAVState>(10),
            ...altitudeConstraint<UAVState>(10, 200),
            noFlyZoneConstraint<UAVState>(zone),
            energyBudgetConstraint<UAVState>(2000),
            minSinrConstraint<UAVState>(-10),
        ];

        const goodState = createUAVState({
            position: [100, 100, 50],  // Outside no-fly zone
            velocity: [10, 10, 0],      // Speed ≈ 14 < 50
            acceleration: [2, 2, 0],    // Accel ≈ 2.8 < 10
            sinrDb: 5,                  // > -10
            energyUsed: 1000,           // < 2000
        });

        const report = evaluateConstraints(constraints, goodState);
        expect(report.feasible).toBe(true);
        expect(report.violations).toHaveLength(0);
    });
});
