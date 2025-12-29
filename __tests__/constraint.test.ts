/**
 * Core Constraint Module Tests
 * 
 * Tests for generic constraint abstractions and evaluation logic.
 * Domain-specific constraints (UAV, communication, etc.) are tested
 * in isac-constraints.test.ts.
 * 
 * Coverage:
 * - ConstraintSpec factory functions (leConstraint, geConstraint, eqConstraint)
 * - Constraint evaluation logic (evaluateConstraint, evaluateConstraints)
 * - Penalty calculation (constraintsToPenalty)
 */

import { describe, it, expect } from 'vitest';
import {
    // Factory functions
    asConstraintSpec,
    leConstraint,
    geConstraint,
    eqConstraint,
    // Evaluation
    evaluateConstraint,
    evaluateConstraints,
    constraintsToPenalty,
    // Types (used in type annotations for test expectations)
    type ConstraintSpec,
    type ConstraintResult,
    type ConstraintReport,
    type ConstraintSeverity,
} from '../src/core';

// ==================== Test State Types ====================

interface SimpleState {
    value: number;
}


// ==================== asConstraintSpec ====================

describe('asConstraintSpec', () => {
    it('should create constraint spec with defaults', () => {
        const constraint = asConstraintSpec<SimpleState>(
            (state) => state.value,
            {
                id: 'test_constraint',
                description: 'Test constraint',
                severity: 'hard',
                threshold: 10,
                operator: 'le',
            }
        );

        expect(constraint.metadata.id).toBe('test_constraint');
        expect(constraint.metadata.penaltyWeight).toBe(1.0);
        expect(constraint.metadata.tolerance).toBe(1e-6);
    });

    it('should preserve custom options', () => {
        const constraint = asConstraintSpec<SimpleState>(
            (state) => state.value,
            {
                id: 'test',
                description: 'Test',
                severity: 'soft',
                threshold: 10,
                operator: 'le',
                penaltyWeight: 5.0,
                tolerance: 0.01,
            }
        );

        expect(constraint.metadata.penaltyWeight).toBe(5.0);
        expect(constraint.metadata.tolerance).toBe(0.01);
    });
});

// ==================== leConstraint ====================

describe('leConstraint', () => {
    it('should create less-than-or-equal constraint', () => {
        const constraint = leConstraint<SimpleState>(
            'max_value',
            (state) => state.value,
            100
        );

        expect(constraint.metadata.id).toBe('max_value');
        expect(constraint.metadata.operator).toBe('le');
        expect(constraint.metadata.threshold).toBe(100);
        expect(constraint.metadata.severity).toBe('hard');
    });

    it('should default to hard severity', () => {
        const constraint = leConstraint<SimpleState>(
            'test',
            (state) => state.value,
            100
        );
        expect(constraint.metadata.severity).toBe('hard');
    });

    it('should allow soft severity', () => {
        const constraint = leConstraint<SimpleState>(
            'test',
            (state) => state.value,
            100,
            'soft'
        );
        expect(constraint.metadata.severity).toBe('soft');
    });

    it('should support custom options', () => {
        const constraint = leConstraint<SimpleState>(
            'speed',
            (state) => state.value,
            20,
            'hard',
            { description: 'Max speed limit', unit: 'm/s' }
        );

        expect(constraint.metadata.description).toBe('Max speed limit');
        expect(constraint.metadata.unit).toBe('m/s');
    });
});

// ==================== geConstraint ====================

describe('geConstraint', () => {
    it('should create greater-than-or-equal constraint', () => {
        const constraint = geConstraint<SimpleState>(
            'min_value',
            (state) => state.value,
            0
        );

        expect(constraint.metadata.id).toBe('min_value');
        expect(constraint.metadata.operator).toBe('ge');
        expect(constraint.metadata.threshold).toBe(0);
    });

    it('should generate default description', () => {
        const constraint = geConstraint<SimpleState>(
            'distance',
            (state) => state.value,
            5
        );
        expect(constraint.metadata.description).toBe('distance >= 5');
    });
});

// ==================== eqConstraint ====================

describe('eqConstraint', () => {
    it('should create equality constraint', () => {
        const constraint = eqConstraint<SimpleState>(
            'target_value',
            (state) => state.value,
            100,
            5
        );

        expect(constraint.metadata.id).toBe('target_value');
        expect(constraint.metadata.operator).toBe('eq');
        expect(constraint.metadata.threshold).toBe(100);
        expect(constraint.metadata.tolerance).toBe(5);
    });

    it('should default tolerance to 1e-6', () => {
        const constraint = eqConstraint<SimpleState>(
            'test',
            (state) => state.value,
            50
        );
        expect(constraint.metadata.tolerance).toBe(1e-6);
    });

    it('should generate description with tolerance', () => {
        const constraint = eqConstraint<SimpleState>(
            'altitude',
            (state) => state.value,
            100,
            5
        );
        expect(constraint.metadata.description).toBe('altitude == 100 (±5)');
    });
});

// ==================== evaluateConstraint ====================

describe('evaluateConstraint', () => {
    describe('le constraint', () => {
        const constraint = leConstraint<SimpleState>(
            'max_value',
            (state) => state.value,
            100,
            'hard'
        );

        it('should be satisfied when value <= threshold', () => {
            const result = evaluateConstraint(constraint, { value: 50 });
            expect(result.satisfied).toBe(true);
            expect(result.violation).toBe(0);
            expect(result.penalty).toBe(0);
        });

        it('should be satisfied when value equals threshold', () => {
            const result = evaluateConstraint(constraint, { value: 100 });
            expect(result.satisfied).toBe(true);
        });

        it('should be violated when value > threshold', () => {
            const result = evaluateConstraint(constraint, { value: 120 });
            expect(result.satisfied).toBe(false);
            expect(result.violation).toBe(20);
        });
    });

    describe('ge constraint', () => {
        const constraint = geConstraint<SimpleState>(
            'min_value',
            (state) => state.value,
            10,
            'hard'
        );

        it('should be satisfied when value >= threshold', () => {
            const result = evaluateConstraint(constraint, { value: 50 });
            expect(result.satisfied).toBe(true);
            expect(result.violation).toBe(0);
        });

        it('should be violated when value < threshold', () => {
            const result = evaluateConstraint(constraint, { value: 5 });
            expect(result.satisfied).toBe(false);
            expect(result.violation).toBe(5);
        });
    });

    describe('eq constraint', () => {
        const constraint = eqConstraint<SimpleState>(
            'target',
            (state) => state.value,
            100,
            5,
            'hard'
        );

        it('should be satisfied when value within tolerance', () => {
            expect(evaluateConstraint(constraint, { value: 100 }).satisfied).toBe(true);
            expect(evaluateConstraint(constraint, { value: 103 }).satisfied).toBe(true);
            expect(evaluateConstraint(constraint, { value: 97 }).satisfied).toBe(true);
        });

        it('should be violated when value outside tolerance', () => {
            const result = evaluateConstraint(constraint, { value: 110 });
            expect(result.satisfied).toBe(false);
            expect(result.violation).toBe(10);
        });
    });

    describe('soft constraint penalty', () => {
        it('should calculate penalty for soft constraint violation', () => {
            const constraint = leConstraint<SimpleState>(
                'soft_limit',
                (state) => state.value,
                100,
                'soft',
                { penaltyWeight: 10 }
            );

            const result = evaluateConstraint(constraint, { value: 120 });
            expect(result.satisfied).toBe(false);
            expect(result.violation).toBe(20);
            expect(result.penalty).toBe(200); // 20 * 10
        });

        it('should not calculate penalty for hard constraint', () => {
            const constraint = leConstraint<SimpleState>(
                'hard_limit',
                (state) => state.value,
                100,
                'hard'
            );

            const result = evaluateConstraint(constraint, { value: 120 });
            expect(result.satisfied).toBe(false);
            expect(result.violation).toBe(20);
            expect(result.penalty).toBe(0);
        });
    });

    describe('result properties', () => {
        it('should include all result fields', () => {
            const constraint = leConstraint<SimpleState>(
                'test',
                (state) => state.value,
                50,
                'soft'
            );

            const result = evaluateConstraint(constraint, { value: 60 });

            expect(result).toMatchObject({
                id: 'test',
                value: 60,
                threshold: 50,
                operator: 'le',
                severity: 'soft',
                satisfied: false,
                violation: 10,
            });
        });
    });
});

// ==================== evaluateConstraints ====================

describe('evaluateConstraints', () => {
    const constraints = [
        leConstraint<SimpleState>('max', (s) => s.value, 100, 'hard'),
        geConstraint<SimpleState>('min', (s) => s.value, 10, 'hard'),
        leConstraint<SimpleState>('preferred', (s) => s.value, 80, 'soft', { penaltyWeight: 5 }),
    ];

    it('should evaluate all constraints', () => {
        const report = evaluateConstraints(constraints, { value: 50 });
        expect(report.results).toHaveLength(3);
    });

    it('should be feasible when all hard constraints satisfied', () => {
        const report = evaluateConstraints(constraints, { value: 50 });
        expect(report.feasible).toBe(true);
    });

    it('should be infeasible when hard constraint violated', () => {
        const report = evaluateConstraints(constraints, { value: 5 });
        expect(report.feasible).toBe(false);
    });

    it('should be feasible when only soft constraints violated', () => {
        const report = evaluateConstraints(constraints, { value: 90 });
        expect(report.feasible).toBe(true);
        expect(report.totalPenalty).toBe(50); // (90 - 80) * 5
    });

    it('should calculate total penalty from soft violations', () => {
        const report = evaluateConstraints(constraints, { value: 95 });
        expect(report.totalPenalty).toBe(75); // (95 - 80) * 5
    });

    it('should list all violations', () => {
        const report = evaluateConstraints(constraints, { value: 5 });
        expect(report.violations).toHaveLength(1);
        expect(report.violations[0].constraintId).toBe('min');
    });

    it('should include timestamp', () => {
        const before = Date.now();
        const report = evaluateConstraints(constraints, { value: 50 });
        const after = Date.now();

        expect(report.timestamp).toBeGreaterThanOrEqual(before);
        expect(report.timestamp).toBeLessThanOrEqual(after);
    });
});

// ==================== constraintsToPenalty ====================

describe('constraintsToPenalty', () => {
    const constraints = [
        leConstraint<SimpleState>('max', (s) => s.value, 100, 'hard'),
        leConstraint<SimpleState>('soft', (s) => s.value, 80, 'soft', { penaltyWeight: 5 }),
    ];

    it('should return 0 when all constraints satisfied', () => {
        const penalty = constraintsToPenalty(constraints, { value: 50 });
        expect(penalty).toBe(0);
    });

    it('should return soft penalty for soft violation', () => {
        const penalty = constraintsToPenalty(constraints, { value: 90 });
        expect(penalty).toBe(50); // (90 - 80) * 5
    });

    it('should return large penalty for hard violation', () => {
        const penalty = constraintsToPenalty(constraints, { value: 120 });
        // Hard: 20 * 1e6 = 2e7, Soft: 40 * 5 = 200
        expect(penalty).toBe(2e7 + 200);
    });

    it('should use custom hard penalty multiplier', () => {
        const penalty = constraintsToPenalty(constraints, { value: 120 }, 100);
        // Hard: 20 * 100 = 2000, Soft: 40 * 5 = 200
        expect(penalty).toBe(2200);
    });
});

