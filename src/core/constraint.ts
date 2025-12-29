/**
 * @module core/constraint
 * @description Unified constraint abstraction for hard/soft constraints
 *
 * Provides ConstraintSpec for explicit constraint handling, separating feasibility
 * conditions from rewards to improve comparability and reproducibility.
 * 
 * This module provides generic constraint abstractions. Domain-specific constraints
 * (UAV, communication, etc.) are located in the isac/constraints module.
 */

import type { ConstraintViolationDetail } from './errors';

// ==================== Types ====================

/**
 * Constraint severity level
 * - hard: Must be satisfied (violation = infeasible)
 * - soft: Penalized but allowed (violation = penalty)
 */
export type ConstraintSeverity = 'hard' | 'soft';

/**
 * Constraint evaluation direction
 * - 'le': value <= threshold (e.g., max speed)
 * - 'ge': value >= threshold (e.g., min distance)
 * - 'eq': value == threshold (within tolerance)
 */
export type ConstraintOperator = 'le' | 'ge' | 'eq';

/**
 * Metadata for a constraint specification
 */
export interface ConstraintMetadata {
    /** Unique identifier for the constraint */
    id: string;
    /** Human-readable description */
    description: string;
    /** Severity level */
    severity: ConstraintSeverity;
    /** Threshold value */
    threshold: number;
    /** Comparison operator */
    operator: ConstraintOperator;
    /** Unit of measurement */
    unit?: string;
    /** Tolerance for equality constraints */
    tolerance?: number;
    /** Penalty weight for soft constraints */
    penaltyWeight?: number;
}

/**
 * Constraint evaluation function
 * @param state - Current state to evaluate
 * @returns The constraint value to be compared against threshold
 */
export type ConstraintEvaluator<S = unknown> = (state: S) => number;

/**
 * ConstraintSpec: Constraint with metadata for evaluation and reporting
 */
export interface ConstraintSpec<S = unknown> {
    /** Constraint metadata */
    metadata: ConstraintMetadata;
    /** Evaluation function */
    evaluate: ConstraintEvaluator<S>;
}

/**
 * Result of evaluating a single constraint
 */
export interface ConstraintResult {
    id: string;
    value: number;
    threshold: number;
    operator: ConstraintOperator;
    severity: ConstraintSeverity;
    satisfied: boolean;
    violation: number;  // How much the constraint is violated (0 if satisfied)
    penalty: number;    // Penalty value (0 if satisfied or hard constraint)
}

/**
 * Report of all constraint evaluations
 */
export interface ConstraintReport {
    /** All constraint results */
    results: ConstraintResult[];
    /** Whether all hard constraints are satisfied */
    feasible: boolean;
    /** Total penalty from soft constraint violations */
    totalPenalty: number;
    /** List of violated constraints */
    violations: ConstraintViolationDetail[];
    /** Timestamp of evaluation */
    timestamp: number;
}

// ==================== Factory Functions ====================

/**
 * Create a ConstraintSpec from an evaluation function and metadata
 */
export function asConstraintSpec<S = unknown>(
    evaluate: ConstraintEvaluator<S>,
    metadata: ConstraintMetadata
): ConstraintSpec<S> {
    return {
        metadata: {
            penaltyWeight: 1.0,
            tolerance: 1e-6,
            ...metadata,
        },
        evaluate,
    };
}

/**
 * Create a "less than or equal" constraint (value <= threshold)
 */
export function leConstraint<S>(
    id: string,
    evaluate: ConstraintEvaluator<S>,
    threshold: number,
    severity: ConstraintSeverity = 'hard',
    options: Partial<Omit<ConstraintMetadata, 'id' | 'threshold' | 'operator' | 'severity'>> = {}
): ConstraintSpec<S> {
    return asConstraintSpec(evaluate, {
        id,
        description: options.description ?? `${id} <= ${threshold}`,
        severity,
        threshold,
        operator: 'le',
        ...options,
    });
}

/**
 * Create a "greater than or equal" constraint (value >= threshold)
 */
export function geConstraint<S>(
    id: string,
    evaluate: ConstraintEvaluator<S>,
    threshold: number,
    severity: ConstraintSeverity = 'hard',
    options: Partial<Omit<ConstraintMetadata, 'id' | 'threshold' | 'operator' | 'severity'>> = {}
): ConstraintSpec<S> {
    return asConstraintSpec(evaluate, {
        id,
        description: options.description ?? `${id} >= ${threshold}`,
        severity,
        threshold,
        operator: 'ge',
        ...options,
    });
}

/**
 * Create an equality constraint (value == threshold within tolerance)
 */
export function eqConstraint<S>(
    id: string,
    evaluate: ConstraintEvaluator<S>,
    threshold: number,
    tolerance: number = 1e-6,
    severity: ConstraintSeverity = 'hard',
    options: Partial<Omit<ConstraintMetadata, 'id' | 'threshold' | 'operator' | 'severity' | 'tolerance'>> = {}
): ConstraintSpec<S> {
    return asConstraintSpec(evaluate, {
        id,
        description: options.description ?? `${id} == ${threshold} (±${tolerance})`,
        severity,
        threshold,
        operator: 'eq',
        tolerance,
        ...options,
    });
}

// ==================== Constraint Evaluation ====================

/**
 * Evaluate a single constraint
 */
export function evaluateConstraint<S>(
    constraint: ConstraintSpec<S>,
    state: S
): ConstraintResult {
    const { metadata } = constraint;
    const value = constraint.evaluate(state);
    const { threshold, operator, severity, tolerance = 1e-6, penaltyWeight = 1.0 } = metadata;

    let satisfied: boolean;
    let violation: number;

    switch (operator) {
        case 'le':
            satisfied = value <= threshold;
            violation = satisfied ? 0 : value - threshold;
            break;
        case 'ge':
            satisfied = value >= threshold;
            violation = satisfied ? 0 : threshold - value;
            break;
        case 'eq':
            satisfied = Math.abs(value - threshold) <= tolerance;
            violation = satisfied ? 0 : Math.abs(value - threshold);
            break;
    }

    // Calculate penalty (only for soft constraints)
    const penalty = severity === 'soft' && !satisfied
        ? violation * penaltyWeight
        : 0;

    return {
        id: metadata.id,
        value,
        threshold,
        operator,
        severity,
        satisfied,
        violation,
        penalty,
    };
}

/**
 * Evaluate multiple constraints and generate a report
 */
export function evaluateConstraints<S>(
    constraints: ConstraintSpec<S>[],
    state: S
): ConstraintReport {
    const results = constraints.map(c => evaluateConstraint(c, state));

    const feasible = results
        .filter(r => r.severity === 'hard')
        .every(r => r.satisfied);

    const totalPenalty = results
        .filter(r => r.severity === 'soft')
        .reduce((sum, r) => sum + r.penalty, 0);

    const violations: ConstraintViolationDetail[] = results
        .filter(r => !r.satisfied)
        .map(r => ({
            constraintId: r.id,
            value: r.value,
            threshold: r.threshold,
            severity: r.severity,
        }));

    return {
        results,
        feasible,
        totalPenalty,
        violations,
        timestamp: Date.now(),
    };
}

/**
 * Convert constraint violations to a penalty value for optimization
 */
export function constraintsToPenalty<S>(
    constraints: ConstraintSpec<S>[],
    state: S,
    hardPenaltyMultiplier: number = 1e6
): number {
    const report = evaluateConstraints(constraints, state);

    // Sum of soft penalties plus large penalty for hard violations
    const hardPenalty = report.results
        .filter(r => r.severity === 'hard' && !r.satisfied)
        .reduce((sum, r) => sum + r.violation * hardPenaltyMultiplier, 0);

    return report.totalPenalty + hardPenalty;
}


