/**
 * @module src/isac/constraints/communicationSpec
 * @description Communication constraint specifications for ISAC
 * 
 * These are ConstraintSpec factories that produce constraints compatible 
 * with the core Runner constraint evaluation system.
 */

import {
    geConstraint,
    type ConstraintSpec,
    type ConstraintSeverity,
    type ConstraintEvaluator,
} from '../../core/constraint';

// ==================== Communication Constraint Specs ====================

/**
 * Minimum SINR constraint
 * @param minSinr - Minimum required SINR in dB
 * @param severity - hard (must satisfy) or soft (penalized)
 */
export function minSinrConstraint<S extends { sinrDb: number }>(
    minSinr: number,
    severity: ConstraintSeverity = 'soft'
): ConstraintSpec<S> {
    return geConstraint(
        'min_sinr',
        (state: S) => state.sinrDb,
        minSinr,
        severity,
        { description: `SINR >= ${minSinr} dB`, unit: 'dB' }
    );
}

/**
 * Minimum distance constraint (e.g., obstacle avoidance)
 * @param id - Unique identifier for this constraint
 * @param distanceEvaluator - Function to evaluate distance from state
 * @param minDist - Minimum required distance in meters
 * @param severity - hard (must satisfy) or soft (penalized)
 */
export function minDistanceConstraint<S>(
    id: string,
    distanceEvaluator: ConstraintEvaluator<S>,
    minDist: number,
    severity: ConstraintSeverity = 'hard'
): ConstraintSpec<S> {
    return geConstraint(
        id,
        distanceEvaluator,
        minDist,
        severity,
        { description: `Distance >= ${minDist} m`, unit: 'm' }
    );
}
