/**
 * @module src/isac/constraints/flightSpec
 * @description Flight constraint specifications for ISAC
 * 
 * These are ConstraintSpec factories that produce constraints compatible 
 * with the core Runner constraint evaluation system.
 */

import {
    leConstraint,
    geConstraint,
    type ConstraintSpec,
    type ConstraintSeverity,
} from '../../core/constraint';

// ==================== Types ====================

/**
 * Bounding box for no-fly zones
 */
export interface BoundingBox {
    min: number[];
    max: number[];
}

// ==================== Flight Constraint Specs ====================

/**
 * Maximum speed constraint
 * @param maxSpeed - Maximum allowed speed in m/s
 * @param severity - hard (must satisfy) or soft (penalized)
 */
export function maxSpeedConstraint<S extends { velocity: number[] }>(
    maxSpeed: number,
    severity: ConstraintSeverity = 'hard'
): ConstraintSpec<S> {
    return leConstraint(
        'max_speed',
        (state: S) => Math.sqrt(state.velocity.reduce((sum, v) => sum + v * v, 0)),
        maxSpeed,
        severity,
        { description: `Speed <= ${maxSpeed} m/s`, unit: 'm/s' }
    );
}

/**
 * Maximum acceleration constraint
 * @param maxAcc - Maximum allowed acceleration in m/s²
 * @param severity - hard (must satisfy) or soft (penalized)
 */
export function maxAccelerationConstraint<S extends { acceleration: number[] }>(
    maxAcc: number,
    severity: ConstraintSeverity = 'hard'
): ConstraintSpec<S> {
    return leConstraint(
        'max_acceleration',
        (state: S) => Math.sqrt(state.acceleration.reduce((sum, a) => sum + a * a, 0)),
        maxAcc,
        severity,
        { description: `Acceleration <= ${maxAcc} m/s²`, unit: 'm/s²' }
    );
}

/**
 * Altitude constraint (min and max)
 * @param minAlt - Minimum altitude in meters
 * @param maxAlt - Maximum altitude in meters
 * @param severity - hard (must satisfy) or soft (penalized)
 * @returns Array of two constraints: [minAltitude, maxAltitude]
 */
export function altitudeConstraint<S extends { position: number[] }>(
    minAlt: number,
    maxAlt: number,
    severity: ConstraintSeverity = 'hard'
): ConstraintSpec<S>[] {
    return [
        geConstraint(
            'min_altitude',
            (state: S) => state.position[2] ?? 0,
            minAlt,
            severity,
            { description: `Altitude >= ${minAlt} m`, unit: 'm' }
        ),
        leConstraint(
            'max_altitude',
            (state: S) => state.position[2] ?? 0,
            maxAlt,
            severity,
            { description: `Altitude <= ${maxAlt} m`, unit: 'm' }
        ),
    ];
}

/**
 * No-fly zone constraint (must stay outside the zone)
 * @param zone - Bounding box defining the no-fly zone
 * @param severity - hard (must satisfy) or soft (penalized)
 */
export function noFlyZoneConstraint<S extends { position: number[] }>(
    zone: BoundingBox,
    severity: ConstraintSeverity = 'hard'
): ConstraintSpec<S> {
    return geConstraint(
        'no_fly_zone',
        (state: S) => {
            const pos = state.position;
            // Signed distance: positive = outside, negative = inside
            const dx = Math.max(zone.min[0] - pos[0], 0, pos[0] - zone.max[0]);
            const dy = Math.max(zone.min[1] - pos[1], 0, pos[1] - zone.max[1]);
            const dz = Math.max(zone.min[2] - (pos[2] ?? 0), 0, (pos[2] ?? 0) - zone.max[2]);
            return Math.sqrt(dx * dx + dy * dy + dz * dz);
        },
        0,  // Must be outside (distance >= 0 when outside)
        severity,
        { description: 'Outside no-fly zone', unit: 'm' }
    );
}

/**
 * Energy budget constraint
 * @param budget - Maximum energy in Joules
 * @param severity - hard (must satisfy) or soft (penalized)
 */
export function energyBudgetConstraint<S extends { energyUsed: number }>(
    budget: number,
    severity: ConstraintSeverity = 'soft'
): ConstraintSpec<S> {
    return leConstraint(
        'energy_budget',
        (state: S) => state.energyUsed,
        budget,
        severity,
        { description: `Energy <= ${budget} J`, unit: 'J' }
    );
}
