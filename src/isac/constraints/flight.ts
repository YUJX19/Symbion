/**
 * @module src/isac/constraints/flight
 * @description Flight Constraints for ISAC
 */

import type { Position3D } from '../planner';

/**
 * Flight constraints configuration
 */
export interface FlightConstraints {
    minAltitude: number;
    maxAltitude: number;
    maxSpeed: number;
    maxAcceleration: number;
    areaBounds: [number, number, number, number]; // [minX, minY, maxX, maxY]
}

/**
 * Check if position is within altitude bounds
 */
export function checkAltitude(
    position: Position3D,
    constraints: FlightConstraints
): boolean {
    return position.z >= constraints.minAltitude &&
        position.z <= constraints.maxAltitude;
}

/**
 * Check if position is within area bounds
 */
export function checkAreaBounds(
    position: Position3D,
    constraints: FlightConstraints
): boolean {
    const [minX, minY, maxX, maxY] = constraints.areaBounds;
    return position.x >= minX && position.x <= maxX &&
        position.y >= minY && position.y <= maxY;
}

/**
 * Check if speed is within limits
 */
export function checkSpeed(
    speed: number,
    constraints: FlightConstraints
): boolean {
    return speed >= 0 && speed <= constraints.maxSpeed;
}

/**
 * Clamp position to valid bounds
 */
export function clampPosition(
    position: Position3D,
    constraints: FlightConstraints
): Position3D {
    const [minX, minY, maxX, maxY] = constraints.areaBounds;
    return {
        x: Math.max(minX, Math.min(maxX, position.x)),
        y: Math.max(minY, Math.min(maxY, position.y)),
        z: Math.max(constraints.minAltitude, Math.min(constraints.maxAltitude, position.z)),
    };
}

/**
 * Validate all flight constraints
 */
export function validateFlightConstraints(
    position: Position3D,
    speed: number,
    constraints: FlightConstraints
): { valid: boolean; violations: string[] } {
    const violations: string[] = [];

    if (!checkAltitude(position, constraints)) {
        violations.push('altitude_violation');
    }
    if (!checkAreaBounds(position, constraints)) {
        violations.push('area_violation');
    }
    if (!checkSpeed(speed, constraints)) {
        violations.push('speed_violation');
    }

    return { valid: violations.length === 0, violations };
}
