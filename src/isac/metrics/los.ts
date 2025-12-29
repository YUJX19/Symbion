/**
 * @module src/isac/metrics/los
 * @description Line-of-Sight Metrics
 */

import type { Position3D } from '../planner';

/**
 * LoS probability based on ITU-R urban model
 */
export function calculateLosProbability(
    elevationAngleDeg: number,
    environment: 'urban' | 'suburban' | 'rural' = 'suburban'
): number {
    const params = {
        urban: { a: 9.61, b: 0.16 },
        suburban: { a: 4.88, b: 0.43 },
        rural: { a: 1.0, b: 1.0 },
    };

    const { a, b } = params[environment];
    const theta = Math.max(0, Math.min(90, elevationAngleDeg));

    return 1 / (1 + a * Math.exp(-b * (theta - a)));
}

/**
 * Calculate elevation angle between UAV and ground user
 */
export function calculateElevationAngle(
    uavPosition: Position3D,
    userPosition: Position3D
): number {
    const dx = userPosition.x - uavPosition.x;
    const dy = userPosition.y - uavPosition.y;
    const dz = uavPosition.z - userPosition.z;

    const horizontalDistance = Math.sqrt(dx * dx + dy * dy);

    if (horizontalDistance < 1e-6) {
        return 90; // Directly above
    }

    return Math.atan(dz / horizontalDistance) * 180 / Math.PI;
}

/**
 * Calculate average LoS percentage for multiple users
 */
export function calculateAverageLoS(
    uavPosition: Position3D,
    userPositions: Position3D[],
    environment: 'urban' | 'suburban' | 'rural' = 'suburban'
): number {
    if (userPositions.length === 0) return 0;

    let totalLosProb = 0;
    for (const userPos of userPositions) {
        const elevation = calculateElevationAngle(uavPosition, userPos);
        totalLosProb += calculateLosProbability(elevation, environment);
    }

    return totalLosProb / userPositions.length;
}
