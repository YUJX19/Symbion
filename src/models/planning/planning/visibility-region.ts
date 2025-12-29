/**
 * @module planning/visibility-region
 * @description Visibility region computation for aerial tracking.
 * 
 * Computes fan-shaped visibility regions around target positions,
 * representing the angular sectors from which the target is visible
 * (not blocked by obstacles).
 * 
 */

import type { Vector3Array } from '../trajectory/types';
import type {
    OccupancyGrid,
    VisibilityRegion,
    VisibilityRegionOptions
} from './tracking-types';
import { checkRayValid } from './visibility-path';

// ==================== Visibility Region Computation ====================

/**
 * Compute a single visibility pair: the center and angular extent of
 * the visible sector from a given observation position around a target.
 * 
 * The algorithm scans left and right from a seed angle to find the
 * boundary angles where visibility is blocked by obstacles.
 * 
 * @param center Target position (center of the observation circle)
 * @param seed Current observation point (will be updated)
 * @param observationDistance Desired observation distance
 * @param map Occupancy grid
 * @param options Computation options
 * @returns Visibility region with center point and half-angle
 */
export function computeVisibilityPair(
    center: Vector3Array,
    seed: Vector3Array,
    observationDistance: number,
    map: OccupancyGrid,
    options: VisibilityRegionOptions = {}
): { visiblePoint: Vector3Array; halfAngle: number; updatedSeed: Vector3Array } {

    const dTheta = options.angularResolution ?? 0.02; // ~1 degree
    const minClearance = options.minClearanceAngle ?? 0.1; // ~6 degrees

    // Calculate initial angle from center to seed
    const dx = seed[0] - center[0];
    const dy = seed[1] - center[1];
    const theta0 = Math.atan2(dy, dx);
    const seedZ = seed[2];

    // Scan left (decreasing angle) to find boundary
    let thetaLeft = theta0 - dTheta;
    while (thetaLeft > theta0 - Math.PI) {
        const testPoint: Vector3Array = [
            center[0] + observationDistance * Math.cos(thetaLeft),
            center[1] + observationDistance * Math.sin(thetaLeft),
            seedZ
        ];

        if (!checkRayValid(testPoint, center, map)) {
            thetaLeft += dTheta; // Back up one step
            break;
        }
        thetaLeft -= dTheta;
    }

    // Scan right (increasing angle) to find boundary
    let thetaRight = theta0 + dTheta;
    while (thetaRight < theta0 + Math.PI) {
        const testPoint: Vector3Array = [
            center[0] + observationDistance * Math.cos(thetaRight),
            center[1] + observationDistance * Math.sin(thetaRight),
            seedZ
        ];

        if (!checkRayValid(testPoint, center, map)) {
            thetaRight -= dTheta; // Back up one step
            break;
        }
        thetaRight += dTheta;
    }

    // Calculate center of visible sector
    const thetaCenter = (thetaLeft + thetaRight) / 2;
    const visiblePoint: Vector3Array = [
        center[0] + observationDistance * Math.cos(thetaCenter),
        center[1] + observationDistance * Math.sin(thetaCenter),
        seedZ
    ];

    // Half-angle of the sector
    const halfAngle = (thetaRight - thetaLeft) / 2;

    // Update seed for next iteration (maintain clearance from boundaries)
    let updatedSeed = [...seed] as Vector3Array;
    const clearanceAngle = Math.min(halfAngle, minClearance);

    if (theta0 - thetaLeft < clearanceAngle) {
        // Too close to left boundary, move seed right
        const newTheta = thetaLeft + clearanceAngle;
        updatedSeed = [
            center[0] + observationDistance * Math.cos(newTheta),
            center[1] + observationDistance * Math.sin(newTheta),
            seedZ
        ];
    } else if (thetaRight - theta0 < clearanceAngle) {
        // Too close to right boundary, move seed left
        const newTheta = thetaRight - clearanceAngle;
        updatedSeed = [
            center[0] + observationDistance * Math.cos(newTheta),
            center[1] + observationDistance * Math.sin(newTheta),
            seedZ
        ];
    }

    return {
        visiblePoint,
        halfAngle,
        updatedSeed
    };
}

/**
 * Compute visibility regions for a sequence of target positions.
 * 
 * For each target, computes a fan-shaped region representing the angles
 * from which the target is visible. Seeds are propagated between targets
 * to maintain trajectory continuity.
 * 
 * @param targetPositions Sequence of target positions
 * @param seedPoints Initial observation points for each target
 * @param map Occupancy grid
 * @param observationDistance Desired observation distance
 * @param options Computation options
 * @returns Array of visibility regions
 * 
 * @example
 * ```typescript
 * const regions = computeVisibilityRegions(
 *   [[5, 0, 0], [10, 0, 0]],  // Target positions
 *   [[5, 5, 2], [10, 5, 2]],  // Initial seed points
 *   occupancyGrid,
 *   5.0  // 5m observation distance
 * );
 * // regions[0].halfAngle gives the angular extent visible from first target
 * ```
 */
export function computeVisibilityRegions(
    targetPositions: Vector3Array[],
    seedPoints: Vector3Array[],
    map: OccupancyGrid,
    observationDistance: number,
    options: VisibilityRegionOptions = {}
): VisibilityRegion[] {

    if (targetPositions.length !== seedPoints.length) {
        throw new Error('Target positions and seed points must have the same length');
    }

    const regions: VisibilityRegion[] = [];
    const seeds = seedPoints.map(s => [...s] as Vector3Array);

    for (let i = 0; i < targetPositions.length; i++) {
        const result = computeVisibilityPair(
            targetPositions[i],
            seeds[i],
            observationDistance,
            map,
            options
        );

        regions.push({
            center: targetPositions[i],
            visiblePoint: result.visiblePoint,
            halfAngle: result.halfAngle,
            distance: observationDistance
        });

        // Propagate updated seed to next target
        if (i < targetPositions.length - 1) {
            seeds[i + 1] = propagateSeed(
                result.updatedSeed,
                targetPositions[i],
                targetPositions[i + 1],
                observationDistance
            );
        }
    }

    return regions;
}

/**
 * Propagate seed from current target to next target.
 * Maintains the relative angular position.
 */
function propagateSeed(
    currentSeed: Vector3Array,
    currentTarget: Vector3Array,
    nextTarget: Vector3Array,
    distance: number
): Vector3Array {
    // Calculate angle from current target to current seed
    const dx = currentSeed[0] - currentTarget[0];
    const dy = currentSeed[1] - currentTarget[1];
    const theta = Math.atan2(dy, dx);

    // Apply same angle to next target
    return [
        nextTarget[0] + distance * Math.cos(theta),
        nextTarget[1] + distance * Math.sin(theta),
        currentSeed[2]
    ];
}

// ==================== Visibility Constraint Checking ====================

/**
 * Check if a position is within a visibility region.
 * 
 * @param position Position to check
 * @param region Visibility region
 * @returns true if position is within the visible sector
 */
export function isInVisibilityRegion(
    position: Vector3Array,
    region: VisibilityRegion
): boolean {
    // Calculate angle from center to position
    const dx = position[0] - region.center[0];
    const dy = position[1] - region.center[1];
    const theta = Math.atan2(dy, dx);

    // Calculate angle from center to visible point (sector center)
    const vdx = region.visiblePoint[0] - region.center[0];
    const vdy = region.visiblePoint[1] - region.center[1];
    const thetaCenter = Math.atan2(vdy, vdx);

    // Check if angle is within sector
    let angleDiff = theta - thetaCenter;
    // Normalize to [-π, π]
    while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
    while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

    return Math.abs(angleDiff) <= region.halfAngle;
}

/**
 * Calculate the angular distance from a position to the visibility region boundary.
 * 
 * @param position Position to check
 * @param region Visibility region
 * @returns Signed angular distance (negative if inside, positive if outside)
 */
export function distanceToVisibilityBoundary(
    position: Vector3Array,
    region: VisibilityRegion
): number {
    const dx = position[0] - region.center[0];
    const dy = position[1] - region.center[1];
    const theta = Math.atan2(dy, dx);

    const vdx = region.visiblePoint[0] - region.center[0];
    const vdy = region.visiblePoint[1] - region.center[1];
    const thetaCenter = Math.atan2(vdy, vdx);

    let angleDiff = theta - thetaCenter;
    while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
    while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;

    // Distance to nearest boundary
    return Math.abs(angleDiff) - region.halfAngle;
}

// ==================== Full Visibility Check ====================

/**
 * Check if there is clear line of sight between two positions.
 * 
 * This is a convenience wrapper around checkRayValid for semantic clarity.
 * 
 * @param observerPos Observer position
 * @param targetPos Target position
 * @param map Occupancy grid
 * @returns true if target is visible from observer
 */
export function isTargetVisible(
    observerPos: Vector3Array,
    targetPos: Vector3Array,
    map: OccupancyGrid
): boolean {
    return checkRayValid(observerPos, targetPos, map);
}

/**
 * Find the best observation point on a circle around the target.
 * 
 * Searches for the point at `distance` from target that:
 * 1. Has clear line of sight to target
 * 2. Is closest to the preferred direction
 * 
 * @param targetPos Target position
 * @param distance Observation distance
 * @param preferredDirection Preferred observation direction [dx, dy]
 * @param height Observation height (Z coordinate)
 * @param map Occupancy grid
 * @returns Best observation point or null if none found
 */
export function findBestObservationPoint(
    targetPos: Vector3Array,
    distance: number,
    preferredDirection: [number, number],
    height: number,
    map: OccupancyGrid
): Vector3Array | null {
    const prefTheta = Math.atan2(preferredDirection[1], preferredDirection[0]);
    const dTheta = Math.PI / 36; // 5 degree increments

    // Search outward from preferred direction
    for (let offset = 0; offset <= Math.PI; offset += dTheta) {
        for (const sign of [1, -1]) {
            const theta = prefTheta + sign * offset;
            const testPoint: Vector3Array = [
                targetPos[0] + distance * Math.cos(theta),
                targetPos[1] + distance * Math.sin(theta),
                height
            ];

            if (checkRayValid(testPoint, targetPos, map)) {
                return testPoint;
            }
        }
    }

    return null;
}

// ==================== Visualization Helpers ====================

/**
 * Generate points along the arc of a visibility region for visualization.
 * 
 * @param region Visibility region
 * @param numPoints Number of points to generate
 * @returns Array of points along the visible arc
 */
export function generateVisibilityArc(
    region: VisibilityRegion,
    numPoints: number = 20
): Vector3Array[] {
    const points: Vector3Array[] = [];

    const vdx = region.visiblePoint[0] - region.center[0];
    const vdy = region.visiblePoint[1] - region.center[1];
    const thetaCenter = Math.atan2(vdy, vdx);
    const z = region.visiblePoint[2];

    for (let i = 0; i <= numPoints; i++) {
        const t = i / numPoints;
        const theta = thetaCenter - region.halfAngle + 2 * region.halfAngle * t;
        points.push([
            region.center[0] + region.distance * Math.cos(theta),
            region.center[1] + region.distance * Math.sin(theta),
            z
        ]);
    }

    return points;
}

/**
 * Generate a fan shape (triangle mesh vertices) for visualization.
 * 
 * @param region Visibility region
 * @param numSegments Number of segments in the fan
 * @returns Array of triangles (each 3 vertices)
 */
export function generateVisibilityFan(
    region: VisibilityRegion,
    numSegments: number = 10
): Vector3Array[][] {
    const triangles: Vector3Array[][] = [];

    const vdx = region.visiblePoint[0] - region.center[0];
    const vdy = region.visiblePoint[1] - region.center[1];
    const thetaCenter = Math.atan2(vdy, vdx);
    const z = region.visiblePoint[2];

    for (let i = 0; i < numSegments; i++) {
        const t1 = i / numSegments;
        const t2 = (i + 1) / numSegments;
        const theta1 = thetaCenter - region.halfAngle + 2 * region.halfAngle * t1;
        const theta2 = thetaCenter - region.halfAngle + 2 * region.halfAngle * t2;

        const p1: Vector3Array = [
            region.center[0] + region.distance * Math.cos(theta1),
            region.center[1] + region.distance * Math.sin(theta1),
            z
        ];
        const p2: Vector3Array = [
            region.center[0] + region.distance * Math.cos(theta2),
            region.center[1] + region.distance * Math.sin(theta2),
            z
        ];

        triangles.push([region.center, p1, p2]);
    }

    return triangles;
}
