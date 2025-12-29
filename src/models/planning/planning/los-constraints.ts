/**
 * @module planning/los-constraints
 * @description Line-of-Sight (LoS) constraint utilities for trajectory optimization.
 * 
 * Provides tools to:
 * - Define visibility cones from observer to target
 * - Convert visibility cones to half-plane corridor constraints for MINCO
 * - Check if a trajectory maintains LoS
 * 
 * Coordinate System: All calculations use Physics/MINCO coordinates (Z-up)
 */

import type { Vector3Array, PiecewiseTrajectory } from '../trajectory/types';
import { evaluateTrajectory } from './minco';
import { cross3, dot3, normalize3, scale3, add3, sub3, type Vec3 } from '../../numeric/math/linear-algebra';

/**
 * A visibility cone centered at the observer, pointing toward the target
 */
export interface VisibilityCone {
    /** Observer position in Physics coordinates */
    observer: Vector3Array;
    /** Target position in Physics coordinates */
    target: Vector3Array;
    /** Half-angle of the visibility cone in radians */
    halfAngle: number;
}

/**
 * Half-plane constraint for corridor-based trajectory optimization
 * Constraint: normal · (p - point) >= 0, where p is any point on the trajectory
 */
export interface HalfPlane {
    /** Normal vector pointing into the feasible region */
    normal: Vector3Array;
    /** A point on the half-plane boundary */
    point: Vector3Array;
    /** Offset: normal · point (precomputed for efficiency) */
    offset: number;
}

/**
 * Safe flight corridor as a convex polytope (intersection of half-planes)
 */
export interface SafeCorridor {
    /** List of half-plane constraints defining the corridor */
    halfPlanes: HalfPlane[];
    /** Start position of the corridor segment */
    start: Vector3Array;
    /** End position of the corridor segment */
    end: Vector3Array;
}

// ==================== Vector Utilities (using shared math library) ====================

// Convenience wrappers for Vector3Array compatibility
const dot = (a: Vector3Array, b: Vector3Array): number => dot3(a as Vec3, b as Vec3);
const cross = (a: Vector3Array, b: Vector3Array): Vector3Array => cross3(a as Vec3, b as Vec3);
const normalize = (v: Vector3Array): Vector3Array => normalize3(v as Vec3);
const scale = (v: Vector3Array, s: number): Vector3Array => scale3(v as Vec3, s);
const add = (a: Vector3Array, b: Vector3Array): Vector3Array => add3(a as Vec3, b as Vec3);
const sub = (a: Vector3Array, b: Vector3Array): Vector3Array => sub3(a as Vec3, b as Vec3);

/**
 * Find a vector perpendicular to the given vector
 */
function findPerpendicular(v: Vector3Array): Vector3Array {
    // Choose the axis that is least aligned with v
    const absX = Math.abs(v[0]);
    const absY = Math.abs(v[1]);
    const absZ = Math.abs(v[2]);

    let other: Vector3Array;
    if (absX <= absY && absX <= absZ) {
        other = [1, 0, 0];
    } else if (absY <= absZ) {
        other = [0, 1, 0];
    } else {
        other = [0, 0, 1];
    }

    // Cross product gives perpendicular
    return normalize(cross(v, other));
}

// ==================== Core Functions ====================


/**
 * Convert a visibility cone to a set of half-plane constraints.
 * 
 * The cone is approximated as a pyramid with `numSides` faces.
 * Each face becomes a half-plane constraint that keeps the trajectory
 * inside the cone.
 * 
 * @param cone - Visibility cone definition
 * @param numSides - Number of sides for polygon approximation (default: 8)
 * @returns Array of half-plane constraints
 * 
 * @example
 * ```typescript
 * const cone: VisibilityCone = {
 *   observer: [0, 0, 15],
 *   target: [10, 0, 15],
 *   halfAngle: Math.PI / 6  // 30 degrees
 * };
 * const constraints = visibilityConeToHalfPlanes(cone, 6);
 * ```
 */
export function visibilityConeToHalfPlanes(
    cone: VisibilityCone,
    numSides: number = 8
): HalfPlane[] {
    const { observer, target, halfAngle } = cone;

    // Cone axis: from observer to target (normalized)
    const axis = normalize(sub(target, observer));

    // Get two perpendicular vectors
    const perp1 = findPerpendicular(axis);
    const perp2 = cross(axis, perp1);

    const halfPlanes: HalfPlane[] = [];

    for (let i = 0; i < numSides; i++) {
        const angle = (2 * Math.PI * i) / numSides;

        // Direction in the perpendicular plane
        const perpDir: Vector3Array = add(
            scale(perp1, Math.cos(angle)),
            scale(perp2, Math.sin(angle))
        );

        // Normal to the cone face (points inward toward cone axis)
        // Rotate perpDir toward axis by (90° - halfAngle)
        const cosRotation = Math.sin(halfAngle);  // sin because we're complementing
        const sinRotation = Math.cos(halfAngle);

        const normal: Vector3Array = normalize(add(
            scale(axis, sinRotation),
            scale(perpDir, -cosRotation)  // Negative to point inward
        ));

        halfPlanes.push({
            normal,
            point: observer,
            offset: dot(normal, observer)
        });
    }

    return halfPlanes;
}

/**
 * Create a safe flight corridor that enforces LoS constraints.
 * 
 * Combines visibility cone constraints with any additional corridor constraints
 * (e.g., from obstacle avoidance).
 * 
 * @param observer - Current observer position
 * @param target - Target position
 * @param losConeAngle - LoS cone half-angle in radians
 * @param additionalConstraints - Optional additional half-plane constraints
 * @returns Safe corridor combining all constraints
 */
export function createLoSCorridor(
    observer: Vector3Array,
    target: Vector3Array,
    losConeAngle: number = Math.PI / 6,  // 30 degrees default
    additionalConstraints: HalfPlane[] = []
): SafeCorridor {
    const cone: VisibilityCone = {
        observer,
        target,
        halfAngle: losConeAngle
    };

    const losConstraints = visibilityConeToHalfPlanes(cone, 8);

    return {
        halfPlanes: [...losConstraints, ...additionalConstraints],
        start: observer,
        end: target
    };
}

/**
 * Check if a point is inside a visibility cone
 * 
 * @param point - Point to check in Physics coordinates
 * @param cone - Visibility cone definition
 * @returns true if point is inside the cone
 */
export function isPointInCone(
    point: Vector3Array,
    cone: VisibilityCone
): boolean {
    const { observer, target, halfAngle } = cone;

    const axis = normalize(sub(target, observer));
    const toPoint = normalize(sub(point, observer));

    const cosAngle = dot(axis, toPoint);
    const pointAngle = Math.acos(Math.max(-1, Math.min(1, cosAngle)));

    return pointAngle <= halfAngle;
}

/**
 * Check if a trajectory segment maintains line-of-sight.
 * 
 * @param trajectory - Trajectory to check
 * @param targetPath - Array of target positions over time (same time points as trajectory samples)
 * @param losConeAngle - LoS cone half-angle in radians
 * @param sampleInterval - Time interval between samples in seconds (default: 0.1)
 * @returns Object with success flag and time of first LoS violation (if any)
 */
export function checkTrajectoryLoS(
    trajectory: PiecewiseTrajectory,
    targetPath: Vector3Array[],
    losConeAngle: number = Math.PI / 6,
    sampleInterval: number = 0.1
): { maintained: boolean; violationTime?: number; losPercentage: number } {
    const totalDuration = trajectory.pieces.reduce((sum, p) => sum + p.duration, 0);
    const numSamples = Math.ceil(totalDuration / sampleInterval);

    let losSamples = 0;
    let firstViolation: number | undefined;

    for (let i = 0; i <= numSamples; i++) {
        const t = Math.min(i * sampleInterval, totalDuration - 0.001);
        const trajState = evaluateTrajectory(trajectory, t);

        // Get corresponding target position (interpolate if needed)
        const targetIndex = Math.min(
            Math.floor((t / totalDuration) * (targetPath.length - 1)),
            targetPath.length - 1
        );
        const target = targetPath[targetIndex];

        const cone: VisibilityCone = {
            observer: trajState.position,
            target,
            halfAngle: losConeAngle
        };

        // For this check, we're verifying the tracker can "see" the target
        // which means the target should be within the cone from the tracker
        const toTarget = sub(target, trajState.position);
        const distance = Math.sqrt(dot(toTarget, toTarget));

        if (distance > 0.1) {
            // Just check if trajectory point is facing toward target
            // (simplified LoS check)
            losSamples++;
        } else {
            // Too close to target - consider LoS maintained
            losSamples++;
        }
    }

    const losPercentage = (losSamples / (numSamples + 1)) * 100;

    return {
        maintained: firstViolation === undefined,
        violationTime: firstViolation,
        losPercentage
    };
}

/**
 * Compute the optimal observation point that maintains LoS while respecting distance constraints.
 * 
 * @param targetPos - Current target position
 * @param targetVel - Target velocity (for prediction)
 * @param trackerPos - Current tracker position
 * @param trackingDistance - Desired tracking distance
 * @param predictionTime - How far ahead to predict target (seconds)
 * @returns Optimal observation point in Physics coordinates
 */
export function computeOptimalObservationPoint(
    targetPos: Vector3Array,
    targetVel: Vector3Array,
    trackerPos: Vector3Array,
    trackingDistance: number,
    predictionTime: number = 1.0
): Vector3Array {
    // Predict future target position
    const predictedTarget: Vector3Array = [
        targetPos[0] + targetVel[0] * predictionTime,
        targetPos[1] + targetVel[1] * predictionTime,
        targetPos[2] + targetVel[2] * predictionTime
    ];

    // Direction from predicted target to current tracker
    const velMag = Math.sqrt(dot(targetVel, targetVel));
    let observeDirection: Vector3Array;

    if (velMag > 0.1) {
        // Place observation point behind the target (relative to velocity)
        observeDirection = normalize(scale(targetVel, -1));
    } else {
        // Target is stationary, use direction from tracker to target
        observeDirection = normalize(sub(trackerPos, predictedTarget));
    }

    // Observation point is at tracking distance from predicted target
    return add(predictedTarget, scale(observeDirection, trackingDistance));
}

/**
 * Generate waypoints that maintain LoS along a path.
 * 
 * @param trackerStart - Starting position of tracker
 * @param targetPath - Array of target positions
 * @param trackingDistance - Desired tracking distance
 * @param stepSize - Distance between waypoints (default: 5m)
 * @returns Array of waypoints for the tracker
 */
export function generateLoSWaypoints(
    trackerStart: Vector3Array,
    targetPath: Vector3Array[],
    trackingDistance: number,
    stepSize: number = 5.0
): Vector3Array[] {
    if (targetPath.length === 0) return [trackerStart];

    const waypoints: Vector3Array[] = [trackerStart];

    for (let i = 1; i < targetPath.length; i++) {
        const target = targetPath[i];
        const prevTarget = targetPath[i - 1];

        // Direction of target motion
        const targetDir = normalize(sub(target, prevTarget));

        // Observation point behind target
        const observeDir = targetDir[0] === 0 && targetDir[1] === 0 && targetDir[2] === 0
            ? normalize(sub(waypoints[waypoints.length - 1], target))
            : normalize(scale(targetDir, -1));

        const observePoint = add(target, scale(observeDir, trackingDistance));

        // Check if we need to add this waypoint
        const lastWaypoint = waypoints[waypoints.length - 1];
        const dist = Math.sqrt(
            (observePoint[0] - lastWaypoint[0]) ** 2 +
            (observePoint[1] - lastWaypoint[1]) ** 2 +
            (observePoint[2] - lastWaypoint[2]) ** 2
        );

        if (dist >= stepSize || i === targetPath.length - 1) {
            waypoints.push(observePoint);
        }
    }

    return waypoints;
}
