/**
 * @module planning/minco-constraints
 * @description MINCO trajectory optimization constraints for aerial tracking.
 * 
 * Provides cost functions and gradients for:
 * - Tracking constraint: maintain distance to moving target
 * - Visibility constraint: stay within visible sector
 * - Landing constraint: converge to landing point
 * 
 */

import type { Vector3Array } from '../trajectory/types';
import type { VisibilityRegion, CostGradient } from './tracking-types';

// ==================== Vector Utilities ====================

/**
 * Compute Euclidean distance between two points
 */
function distance(a: Vector3Array, b: Vector3Array): number {
    const dx = a[0] - b[0];
    const dy = a[1] - b[1];
    const dz = a[2] - b[2];
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * Compute squared Euclidean distance
 */
function distanceSquared(a: Vector3Array, b: Vector3Array): number {
    const dx = a[0] - b[0];
    const dy = a[1] - b[1];
    const dz = a[2] - b[2];
    return dx * dx + dy * dy + dz * dz;
}

/**
 * Normalize angle to [-π, π]
 */
function normalizeAngle(angle: number): number {
    while (angle > Math.PI) angle -= 2 * Math.PI;
    while (angle < -Math.PI) angle += 2 * Math.PI;
    return angle;
}

// ==================== Tracking Constraint ====================

/**
 * Compute tracking cost and gradient.
 * 
 * Penalizes deviation from desired tracking distance to target.
 * Uses a quadratic penalty: cost = weight * (distance - desiredDist)²
 * 
 * @param position Current drone position
 * @param targetPosition Target position to track
 * @param desiredDistance Desired tracking distance
 * @param weight Penalty weight
 * @returns Cost value and gradient with respect to position
 * 
 * @example
 * ```typescript
 * const { cost, gradient } = computeTrackingCost(
 *   [10, 0, 2],      // Drone at (10, 0, 2)
 *   [5, 0, 0],       // Target at (5, 0, 0)
 *   5.0,             // Want to be 5m away
 *   1000.0           // Weight
 * );
 * // Use gradient for optimization update
 * ```
 */
export function computeTrackingCost(
    position: Vector3Array,
    targetPosition: Vector3Array,
    desiredDistance: number,
    weight: number
): CostGradient {
    const dx = position[0] - targetPosition[0];
    const dy = position[1] - targetPosition[1];
    const dz = position[2] - targetPosition[2];

    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
    const distError = dist - desiredDistance;

    // Quadratic cost
    const cost = weight * distError * distError;

    // Gradient: d/dp [ w * (||p - t|| - d)² ]
    //         = 2 * w * (||p - t|| - d) * (p - t) / ||p - t||
    if (dist < 1e-6) {
        return { cost, gradient: [0, 0, 0] };
    }

    const factor = 2.0 * weight * distError / dist;
    const gradient: Vector3Array = [
        factor * dx,
        factor * dy,
        factor * dz
    ];

    return { cost, gradient };
}

/**
 * Compute tracking cost with horizontal distance only.
 * 
 * Useful for ground vehicle tracking where vertical distance
 * should be handled separately (altitude maintenance).
 * 
 * @param position Current drone position
 * @param targetPosition Target position
 * @param desiredDistance Desired horizontal tracking distance
 * @param weight Penalty weight
 * @returns Cost and gradient (z-gradient is zero)
 */
export function computeHorizontalTrackingCost(
    position: Vector3Array,
    targetPosition: Vector3Array,
    desiredDistance: number,
    weight: number
): CostGradient {
    const dx = position[0] - targetPosition[0];
    const dy = position[1] - targetPosition[1];

    const dist = Math.sqrt(dx * dx + dy * dy);
    const distError = dist - desiredDistance;

    const cost = weight * distError * distError;

    if (dist < 1e-6) {
        return { cost, gradient: [0, 0, 0] };
    }

    const factor = 2.0 * weight * distError / dist;
    const gradient: Vector3Array = [
        factor * dx,
        factor * dy,
        0
    ];

    return { cost, gradient };
}

// ==================== Visibility Constraint ====================

/**
 * Compute visibility cost and gradient.
 * 
 * Penalizes being outside the visible sector. Uses a soft constraint
 * that activates when the angular deviation exceeds the half-angle.
 * 
 * Cost = weight * max(0, |θ - θ_center| - halfAngle)²
 * 
 * @param position Current drone position
 * @param region Visibility region (target, visible point, half-angle)
 * @param weight Penalty weight
 * @returns Cost and gradient
 * 
 * @example
 * ```typescript
 * const { cost, gradient } = computeVisibilityCost(
 *   dronePosition,
 *   { center: target, visiblePoint: observationPoint, halfAngle: Math.PI/4 },
 *   10000.0
 * );
 * ```
 */
export function computeVisibilityCost(
    position: Vector3Array,
    region: VisibilityRegion,
    weight: number
): CostGradient {
    // Vector from target to drone
    const dx = position[0] - region.center[0];
    const dy = position[1] - region.center[1];
    const theta = Math.atan2(dy, dx);

    // Vector from target to visible point (sector center)
    const vdx = region.visiblePoint[0] - region.center[0];
    const vdy = region.visiblePoint[1] - region.center[1];
    const thetaCenter = Math.atan2(vdy, vdx);

    // Angular deviation
    const angleDiff = normalizeAngle(theta - thetaCenter);
    const absAngleDiff = Math.abs(angleDiff);

    // Check if inside visible sector
    if (absAngleDiff <= region.halfAngle) {
        return { cost: 0, gradient: [0, 0, 0] };
    }

    // Violation amount
    const violation = absAngleDiff - region.halfAngle;
    const cost = weight * violation * violation;

    // Gradient computation
    // d(theta)/d(position) = d(atan2(dy, dx))/d(p)
    // = [-dy/(dx²+dy²), dx/(dx²+dy²), 0]
    const r2 = dx * dx + dy * dy;
    if (r2 < 1e-6) {
        return { cost, gradient: [0, 0, 0] };
    }

    const dThetaDx = -dy / r2;
    const dThetaDy = dx / r2;

    // Chain rule: d(cost)/d(p) = d(cost)/d(theta) * d(theta)/d(p)
    // d(cost)/d(theta) = 2 * weight * violation * sign(angleDiff)
    const sign = angleDiff > 0 ? 1 : -1;
    const dCostDTheta = 2 * weight * violation * sign;

    const gradient: Vector3Array = [
        dCostDTheta * dThetaDx,
        dCostDTheta * dThetaDy,
        0
    ];

    return { cost, gradient };
}

/**
 * Compute visibility cost with distance penalty.
 * 
 * Combines angular visibility constraint with distance maintenance.
 * Useful when both visibility and distance need to be enforced.
 * 
 * @param position Drone position
 * @param region Visibility region
 * @param visibilityWeight Weight for angular constraint
 * @param distanceWeight Weight for distance constraint
 * @returns Combined cost and gradient
 */
export function computeCombinedVisibilityCost(
    position: Vector3Array,
    region: VisibilityRegion,
    visibilityWeight: number,
    distanceWeight: number
): CostGradient {
    // Visibility (angular) cost
    const visCost = computeVisibilityCost(position, region, visibilityWeight);

    // Distance cost (penalize deviation from observation distance)
    const distCost = computeTrackingCost(
        position,
        region.center,
        region.distance,
        distanceWeight
    );

    return {
        cost: visCost.cost + distCost.cost,
        gradient: [
            visCost.gradient[0] + distCost.gradient[0],
            visCost.gradient[1] + distCost.gradient[1],
            visCost.gradient[2] + distCost.gradient[2]
        ]
    };
}

// ==================== Landing Constraint ====================

/**
 * Compute landing cost and gradient.
 * 
 * Penalizes deviation from the landing point. Uses a quadratic
 * penalty that attracts the trajectory toward the landing position.
 * 
 * @param position Drone position
 * @param landingPoint Target landing position
 * @param weight Penalty weight
 * @returns Cost and gradient
 */
export function computeLandingCost(
    position: Vector3Array,
    landingPoint: Vector3Array,
    weight: number
): CostGradient {
    const dx = position[0] - landingPoint[0];
    const dy = position[1] - landingPoint[1];
    const dz = position[2] - landingPoint[2];

    const distSq = dx * dx + dy * dy + dz * dz;
    const cost = weight * distSq;

    const gradient: Vector3Array = [
        2 * weight * dx,
        2 * weight * dy,
        2 * weight * dz
    ];

    return { cost, gradient };
}

/**
 * Compute landing cost with soft approach.
 * 
 * Uses a smooth transition function that increases penalty
 * as the drone gets closer to the landing point, encouraging
 * a gentle approach.
 * 
 * @param position Drone position
 * @param landingPoint Target landing position
 * @param approachDistance Distance at which full penalty applies
 * @param weight Penalty weight
 * @returns Cost and gradient
 */
export function computeSoftLandingCost(
    position: Vector3Array,
    landingPoint: Vector3Array,
    approachDistance: number,
    weight: number
): CostGradient {
    const dist = distance(position, landingPoint);

    // Outside approach zone: no penalty
    if (dist > approachDistance) {
        // Still pull toward landing point, but gently
        const pullFactor = 0.1;
        return computeLandingCost(position, landingPoint, weight * pullFactor);
    }

    // Inside approach zone: increasing penalty
    const normalizedDist = dist / approachDistance;
    const ramp = 1 - normalizedDist; // 0 at approach distance, 1 at landing point

    return computeLandingCost(position, landingPoint, weight * (0.1 + 0.9 * ramp));
}

// ==================== Corridor Constraint ====================

/**
 * Compute corridor constraint cost.
 * 
 * Penalizes being outside a convex polyhedron (H-representation).
 * Each face is defined by a normal vector n and a point p,
 * constraint: n · (x - p) ≤ 0
 * 
 * @param position Drone position
 * @param hPoly Polyhedron in H-representation (6 x numFaces matrix)
 *              Each column: [n_x, n_y, n_z, p_x, p_y, p_z]
 * @param weight Penalty weight
 * @returns Cost and gradient
 */
export function computeCorridorCost(
    position: Vector3Array,
    hPoly: number[][],  // Array of [nx, ny, nz, px, py, pz]
    weight: number
): CostGradient {
    let totalCost = 0;
    const totalGradient: Vector3Array = [0, 0, 0];

    for (const face of hPoly) {
        const nx = face[0], ny = face[1], nz = face[2];
        const px = face[3], py = face[4], pz = face[5];

        // Compute signed distance to plane
        // d = n · (position - p)
        const d = nx * (position[0] - px) +
            ny * (position[1] - py) +
            nz * (position[2] - pz);

        // Only penalize if outside (d > 0)
        if (d > 0) {
            totalCost += weight * d * d;

            // Gradient: 2 * weight * d * n
            totalGradient[0] += 2 * weight * d * nx;
            totalGradient[1] += 2 * weight * d * ny;
            totalGradient[2] += 2 * weight * d * nz;
        }
    }

    return { cost: totalCost, gradient: totalGradient };
}

// ==================== Dynamics Constraints ====================

/**
 * Compute velocity constraint cost.
 * 
 * Penalizes exceeding maximum velocity.
 * 
 * @param velocity Current velocity
 * @param maxVelocity Maximum allowed velocity
 * @param weight Penalty weight
 * @returns Cost and gradient (w.r.t. velocity)
 */
export function computeVelocityCost(
    velocity: Vector3Array,
    maxVelocity: number,
    weight: number
): CostGradient {
    const speed = Math.sqrt(
        velocity[0] * velocity[0] +
        velocity[1] * velocity[1] +
        velocity[2] * velocity[2]
    );

    if (speed <= maxVelocity) {
        return { cost: 0, gradient: [0, 0, 0] };
    }

    const violation = speed - maxVelocity;
    const cost = weight * violation * violation;

    // Gradient: 2 * weight * violation * v / ||v||
    const factor = 2 * weight * violation / speed;
    const gradient: Vector3Array = [
        factor * velocity[0],
        factor * velocity[1],
        factor * velocity[2]
    ];

    return { cost, gradient };
}

/**
 * Compute acceleration constraint cost.
 * 
 * Penalizes exceeding maximum acceleration.
 * 
 * @param acceleration Current acceleration
 * @param maxAcceleration Maximum allowed acceleration
 * @param weight Penalty weight
 * @returns Cost and gradient
 */
export function computeAccelerationCost(
    acceleration: Vector3Array,
    maxAcceleration: number,
    weight: number
): CostGradient {
    const accMag = Math.sqrt(
        acceleration[0] * acceleration[0] +
        acceleration[1] * acceleration[1] +
        acceleration[2] * acceleration[2]
    );

    if (accMag <= maxAcceleration) {
        return { cost: 0, gradient: [0, 0, 0] };
    }

    const violation = accMag - maxAcceleration;
    const cost = weight * violation * violation;

    const factor = 2 * weight * violation / accMag;
    const gradient: Vector3Array = [
        factor * acceleration[0],
        factor * acceleration[1],
        factor * acceleration[2]
    ];

    return { cost, gradient };
}

// ==================== Combined Cost Functions ====================

/**
 * Compute total tracking cost for a trajectory point.
 * 
 * Combines tracking distance, visibility, velocity, and acceleration
 * constraints into a single cost function for trajectory optimization.
 * 
 * @param state Current state {position, velocity, acceleration}
 * @param target Target position
 * @param region Visibility region (optional)
 * @param config Cost weights and limits
 * @returns Total cost and gradients
 */
export function computeTotalTrackingCost(
    state: {
        position: Vector3Array;
        velocity: Vector3Array;
        acceleration: Vector3Array;
    },
    target: Vector3Array,
    region: VisibilityRegion | null,
    config: {
        trackingDistance: number;
        trackingWeight: number;
        visibilityWeight: number;
        maxVelocity: number;
        velocityWeight: number;
        maxAcceleration: number;
        accelerationWeight: number;
    }
): {
    totalCost: number;
    positionGradient: Vector3Array;
    velocityGradient: Vector3Array;
    accelerationGradient: Vector3Array;
} {
    // Tracking distance cost
    const trackCost = computeHorizontalTrackingCost(
        state.position,
        target,
        config.trackingDistance,
        config.trackingWeight
    );

    // Visibility cost
    let visCost: CostGradient = { cost: 0, gradient: [0, 0, 0] };
    if (region) {
        visCost = computeVisibilityCost(state.position, region, config.visibilityWeight);
    }

    // Velocity cost
    const velCost = computeVelocityCost(
        state.velocity,
        config.maxVelocity,
        config.velocityWeight
    );

    // Acceleration cost
    const accCost = computeAccelerationCost(
        state.acceleration,
        config.maxAcceleration,
        config.accelerationWeight
    );

    return {
        totalCost: trackCost.cost + visCost.cost + velCost.cost + accCost.cost,
        positionGradient: [
            trackCost.gradient[0] + visCost.gradient[0],
            trackCost.gradient[1] + visCost.gradient[1],
            trackCost.gradient[2] + visCost.gradient[2]
        ],
        velocityGradient: velCost.gradient,
        accelerationGradient: accCost.gradient
    };
}
