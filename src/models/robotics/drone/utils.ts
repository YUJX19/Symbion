
import { Vector2, Pose } from './types'

/**
 * Normalize angle to [-π, π]
 */
export function normalizeAngle(angle: number): number {
    while (angle > Math.PI) angle -= 2 * Math.PI
    while (angle < -Math.PI) angle += 2 * Math.PI
    return angle
}

/**
 * Calculate Euclidean distance between two points
 */
export function distance(p1: Vector2, p2: Vector2): number {
    return Math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)
}

/**
 * Check if robot reached goal
 */
export function goalReached(currentPose: Pose, goal: Pose, threshold: number = 0.1): boolean {
    const dist = Math.sqrt((currentPose.x - goal.x) ** 2 + (currentPose.y - goal.y) ** 2)
    return dist < threshold
}

/**
 * Generate Gaussian random number (Box-Muller transform)
 * Returns standard normal distribution (mean=0, std=1)
 */
export function gaussianRandom(): number {
    let u = 0, v = 0
    while (u === 0) u = Math.random() // Converting [0,1) to (0,1)
    while (v === 0) v = Math.random()
    return Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v)
}
