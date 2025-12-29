/**
 * @module sensing/utils
 * @description Utility functions for the Sensing module
 */

/**
 * Normalizes an angle to the range [-π, π]
 * 
 * @param angle - Angle in radians
 * @returns Normalized angle in radians
 */
export function normalizeAngle(angle: number): number {
    while (angle > Math.PI) angle -= 2 * Math.PI;
    while (angle < -Math.PI) angle += 2 * Math.PI;
    return angle;
}

/**
 * Generates a Gaussian random number using the Box-Muller transform
 * 
 * @returns Standard normal distribution (mean=0, std=1)
 */
export function gaussianRandom(): number {
    let u = 0, v = 0;
    while (u === 0) u = Math.random(); // Transform [0,1) to (0,1)
    while (v === 0) v = Math.random();
    return Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
}

/**
 * Calculates the Euclidean distance between two 2D points
 */
export function distance(p1: { x: number; y: number }, p2: { x: number; y: number }): number {
    return Math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2);
}

