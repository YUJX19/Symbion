/**
 * Dynamics module utility functions
 */

/**
 * Normalize angle to [-π, π]
 */
export function normalizeAngle(angle: number): number {
    while (angle > Math.PI) angle -= 2 * Math.PI;
    while (angle < -Math.PI) angle += 2 * Math.PI;
    return angle;
}

/**
 * Generate Gaussian random number (Box-Muller transform)
 * Returns standard normal distribution (mean=0, std=1)
 */
export function gaussianRandom(): number {
    let u = 0, v = 0;
    while (u === 0) u = Math.random(); // Convert [0,1) to (0,1)
    while (v === 0) v = Math.random();
    return Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
}
