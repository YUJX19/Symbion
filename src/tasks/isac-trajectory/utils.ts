/**
 * @module tasks/isac-trajectory/task1/utils
 * @description Trajectory sampling and validation utilities for ISAC optimization
 * 
 * Provides utilities for:
 * - Trajectory sampling with full kinematic state
 * - Distance calculations
 * - Target path interpolation
 * 
 * These utilities are used by objective and constraint modules for evaluation.
 */

import type { PiecewiseTrajectory } from '../../models/planning/trajectory/types';
import { evaluateTrajectory } from '../../models/planning/planning/minco';
import type { TrajectorySample } from './types';

// Re-export TrajectorySample for convenience
export type { TrajectorySample } from './types';

/**
 * Sample a piecewise trajectory at fixed time intervals.
 * Uses Symbion's evaluateTrajectory for consistent polynomial evaluation.
 * 
 * @param traj - Piecewise polynomial trajectory from MINCO
 * @param dt - Sample interval in seconds
 * @returns Array of kinematic samples
 */
export function sampleTrajectory(traj: PiecewiseTrajectory, dt: number): TrajectorySample[] {
    const samples: TrajectorySample[] = [];
    const totalDuration = traj.pieces.reduce((sum, p) => sum + p.duration, 0);

    for (let t = 0; t <= totalDuration; t += dt) {
        const state = evaluateTrajectory(traj, Math.min(t, totalDuration - 1e-6));
        const speed = Math.sqrt(
            state.velocity[0] ** 2 + state.velocity[1] ** 2 + state.velocity[2] ** 2
        );
        const accMag = Math.sqrt(
            state.acceleration[0] ** 2 + state.acceleration[1] ** 2 + state.acceleration[2] ** 2
        );

        samples.push({
            t,
            position: state.position as [number, number, number],
            velocity: state.velocity as [number, number, number],
            acceleration: state.acceleration as [number, number, number],
            speed,
            accMagnitude: accMag
        });
    }

    return samples;
}

/** Euclidean distance between two 3D points */
export function distance3D(a: [number, number, number], b: [number, number, number]): number {
    return Math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2);
}

/** Linear interpolation of target path at time t */
export function getTargetAtTime(
    targetPath: [number, number, number][],
    t: number,
    totalTime: number
): [number, number, number] {
    const ratio = Math.max(0, Math.min(1, t / totalTime));
    const idx = ratio * (targetPath.length - 1);
    const i0 = Math.floor(idx);
    const i1 = Math.min(i0 + 1, targetPath.length - 1);
    const alpha = idx - i0;

    return [
        targetPath[i0][0] * (1 - alpha) + targetPath[i1][0] * alpha,
        targetPath[i0][1] * (1 - alpha) + targetPath[i1][1] * alpha,
        targetPath[i0][2] * (1 - alpha) + targetPath[i1][2] * alpha
    ];
}
