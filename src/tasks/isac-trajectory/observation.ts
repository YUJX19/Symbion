/**
 * @module tasks/isac-trajectory/input
 * @description Input extraction for ISAC trajectory task
 */

import type { ScenarioState, UserChannelState } from './scenario';
import type { IsacTrajectoryConfig, Position3D } from './config';

// ==================== Types ====================

/**
 * Full input state for ISAC trajectory task
 */
export interface IsacInput {
    /** Normalized UAV position (x, y, z) in [-1, 1] */
    normalizedPosition: number[];
    /** Normalized UAV velocity */
    normalizedVelocity: number[];
    /** LoS status per user (0 = NLoS, 1 = LoS) */
    losStatus: number[];
    /** Normalized SINR per user */
    normalizedSinr: number[];
    /** Normalized distance to each user */
    normalizedDistances: number[];
    /** Overall LoS percentage */
    losPercentage: number;
    /** URLLC satisfaction status */
    urllcSatisfied: number[];
    /** Normalized energy used */
    normalizedEnergy: number;
    /** Normalized step progress */
    progress: number;
}

// ==================== Input Functions ====================

/**
 * Extract input from scenario state
 */
export function extractInput(
    state: ScenarioState,
    config: IsacTrajectoryConfig
): IsacInput {
    const [minX, minY, maxX, maxY] = config.areaBounds;
    const { minAltitude, maxAltitude, maxSpeed } = config.kinematics;

    // Normalize position to [-1, 1]
    const normalizedPosition = [
        2 * (state.uav.position.x - minX) / (maxX - minX) - 1,
        2 * (state.uav.position.y - minY) / (maxY - minY) - 1,
        2 * (state.uav.position.z - minAltitude) / (maxAltitude - minAltitude) - 1,
    ];

    // Normalize velocity
    const normalizedVelocity = [
        state.uav.velocity.x / maxSpeed,
        state.uav.velocity.y / maxSpeed,
        state.uav.velocity.z / maxSpeed,
    ];

    // LoS status (binary)
    const losStatus = state.userChannels.map(c => c.isLos ? 1 : 0);

    // Normalize SINR (assume range -10 to 40 dB)
    const normalizedSinr = state.userChannels.map(c =>
        (c.sinrDb + 10) / 50 // Map -10..40 to 0..1
    );

    // Normalize distances
    const maxDistance = Math.sqrt(
        (maxX - minX) ** 2 + (maxY - minY) ** 2 + maxAltitude ** 2
    );
    const normalizedDistances = state.userChannels.map(c =>
        c.distance / maxDistance
    );

    // URLLC satisfaction
    const urllcSatisfied = state.urllcSatisfied.map(s => s ? 1 : 0);

    // Normalized energy (assume max 10000 J per episode)
    const maxEnergy = 10000;
    const normalizedEnergy = Math.min(1, state.uav.energyUsed / maxEnergy);

    // Progress
    const progress = state.step / config.episodeHorizon;

    return {
        normalizedPosition,
        normalizedVelocity,
        losStatus,
        normalizedSinr,
        normalizedDistances,
        losPercentage: state.losPercentage,
        urllcSatisfied,
        normalizedEnergy,
        progress,
    };
}

/**
 * Convert input to flat vector
 */
export function inputToVector(input: IsacInput): number[] {
    return [
        ...input.normalizedPosition,
        ...input.normalizedVelocity,
        ...input.losStatus,
        ...input.normalizedSinr,
        ...input.normalizedDistances,
        input.losPercentage,
        ...input.urllcSatisfied,
        input.normalizedEnergy,
        input.progress,
    ];
}

/**
 * Get input space dimension based on config
 */
export function getInputDimension(config: IsacTrajectoryConfig): number {
    const numUsers = config.users.length;
    // 3 (pos) + 3 (vel) + numUsers (los) + numUsers (sinr) + numUsers (dist)
    // + 1 (losPercent) + numUsers (urllc) + 1 (energy) + 1 (progress)
    return 3 + 3 + numUsers * 4 + 3;
}

/**
 * Get input space definition
 */
export function getInputSpace(config: IsacTrajectoryConfig) {
    const dim = getInputDimension(config);
    return {
        kind: 'box' as const,
        shape: [dim],
        low: -1,
        high: 1,
        dtype: 'float32' as const,
    };
}

/**
 * Get output space definition
 *
 * Output: 3D position offset (continuous) or waypoint index (discrete)
 */
export function getOutputSpace(config: IsacTrajectoryConfig, continuous: boolean = true) {
    if (continuous) {
        // Continuous: (dx, dy, dz) normalized
        return {
            kind: 'box' as const,
            shape: [3],
            low: -1,
            high: 1,
            dtype: 'float32' as const,
        };
    }
    // Discrete: predefined waypoint directions (26 directions + hover)
    return {
        kind: 'discrete' as const,
        n: 27,
    };
}

/**
 * Convert discrete output to position offset
 */
export function discreteOutputToOffset(output: number, stepSize: number = 20): Position3D {
    // 27 actions: 3x3x3 grid (including center = hover)
    const x = (output % 3) - 1; // -1, 0, 1
    const y = Math.floor((output / 3) % 3) - 1;
    const z = Math.floor(output / 9) - 1;

    return {
        x: x * stepSize,
        y: y * stepSize,
        z: z * stepSize,
    };
}

/**
 * Convert continuous output to position offset
 */
export function continuousOutputToOffset(
    output: number[],
    config: IsacTrajectoryConfig
): Position3D {
    const maxMove = config.kinematics.maxSpeed * config.waypointDurationS;
    return {
        x: output[0] * maxMove,
        y: output[1] * maxMove,
        z: output[2] * maxMove * 0.5, // Limit vertical movement
    };
}
