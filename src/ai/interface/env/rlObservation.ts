/**
 * @module ai/env/rlObservation
 * @description UAV observation extraction utilities.
 *
 * Provides default and customizable observation extractors for
 * converting simulation state to AI-consumable observations.
 */

import type { Observation, Space, UavState, ObservationExtractor } from '../interface';

// ==================== Default Observation Space ====================

/**
 * Default UAV observation space definition
 *
 * Covers typical UAV communication scenarios:
 * - UAV position and velocity (6D)
 * - Battery level (1D)
 * - Channel SINR (1D)
 * - Data rate (1D)
 * - Block error rate (1D)
 * - Waypoint error (3D)
 * - Serving base station ID (discrete)
 */
export const DEFAULT_UAV_OBSERVATION_SPACE: Space = {
    kind: 'dict',
    spaces: {
        uav_pos: { kind: 'box', shape: [3], low: -10000, high: 10000 },
        uav_vel: { kind: 'box', shape: [3], low: -100, high: 100 },
        battery: { kind: 'box', shape: [1], low: 0, high: 1 },
        sinr_db: { kind: 'box', shape: [1], low: -20, high: 40 },
        rate_mbps: { kind: 'box', shape: [1], low: 0, high: 1000 },
        bler: { kind: 'box', shape: [1], low: 0, high: 1 },
        wp_error: { kind: 'box', shape: [3], low: -1000, high: 1000 },
        bs_id: { kind: 'discrete', n: 100 },
    },
};

// ==================== Default Observation Extractor ====================

/**
 * Default UAV observation extractor
 *
 * Converts a UavState object to a flat observation dictionary
 * suitable for RL training.
 *
 * @param state - UAV simulation state
 * @returns Observation dictionary
 *
 * @example
 * ```typescript
 * const state: UavState = {
 *   uav: { position: [100, 200, 50], velocity: [5, -2, 0], battery: 0.85 },
 *   channel: { sinrDb: 15.5, rate: 50.2, bler: 0.01 },
 *   mission: { target: [500, 500, 100], waypointError: [10, 5, 2] },
 *   network: { servingBsId: 3 },
 * };
 *
 * const obs = defaultUavObservation(state);
 * // obs = {
 * //   uav_pos: [100, 200, 50],
 * //   uav_vel: [5, -2, 0],
 * //   battery: 0.85,
 * //   sinr_db: 15.5,
 * //   rate_mbps: 50.2,
 * //   bler: 0.01,
 * //   wp_error: [10, 5, 2],
 * //   bs_id: 3,
 * // }
 * ```
 */
export function defaultUavObservation(state: UavState): Observation {
    const pos = state.uav.position ?? [0, 0, 0];
    const vel = state.uav.velocity ?? [0, 0, 0];

    return {
        uav_pos: pos,
        uav_vel: vel,
        battery: state.uav.battery ?? 1.0,
        sinr_db: state.channel.sinrDb ?? 0,
        rate_mbps: state.channel.rate ?? 0,
        bler: state.channel.bler ?? 0,
        wp_error: state.mission?.waypointError ?? [0, 0, 0],
        bs_id: state.network?.servingBsId ?? 0,
    };
}

// ==================== Extended Observation Extractors ====================

/**
 * Minimal observation extractor (position + SINR only)
 *
 * Useful for simple RL experiments with reduced state space.
 */
export function minimalUavObservation(state: UavState): Observation {
    return {
        pos: state.uav.position ?? [0, 0, 0],
        sinr: state.channel.sinrDb ?? 0,
    };
}

/**
 * Full observation extractor with all available fields
 *
 * Includes optional fields like attitude, constraints, and time.
 */
export function fullUavObservation(state: UavState): Observation {
    const pos = state.uav.position ?? [0, 0, 0];
    const vel = state.uav.velocity ?? [0, 0, 0];
    const acc = state.uav.acceleration ?? [0, 0, 0];

    const obs: Record<string, Observation> = {
        // UAV state
        uav_pos: pos,
        uav_vel: vel,
        uav_acc: acc,
        battery: state.uav.battery ?? 1.0,

        // Channel state
        sinr_db: state.channel.sinrDb ?? 0,
        rate_mbps: state.channel.rate ?? 0,
        bler: state.channel.bler ?? 0,
        path_loss: state.channel.pathLoss ?? 0,
        rician_k: state.channel.ricianK ?? 0,
    };

    // Attitude (if available)
    if (state.uav.attitude) {
        obs.roll = state.uav.attitude.roll;
        obs.pitch = state.uav.attitude.pitch;
        obs.yaw = state.uav.attitude.yaw;
    }

    // Mission state
    if (state.mission) {
        obs.target = state.mission.target;
        obs.wp_error = state.mission.waypointError ?? [0, 0, 0];
        obs.progress = state.mission.progress ?? 0;
    }

    // Network state
    if (state.network) {
        obs.bs_id = state.network.servingBsId;
        if (state.network.neighborBsIds) {
            obs.neighbor_bs = state.network.neighborBsIds;
        }
        obs.handover_pending = state.network.handoverPending ? 1 : 0;
    }

    // Constraints
    if (state.constraints) {
        obs.nfz_dist = state.constraints.noFlyZoneDistance ?? 1000;
        obs.acc_margin = state.constraints.maxAccelerationMargin ?? 1;
        obs.tilt_margin = state.constraints.maxTiltMargin ?? 1;
    }

    // Time
    if (state.time) {
        obs.step = state.time.step;
        obs.sim_time = state.time.simTime;
    }

    return obs;
}

// ==================== Observation Builder ====================

/**
 * Observation builder for creating custom extractors
 *
 * @example
 * ```typescript
 * const extractor = new ObservationBuilder<MyState>()
 *   .add('pos', s => s.uav.position)
 *   .add('vel', s => s.uav.velocity)
 *   .add('sinr', s => s.channel.sinrDb)
 *   .add('custom', s => computeCustomFeature(s))
 *   .build();
 *
 * const obs = extractor(state);
 * ```
 */
export class ObservationBuilder<S> {
    private extractors: {
        key: string;
        fn: (state: S) => number | number[];
    }[] = [];

    /**
     * Add an observation field
     *
     * @param key - Field name in output observation
     * @param extractor - Function to extract value from state
     */
    add(key: string, extractor: (state: S) => number | number[]): this {
        this.extractors.push({ key, fn: extractor });
        return this;
    }

    /**
     * Build the observation extractor function
     */
    build(): ObservationExtractor<S> {
        const extractors = [...this.extractors];
        return (state: S): Observation => {
            const obs: Record<string, number | number[]> = {};
            for (const { key, fn } of extractors) {
                obs[key] = fn(state);
            }
            return obs;
        };
    }
}

// ==================== Normalization Utilities ====================

/**
 * Normalize observation values to [-1, 1] or [0, 1] range
 *
 * @param obs - Observation to normalize
 * @param space - Observation space with bounds
 * @returns Normalized observation
 */
export function normalizeObservation(
    obs: Observation,
    space: Space
): Observation {
    if (typeof obs === 'number') {
        return normalizeScalar(obs, space);
    }

    if (Array.isArray(obs)) {
        if (space.kind !== 'box') return obs;
        return normalizeArray(obs, space);
    }

    if (typeof obs === 'object' && obs !== null) {
        if (space.kind !== 'dict') return obs;
        const result: Record<string, Observation> = {};
        for (const [key, value] of Object.entries(obs)) {
            const subSpace = space.spaces[key];
            result[key] = subSpace
                ? normalizeObservation(value as Observation, subSpace)
                : value;
        }
        return result;
    }

    return obs;
}

/**
 * Normalize a scalar value to [-1, 1] or [0, 1] range based on space type.
 *
 * - For discrete spaces: normalizes to [0, 1] using value / (n - 1)
 * - For box spaces: normalizes to [-1, 1] using linear interpolation
 *
 * @param value - The scalar value to normalize
 * @param space - The space definition containing bounds
 * @returns Normalized value, or original value if space kind is unsupported
 */
function normalizeScalar(value: number, space: Space): number {
    if (space.kind !== 'box' && space.kind !== 'discrete') return value;

    if (space.kind === 'discrete') {
        // Normalize discrete to [0, 1]
        return value / (space.n - 1);
    }

    const low = typeof space.low === 'number' ? space.low : space.low[0];
    const high = typeof space.high === 'number' ? space.high : space.high[0];

    if (!Number.isFinite(low) || !Number.isFinite(high)) return value;

    // Normalize to [-1, 1]
    return ((value - low) / (high - low)) * 2 - 1;
}

/**
 * Normalize an array of values to [-1, 1] range based on box space bounds.
 *
 * For each element, applies linear normalization: ((v - low) / (high - low)) * 2 - 1
 * Handles both scalar bounds and per-dimension bounds arrays.
 *
 * @param arr - The array of values to normalize
 * @param space - Box space with low/high bounds
 * @returns Array of normalized values
 */
function normalizeArray(arr: number[], space: { kind: 'box'; low: number | number[]; high: number | number[] }): number[] {
    return arr.map((v, i) => {
        const low = typeof space.low === 'number' ? space.low : (space.low[i] ?? 0);
        const high = typeof space.high === 'number' ? space.high : (space.high[i] ?? 1);

        if (!Number.isFinite(low) || !Number.isFinite(high)) return v;

        return ((v - low) / (high - low)) * 2 - 1;
    });
}
