/**
 * @module ai/env/rlAction
 * @description UAV action mapping utilities.
 *
 * Provides default and customizable action mappers for
 * converting AI outputs to simulation commands.
 */

import { getMcsConfig, type Action, type Space, type UavCommand, type ActionMapper } from '../interface';

// ==================== Default Action Space ====================

/**
 * Default UAV action space definition
 *
 * Covers typical UAV communication control:
 * - Flight velocity delta (3D continuous, normalized [-1, 1])
 * - Transmission power (1D continuous, normalized [0, 1])
 * - MCS index (discrete, 0-22)
 */
export const DEFAULT_UAV_ACTION_SPACE: Space = {
    kind: 'dict',
    spaces: {
        flight: { kind: 'box', shape: [3], low: -1, high: 1 },
        txPower: { kind: 'box', shape: [1], low: 0, high: 1 },
        mcs: { kind: 'discrete', n: 23 },
    },
};

/**
 * Simple discrete action space for basic experiments
 */
export const SIMPLE_ACTION_SPACE: Space = {
    kind: 'discrete',
    n: 7,
    labels: [
        'hover',
        'forward',
        'backward',
        'left',
        'right',
        'up',
        'down',
    ],
};

/**
 * Flight-only action space (no communication control)
 */
export const FLIGHT_ONLY_ACTION_SPACE: Space = {
    kind: 'box',
    shape: [3],
    low: -1,
    high: 1,
};

/**
 * Extended action space with beam control
 */
export const EXTENDED_UAV_ACTION_SPACE: Space = {
    kind: 'dict',
    spaces: {
        flight: { kind: 'box', shape: [3], low: -1, high: 1 },
        txPower: { kind: 'box', shape: [1], low: 0, high: 1 },
        mcs: { kind: 'discrete', n: 23 },
        beam: { kind: 'discrete', n: 64 },
        bandwidth: { kind: 'discrete', n: 5, labels: ['5MHz', '10MHz', '15MHz', '20MHz', '40MHz'] },
    },
};

// ==================== Default Action Mapper ====================

/**
 * Configuration for default action mapper
 */
export interface ActionMapperConfig {
    /** Maximum velocity delta [m/s] (default: 5) */
    maxVelocityDelta?: number;
    /** Minimum TX power [dBm] (default: 0) */
    minTxPowerDbm?: number;
    /** Maximum TX power [dBm] (default: 23) */
    maxTxPowerDbm?: number;
    /** Number of beams (default: 64) */
    numBeams?: number;
}

const DEFAULT_MAPPER_CONFIG: Required<ActionMapperConfig> = {
    maxVelocityDelta: 5,
    minTxPowerDbm: 0,
    maxTxPowerDbm: 23,
    numBeams: 64,
};

/**
 * Default UAV action mapper
 *
 * Converts normalized AI actions to physical UAV commands.
 *
 * @param action - Action from AI agent (Dict format expected)
 * @param config - Optional configuration
 * @returns UAV command structure
 *
 * @example
 * ```typescript
 * const action = {
 *   flight: [0.5, -0.2, 0.1],
 *   txPower: [0.8],
 *   mcs: 15,
 * };
 *
 * const cmd = defaultUavActionMapper(action);
 * // cmd = {
 * //   flight: { velocityDelta: [2.5, -1.0, 0.5] },
 * //   comm: { txPowerDbm: 18.4, mcsIndex: 15 },
 * // }
 * ```
 */
export function defaultUavActionMapper(
    action: Action,
    config: ActionMapperConfig = {}
): UavCommand {
    const cfg = { ...DEFAULT_MAPPER_CONFIG, ...config };

    // Handle simple discrete action
    if (typeof action === 'number') {
        return discreteActionToCommand(action, cfg);
    }

    // Handle array action (flight only)
    if (Array.isArray(action)) {
        return {
            flight: {
                velocityDelta: action.map((v) => v * cfg.maxVelocityDelta),
            },
        };
    }

    // Handle dict action
    const dictAction = action as Record<string, unknown>;
    const cmd: UavCommand = {};

    // Flight control
    if ('flight' in dictAction) {
        const flight = dictAction.flight as number[];
        cmd.flight = {
            velocityDelta: flight.map((v) => v * cfg.maxVelocityDelta),
        };
    }

    // Communication control
    if ('txPower' in dictAction || 'mcs' in dictAction) {
        cmd.comm = {};

        if ('txPower' in dictAction) {
            const txPower = dictAction.txPower as number[];
            const normalized = txPower[0] ?? 0.5;
            cmd.comm.txPowerDbm =
                cfg.minTxPowerDbm +
                normalized * (cfg.maxTxPowerDbm - cfg.minTxPowerDbm);
        }

        if ('mcs' in dictAction) {
            cmd.comm.mcsIndex = Math.floor(dictAction.mcs as number);
        }
    }

    // Beam control
    if ('beam' in dictAction) {
        cmd.beam = {
            beamIndex: Math.floor(dictAction.beam as number),
        };
    }

    // Bandwidth
    if ('bandwidth' in dictAction && cmd.comm) {
        const bwIndex = dictAction.bandwidth as number;
        const bandwidths = [5, 10, 15, 20, 40];
        cmd.comm.bandwidth = bandwidths[bwIndex] ?? 20;
    }

    return cmd;
}

/**
 * Convert simple discrete action to command
 */
function discreteActionToCommand(
    action: number,
    cfg: Required<ActionMapperConfig>
): UavCommand {
    const vd = cfg.maxVelocityDelta;

    // Action labels: hover, forward, backward, left, right, up, down
    const velocityMap: Record<number, number[]> = {
        0: [0, 0, 0], // hover
        1: [vd, 0, 0], // forward
        2: [-vd, 0, 0], // backward
        3: [0, vd, 0], // left
        4: [0, -vd, 0], // right
        5: [0, 0, vd], // up
        6: [0, 0, -vd], // down
    };

    return {
        flight: {
            velocityDelta: velocityMap[action] ?? [0, 0, 0],
        },
    };
}

// ==================== Action Builder ====================

/**
 * Action builder for creating custom mappers
 *
 * @example
 * ```typescript
 * const mapper = new ActionBuilder<MyCommand>()
 *   .map('flight', (v) => ({ velocity: v.map(x => x * 10) }))
 *   .map('power', (v) => ({ txPower: v[0] * 23 }))
 *   .build();
 *
 * const cmd = mapper(action);
 * ```
 */
export class ActionBuilder<Cmd> {
    private mappers: {
        key: string;
        fn: (value: unknown) => Partial<Cmd>;
    }[] = [];

    /**
     * Add a field mapper
     *
     * @param key - Field name in action
     * @param mapper - Function to convert value to partial command
     */
    map(key: string, mapper: (value: unknown) => Partial<Cmd>): this {
        this.mappers.push({ key, fn: mapper });
        return this;
    }

    /**
     * Build the action mapper function
     */
    build(): ActionMapper<Action, Cmd> {
        const mappers = [...this.mappers];
        return (action: Action): Cmd => {
            if (typeof action !== 'object' || action === null) {
                return {} as Cmd;
            }

            const dictAction = action as Record<string, unknown>;
            let result = {} as Cmd;

            for (const { key, fn } of mappers) {
                if (key in dictAction) {
                    result = { ...result, ...fn(dictAction[key]) };
                }
            }

            return result;
        };
    }
}

// ==================== Action Validation ====================

/**
 * Clamp action values to valid ranges
 *
 * @param action - Action to clamp
 * @param space - Action space with bounds
 * @returns Clamped action
 */
export function clampAction(action: Action, space: Space): Action {
    if (typeof action === 'number') {
        if (space.kind === 'discrete') {
            return Math.max(0, Math.min(space.n - 1, Math.floor(action)));
        }
        if (space.kind === 'box') {
            const low = typeof space.low === 'number' ? space.low : space.low[0];
            const high =
                typeof space.high === 'number' ? space.high : space.high[0];
            return Math.max(low, Math.min(high, action));
        }
        return action;
    }

    if (Array.isArray(action)) {
        if (space.kind === 'box') {
            return action.map((v, i) => {
                const low =
                    typeof space.low === 'number' ? space.low : (space.low[i] ?? -1);
                const high =
                    typeof space.high === 'number'
                        ? space.high
                        : (space.high[i] ?? 1);
                return Math.max(low, Math.min(high, v));
            });
        }
        if (space.kind === 'multiDiscrete') {
            return action.map((v, i) =>
                Math.max(0, Math.min(space.nvec[i] - 1, Math.floor(v)))
            );
        }
        return action;
    }

    if (typeof action === 'object' && action !== null) {
        if (space.kind === 'dict') {
            const result: Record<string, Action> = {};
            for (const [key, value] of Object.entries(action)) {
                const subSpace = space.spaces[key];
                result[key] = subSpace
                    ? clampAction(value as Action, subSpace)
                    : value;
            }
            return result;
        }
    }

    return action;
}

// ==================== MCS Utilities ====================

/**
 * Get spectral efficiency for MCS index
 */
export function getMcsSpectralEfficiency(mcsIndex: number): number {
    return getMcsConfig(mcsIndex).spectralEfficiency;
}

/**
 * Get modulation type for MCS index
 */
export function getMcsModulation(mcsIndex: number): string {
    return getMcsConfig(mcsIndex).modulation;
}

/**
 * Find optimal MCS for target SINR (simplified)
 *
 * @param sinrDb - Target SINR in dB
 * @param targetBler - Target block error rate (default: 0.1)
 * @returns Recommended MCS index
 */
export function findOptimalMcs(sinrDb: number, targetBler = 0.1): number {
    // Simplified SINR to MCS mapping based on typical 5G NR curves
    // In practice, this would use BLER lookup tables
    const sinrThresholds = [
        -6, -4, -2, 0, 2, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
        19, 20, 21, 22,
    ];

    // Adjust for target BLER (lower BLER needs higher SINR margin)
    const margin = targetBler < 0.01 ? 3 : targetBler < 0.1 ? 1 : 0;
    const effectiveSinr = sinrDb - margin;

    // Find highest MCS that can be supported
    let mcsIndex = 0;
    for (let i = 0; i < sinrThresholds.length; i++) {
        if (effectiveSinr >= sinrThresholds[i]) {
            mcsIndex = i;
        } else {
            break;
        }
    }

    return mcsIndex;
}
