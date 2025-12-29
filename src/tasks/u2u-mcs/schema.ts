/**
 * @module tasks/u2u-mcs/schema
 * @description Schema definitions for U2U-MCS Task2
 * 
 * Provides:
 * - Observation and Action space definitions
 * - Schema hash for handshake validation
 */

// ==================== Hash Utility ====================

/**
 * Create a stable hash from a string (djb2 algorithm)
 */
function createSchemaHash(str: string): string {
    let hash = 5381;
    for (let i = 0; i < str.length; i++) {
        hash = ((hash << 5) + hash) ^ str.charCodeAt(i);
    }
    const h1 = (hash >>> 0).toString(16).padStart(8, '0');
    const h2 = ((hash * 31) >>> 0).toString(16).padStart(8, '0');
    return h1 + h2;
}

// ==================== Space Definitions ====================

/**
 * Observation space for Task2
 * 
 * 7-dimensional normalized vector:
 * - [0] sinrDb / 30     : Normalized SINR (dB)
 * - [1] distance / 200  : Normalized distance (m)
 * - [2] speed / 30      : Normalized relative speed (m/s)
 * - [3] recentAckRate   : Recent ACK rate (0-1)
 * - [4] recentBler      : Recent BLER (0-1)
 * - [5] lastMcs / 22    : Normalized last MCS index
 * - [6] step / maxSteps : Normalized step progress
 */
export const TASK2_OBS_SPACE = {
    kind: 'box' as const,
    shape: [7] as const,
    low: [-1, 0, 0, 0, 0, 0, 0],
    high: [2, 1, 1, 1, 1, 1, 1],
    dtype: 'float32' as const,
    description: '[sinrDb, distance, speed, ackRate, bler, lastMcs, step] (normalized)'
};

/**
 * Action space for Task2
 * 
 * Discrete MCS selection (0-22, 23 total options)
 */
export const TASK2_ACT_SPACE = {
    kind: 'discrete' as const,
    n: 23,
    description: 'MCS index (0-22)'
};

// ==================== Combined Schema ====================

/**
 * Complete schema for Task2 protocol
 */
export const TASK2_SCHEMA = {
    observationSpace: TASK2_OBS_SPACE,
    actionSpace: TASK2_ACT_SPACE,
    version: '1.0.0'
};

/**
 * Stable hash for handshake validation
 * 
 * This hash ensures agent and environment are using compatible schemas.
 * If schema changes, hash will change, causing handshake to fail.
 */
export const TASK2_SCHEMA_HASH = createSchemaHash(
    JSON.stringify({
        obs: TASK2_OBS_SPACE,
        act: TASK2_ACT_SPACE
    })
);

// ==================== Validation ====================

/**
 * Validate that a schema hash matches expected
 */
export function validateSchemaHash(received: string): boolean {
    return received === TASK2_SCHEMA_HASH;
}

/**
 * Validate observation bounds
 */
export function validateObservation(obs: number[]): { valid: boolean; error?: string } {
    if (obs.length !== TASK2_OBS_SPACE.shape[0]) {
        return {
            valid: false,
            error: `Expected ${TASK2_OBS_SPACE.shape[0]} elements, got ${obs.length}`
        };
    }
    return { valid: true };
}

/**
 * Validate action value
 */
export function validateAction(action: number): { valid: boolean; clamped: number; error?: string } {
    if (typeof action !== 'number' || isNaN(action)) {
        return { valid: false, clamped: 0, error: `Invalid action type: ${typeof action}` };
    }

    const clamped = Math.max(0, Math.min(22, Math.floor(action)));
    const valid = action >= 0 && action <= 22 && Number.isInteger(action);

    return {
        valid,
        clamped,
        error: valid ? undefined : `Action ${action} out of bounds [0, 22]`
    };
}
