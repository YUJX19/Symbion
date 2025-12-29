/**
 * @module ai/interface
 * @description Generic AI Interface types and protocol definitions for Symbion.
 *
 * This module provides a framework-agnostic type system for connecting Symbion
 * to various AI backends (RL agents, LLMs, Vision models, etc.).
 *
 * ## Architecture
 *
 * ```
 * ┌─────────────────────────────────────────────────────────────────┐
 * │                      AI Backend                                 │
 * │              (RL / LLM / Vision / Custom)                       │
 * └────────────────────────────┬────────────────────────────────────┘
 *                              │ WebSocket (Protocol v1)
 *                              ▼
 * ┌─────────────────────────────────────────────────────────────────┐
 * │                    AiInterfaceClient                            │
 * │                   Input → AI → Output                           │
 * └─────────────────────────────────────────────────────────────────┘
 * ```
 *
 * ## Key Concepts
 *
 * - **Space**: Input/output data structure definitions (Discrete, Box, Dict, etc.)
 * - **Protocol**: Message format with requestId pairing for reliable communication
 * - **Input/Output**: Generic typed data exchange between simulator and AI
 *
 * ## RL Compatibility
 *
 * For Reinforcement Learning, use the RL-specific aliases:
 * - `Observation` = `Input`
 * - `Action` = `Output`
 * - `observationSpace` = `inputSpace`
 * - `actionSpace` = `outputSpace`
 */

// ==================== Space Types (AI Environment) ====================

/**
 * Discrete space - finite set of possible values [0, n)
 */
export interface DiscreteSpace {
    kind: 'discrete';
    /** Number of discrete values */
    n: number;
    /** Optional labels for each value */
    labels?: string[];
}

/**
 * Box space - n-dimensional continuous space with bounds
 */
export interface BoxSpace {
    kind: 'box';
    /** Shape of the space (e.g., [3] for 3D vector) */
    shape: number[];
    /** Lower bound (scalar or per-dimension) */
    low: number | number[];
    /** Upper bound (scalar or per-dimension) */
    high: number | number[];
    /** Optional dtype hint ('float32', 'float64') */
    dtype?: 'float32' | 'float64';
}

/**
 * MultiDiscrete space - multiple discrete spaces combined
 */
export interface MultiDiscreteSpace {
    kind: 'multiDiscrete';
    /** Number of values for each dimension */
    nvec: number[];
    /** Optional labels for each dimension's values */
    labels?: string[][];
}

/**
 * Dict space - dictionary of named spaces
 */
export interface DictSpace {
    kind: 'dict';
    /** Named sub-spaces */
    spaces: Record<string, Space>;
}

/**
 * Tuple space - ordered sequence of spaces
 */
export interface TupleSpace {
    kind: 'tuple';
    /** Sub-spaces in order */
    spaces: Space[];
}

/**
 * Union type for all AI environment spaces
 */
export type Space =
    | DiscreteSpace
    | BoxSpace
    | MultiDiscreteSpace
    | DictSpace
    | TupleSpace;

// ==================== Generic Input/Output Types ====================

/**
 * Generic input value type - can be nested for Dict/Tuple spaces.
 * Used for any data passed TO the AI backend.
 */
export type InputValue = number | number[] | InputDict;

/**
 * Input dictionary type
 */
export interface InputDict {
    [key: string]: InputValue;
}

/**
 * Generic input type - data sent to AI backend
 */
export type Input = InputValue;

/**
 * Generic output value type - can be nested for Dict/Tuple spaces.
 * Used for any data returned FROM the AI backend.
 */
export type OutputValue = number | number[] | OutputDict;

/**
 * Output dictionary type
 */
export interface OutputDict {
    [key: string]: OutputValue;
}

/**
 * Generic output type - data returned from AI backend
 */
export type Output = OutputValue;

// ==================== RL Backward Compatibility Aliases ====================

/**
 * @deprecated Use Input instead. Kept for RL compatibility.
 */
export type Observation = Input;
export type ObservationValue = InputValue;
export type ObservationDict = InputDict;

/**
 * @deprecated Use Output instead. Kept for RL compatibility.
 */
export type Action = Output;
export type ActionValue = OutputValue;
export type ActionDict = OutputDict;

// ==================== Protocol Message Types ====================

/**
 * All possible message types in the protocol
 */
export type MsgType =
    // RL-specific messages
    | 'hello'
    | 'hello_ack'
    | 'config'
    | 'ack'
    | 'reset'
    | 'step'
    | 'action'
    // Generic AI messages
    | 'query'
    | 'response'
    | 'batch_query'
    | 'batch_response'
    | 'stream_start'
    | 'stream_chunk'
    | 'stream_end'
    // Utility messages
    | 'ping'
    | 'pong'
    | 'error'
    | 'event';


/**
 * Protocol message structure
 *
 * All messages follow this format for reliable request-response pairing.
 */
export interface Msg<T = unknown> {
    /** Protocol version (always 1 for now) */
    v: 1;
    /** Message type */
    type: MsgType;
    /** Session identifier */
    sessionId: string;
    /** Sequence number (monotonically increasing per session) */
    seq: number;
    /** Timestamp in milliseconds */
    timestamp: number;
    /** Request ID for request-response pairing (required for step/reset/action) */
    requestId?: string;
    /** Message payload */
    data: T;
}

// ==================== Protocol Data Payloads ====================

/**
 * Hello message payload (client → server)
 */
export interface HelloData {
    clientVersion: string;
    observationSpace: Space;
    actionSpace: Space;
    /** Schema hash for reproducibility verification */
    schemaHash?: string;
    /** Client capabilities */
    capabilities?: string[];
}

/**
 * Hello acknowledgment payload (server → client)
 */
export interface HelloAckData {
    serverVersion: string;
    accepted: boolean;
    error?: string;
    /** Server's computed schema hash (for validation) */
    schemaHash?: string;
    /** Server capabilities */
    capabilities?: string[];
}

/**
 * Reset message payload (client → server)
 */
export interface ResetData {
    observation: Observation;
    info?: Record<string, unknown>;
}

/**
 * Step message payload (client → server)
 */
export interface StepData {
    observation: Observation;
    reward: number;
    done: boolean;
    truncated?: boolean;
    info?: Record<string, unknown>;
}

/**
 * Action message payload (server → client)
 */
export interface ActionData {
    action: Action;
}

/**
 * Error message payload
 */
export interface ErrorData {
    code: string;
    message: string;
    details?: unknown;
}

/**
 * Event message payload (for logging, metrics, etc.)
 */
export interface EventData {
    eventType: string;
    data: unknown;
}

// ==================== Error Codes ====================

/**
 * Standard error codes for the protocol
 */
export const ErrorCodes = {
    /** Protocol version mismatch */
    VERSION_MISMATCH: 'VERSION_MISMATCH',
    /** Request timeout */
    TIMEOUT: 'TIMEOUT',
    /** Invalid message format */
    INVALID_MESSAGE: 'INVALID_MESSAGE',
    /** Unknown request ID */
    UNKNOWN_REQUEST_ID: 'UNKNOWN_REQUEST_ID',
    /** Action validation failed */
    INVALID_ACTION: 'INVALID_ACTION',
    /** Observation validation failed */
    INVALID_OBSERVATION: 'INVALID_OBSERVATION',
    /** Connection error */
    CONNECTION_ERROR: 'CONNECTION_ERROR',
    /** Server internal error */
    INTERNAL_ERROR: 'INTERNAL_ERROR',
    /** Concurrent step rejected (strictStep mode) */
    CONCURRENT_STEP: 'CONCURRENT_STEP',
    /** Schema hash mismatch */
    SCHEMA_MISMATCH: 'SCHEMA_MISMATCH',
    /** Protocol error (invalid state, wrong message type, etc.) */
    PROTOCOL_ERROR: 'PROTOCOL_ERROR',
} as const;

export type ErrorCode = (typeof ErrorCodes)[keyof typeof ErrorCodes];

// ==================== Configuration ====================

/**
 * Configuration for AiInterfaceClient.
 * Supports both generic (inputSpace/outputSpace) and RL-compatible (observationSpace/actionSpace) naming.
 */
export interface AiInterfaceConfig {
    /** WebSocket server host (default: 'localhost') */
    host?: string;
    /** WebSocket server port (default: 8765) */
    port?: number;
    /** Protocol version (default: 1) */
    protocolVersion?: 1;

    /** Input data structure definition */
    inputSpace?: Space;
    /** Output data structure definition */
    outputSpace?: Space;

    /**
     * @deprecated Use inputSpace instead. Kept for RL compatibility.
     */
    observationSpace?: Space;
    /**
     * @deprecated Use outputSpace instead. Kept for RL compatibility.
     */
    actionSpace?: Space;

    /** Timeout for queries in milliseconds (default: 30000) */
    timeoutMs?: number;
    /** Whether to auto-reconnect on disconnect (default: true) */
    autoReconnect?: boolean;
    /** Reconnection delay in milliseconds (default: 1000) */
    reconnectDelayMs?: number;
    /** Heartbeat interval in milliseconds (default: 10000, 0 to disable) */
    heartbeatMs?: number;
    /**
     * Strict mode (default: true)
     * - true: Throws error if previous query not completed
     * - false: Queues requests (increased latency)
     */
    strictStep?: boolean;
}

/**
 * Default configuration values
 */
export const DEFAULT_AI_CONFIG: Required<AiInterfaceConfig> = {
    host: 'localhost',
    port: 8765,
    protocolVersion: 1,
    inputSpace: { kind: 'box', shape: [1], low: -Infinity, high: Infinity },
    outputSpace: { kind: 'discrete', n: 1 },
    observationSpace: { kind: 'box', shape: [1], low: -Infinity, high: Infinity },
    actionSpace: { kind: 'discrete', n: 1 },
    timeoutMs: 30000,
    autoReconnect: true,
    reconnectDelayMs: 1000,
    heartbeatMs: 10000,
    strictStep: true,
};

// ==================== UAV Domain Types (RL-Specific) ====================
// NOTE: These types are RL/UAV-specific and kept here for backward compatibility.
// For new code, import from 'symbion/ai/env/rlTypes' instead.

/**
 * UAV state for observation extraction.
 * @deprecated Import from rlTypes instead for new RL code.
 */
export interface UavState {
    uav: {
        position: number[];
        velocity: number[];
        acceleration?: number[];
        battery: number;
        attitude?: { roll: number; pitch: number; yaw: number };
    };
    channel: {
        sinrDb: number;
        rate: number;
        bler?: number;
        pathLoss?: number;
        ricianK?: number;
    };
    mission?: {
        target: number[];
        waypointError?: number[];
        progress?: number;
    };
    network?: {
        servingBsId: number;
        neighborBsIds?: number[];
        handoverPending?: boolean;
    };
    constraints?: {
        noFlyZoneDistance?: number;
        maxAccelerationMargin?: number;
        maxTiltMargin?: number;
    };
    time?: {
        step: number;
        simTime: number;
    };
}

/**
 * UAV command structure (action output).
 * @deprecated Import from rlTypes instead for new RL code.
 */
export interface UavCommand {
    flight?: {
        velocityDelta?: number[];
        acceleration?: number[];
        waypointIndex?: number;
        heading?: number;
        climbRate?: number;
    };
    comm?: {
        txPowerDbm?: number;
        mcsIndex?: number;
        bandwidth?: number;
        subcarrier?: number[];
    };
    beam?: {
        beamIndex?: number;
        phaseShift?: number[];
    };
    network?: {
        targetBsId?: number;
        handoverTrigger?: boolean;
    };
}

// ==================== Extractor / Mapper Interfaces (RL-Specific) ====================
// NOTE: These are RL-specific interfaces. For new code, import from rlTypes.

/**
 * Function type for extracting observations from simulation state.
 * @deprecated Import from rlTypes instead for new RL code.
 */
export interface ObservationExtractor<S = unknown> {
    (state: S): Observation;
}

/**
 * Function type for mapping actions to simulation commands.
 * @deprecated Import from rlTypes instead for new RL code.
 */
export interface ActionMapper<A = Action, Cmd = unknown> {
    (action: A): Cmd;
}

// ==================== Event Types ====================

/**
 * Client event types
 */
export type ClientEventType =
    | 'connected'
    | 'disconnected'
    | 'reconnecting'
    | 'error'
    | 'step_complete'
    | 'reset_complete'
    | 'heartbeat';

/**
 * Client event payload
 */
export interface ClientEvent {
    type: ClientEventType;
    timestamp: number;
    data?: unknown;
}

/**
 * Event callback function type
 */
export type EventCallback = (event: ClientEvent) => void;

// ==================== Utility Functions ====================

/**
 * Validate if a value matches a Space definition
 */
export function validateSpace(value: unknown, space: Space): boolean {
    switch (space.kind) {
        case 'discrete':
            return (
                typeof value === 'number' &&
                Number.isInteger(value) &&
                value >= 0 &&
                value < space.n
            );

        case 'box': {
            if (!Array.isArray(value)) return false;
            const flat = value.flat(Infinity) as number[];
            const expectedSize = space.shape.reduce((a, b) => a * b, 1);
            if (flat.length !== expectedSize) return false;

            const low = typeof space.low === 'number' ? space.low : null;
            const high = typeof space.high === 'number' ? space.high : null;
            const lowArr = Array.isArray(space.low) ? space.low : null;
            const highArr = Array.isArray(space.high) ? space.high : null;

            return flat.every((v, i) => {
                const lo = lowArr ? (lowArr[i] ?? -Infinity) : (low ?? -Infinity);
                const hi = highArr ? (highArr[i] ?? Infinity) : (high ?? Infinity);
                return typeof v === 'number' && v >= lo && v <= hi;
            });
        }

        case 'multiDiscrete': {
            if (!Array.isArray(value)) return false;
            if (value.length !== space.nvec.length) return false;
            return value.every(
                (v, i) =>
                    typeof v === 'number' &&
                    Number.isInteger(v) &&
                    v >= 0 &&
                    v < space.nvec[i]
            );
        }

        case 'dict': {
            if (typeof value !== 'object' || value === null) return false;
            const obj = value as Record<string, unknown>;
            return Object.entries(space.spaces).every(([key, subSpace]) =>
                key in obj ? validateSpace(obj[key], subSpace) : false
            );
        }

        case 'tuple': {
            if (!Array.isArray(value)) return false;
            if (value.length !== space.spaces.length) return false;
            return space.spaces.every((subSpace, i) =>
                validateSpace(value[i], subSpace)
            );
        }

        default:
            return false;
    }
}

/**
 * Generate a unique session ID
 */
export function generateSessionId(): string {
    const timestamp = Date.now().toString(36);
    const random = Math.random().toString(36).slice(2, 10);
    return `session-${timestamp}-${random}`;
}

/**
 * Generate a unique request ID
 */
export function generateRequestId(): string {
    const timestamp = Date.now().toString(36);
    const random = Math.random().toString(36).slice(2, 8);
    return `req-${timestamp}-${random}`;
}

// ==================== 5G NR MCS Table ====================

/**
 * MCS configuration entry
 */
export interface McsConfig {
    modulation: string;
    codeRate: number;
    spectralEfficiency: number;
}

/**
 * 5G NR MCS Table (simplified, based on 3GPP TS 38.214)
 */
export const MCS_TABLE: McsConfig[] = [
    { modulation: 'QPSK', codeRate: 120 / 1024, spectralEfficiency: 0.2344 },
    { modulation: 'QPSK', codeRate: 193 / 1024, spectralEfficiency: 0.377 },
    { modulation: 'QPSK', codeRate: 308 / 1024, spectralEfficiency: 0.6016 },
    { modulation: 'QPSK', codeRate: 449 / 1024, spectralEfficiency: 0.877 },
    { modulation: 'QPSK', codeRate: 602 / 1024, spectralEfficiency: 1.1758 },
    { modulation: '16QAM', codeRate: 378 / 1024, spectralEfficiency: 1.4766 },
    { modulation: '16QAM', codeRate: 434 / 1024, spectralEfficiency: 1.6953 },
    { modulation: '16QAM', codeRate: 490 / 1024, spectralEfficiency: 1.9141 },
    { modulation: '16QAM', codeRate: 553 / 1024, spectralEfficiency: 2.1602 },
    { modulation: '16QAM', codeRate: 616 / 1024, spectralEfficiency: 2.4063 },
    { modulation: '64QAM', codeRate: 438 / 1024, spectralEfficiency: 2.5703 },
    { modulation: '64QAM', codeRate: 466 / 1024, spectralEfficiency: 2.7305 },
    { modulation: '64QAM', codeRate: 517 / 1024, spectralEfficiency: 3.0293 },
    { modulation: '64QAM', codeRate: 567 / 1024, spectralEfficiency: 3.3223 },
    { modulation: '64QAM', codeRate: 616 / 1024, spectralEfficiency: 3.6094 },
    { modulation: '64QAM', codeRate: 666 / 1024, spectralEfficiency: 3.9023 },
    { modulation: '64QAM', codeRate: 719 / 1024, spectralEfficiency: 4.2129 },
    { modulation: '64QAM', codeRate: 772 / 1024, spectralEfficiency: 4.5234 },
    { modulation: '64QAM', codeRate: 822 / 1024, spectralEfficiency: 4.8164 },
    { modulation: '256QAM', codeRate: 873 / 1024, spectralEfficiency: 5.1152 },
    { modulation: '256QAM', codeRate: 910 / 1024, spectralEfficiency: 5.332 },
    { modulation: '256QAM', codeRate: 948 / 1024, spectralEfficiency: 5.5547 },
    { modulation: '256QAM', codeRate: 1006 / 1024, spectralEfficiency: 5.8906 },
];

/**
 * Get MCS configuration by index
 */
export function getMcsConfig(mcsIndex: number): McsConfig {
    const idx = Math.max(0, Math.min(MCS_TABLE.length - 1, mcsIndex));
    return MCS_TABLE[idx];
}
