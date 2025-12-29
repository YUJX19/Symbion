/**
 * @module ai/env/rlTypes
 * @description RL-specific type re-exports and extensions.
 *
 * This module re-exports RL-related types from the base interface
 * and adds RL-specific event types.
 *
 * Use these types when working with Reinforcement Learning agents.
 */

// Re-export RL types from base interface (backward compatibility)
export type {
    Observation,
    ObservationValue,
    ObservationDict,
    Action,
    ActionValue,
    ActionDict,
    StepData,
    ResetData,
    ActionData,
    ObservationExtractor,
    ActionMapper,
    UavState,
    UavCommand,
} from '../interface';

// ==================== RL Event Types (RL-Specific) ====================

/**
 * RL-specific client event types
 */
export type RlClientEventType =
    | 'connected'
    | 'disconnected'
    | 'reconnecting'
    | 'error'
    | 'step_complete'
    | 'reset_complete'
    | 'heartbeat';

/**
 * RL client event payload
 */
export interface RlClientEvent {
    type: RlClientEventType;
    timestamp: number;
    data?: unknown;
}

/**
 * RL event callback function type
 */
export type RlEventCallback = (event: RlClientEvent) => void;
