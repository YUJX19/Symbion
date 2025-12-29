/**
 * @module ai/env/rlEnvClient
 * @description RL Environment Client - Gymnasium-style interface for RL agents.
 *
 * This module provides a Reinforcement Learning environment client that extends
 * the generic AiInterfaceClient with RL-specific methods (step, reset).
 *
 * ## Usage Patterns
 *
 * ### 1. RL-Style (Gymnasium API)
 * ```typescript
 * const action = await client.step(observation, reward, done);
 * ```
 *
 * ### 2. Generic Query (alternative)
 * ```typescript
 * const action = await client.query(observation, { metadata: { reward, done } });
 * ```
 *
 * Both patterns are supported and interchangeable.
 */

import {
    type Observation,
    type Action,
    type Space,
    type AiInterfaceConfig,
    type StepData,
    type ResetData,
    ErrorCodes,
} from '../interface';

import {
    AiInterfaceClient,
    type QueryContext,
    type ConnectionState,
} from '../client';

import { ProtocolError } from '../protocol';
import type { Transport } from '../transport';

// ==================== Types ====================

/**
 * Step result from AI agent
 */
export interface StepResult {
    action: Action;
    latencyMs: number;
}

/**
 * Client states (alias for ConnectionState for backward compatibility)
 */
export type ClientState = ConnectionState;

/**
 * RL-specific query context
 */
export interface RlQueryContext extends QueryContext {
    /** Reward from previous action */
    reward?: number;
    /** Whether episode has ended */
    done?: boolean;
    /** Whether episode was truncated */
    truncated?: boolean;
}

// ==================== UavGymEnvClient ====================

/**
 * RL Environment Client (Gymnasium-style)
 *
 * Extends the generic AiInterfaceClient with Reinforcement Learning-specific
 * methods (step, reset) following the Gymnasium API pattern.
 *
 * ## Features
 *
 * - **Gymnasium API**: Compatible with standard RL environment interface
 * - **Generic Query**: Also supports generic query() for framework-agnostic usage
 * - **Protocol v1**: Reliable request-response pairing with timeouts
 * - **Strict Step Mode**: Prevents concurrent step calls
 * - **Auto-Reconnect**: Automatic reconnection on disconnect
 * - **Heartbeat**: Keep-alive mechanism
 * - **Event System**: Subscribe to client lifecycle events
 *
 * @example
 * ```typescript
 * const env = new UavGymEnvClient({
 *   observationSpace: {
 *     kind: 'dict',
 *     spaces: {
 *       uav_pos: { kind: 'box', shape: [3], low: -1000, high: 1000 },
 *       sinr_db: { kind: 'box', shape: [1], low: -20, high: 40 },
 *     },
 *   },
 *   actionSpace: {
 *     kind: 'dict',
 *     spaces: {
 *       flight: { kind: 'box', shape: [3], low: -1, high: 1 },
 *       mcs: { kind: 'discrete', n: 23 },
 *     },
 *   },
 * });
 *
 * await env.connect();
 *
 * // Option 1: RL-style usage
 * const obs = { uav_pos: [100, 200, 50], sinr_db: [15.5] };
 * const action = await env.step(obs, reward, done);
 *
 * // Option 2: Generic query usage
 * const action2 = await env.query(obs, { metadata: { reward, done } });
 * ```
 */
export class UavGymEnvClient extends AiInterfaceClient {
    private stepInProgress = false;

    /**
     * Create a new UavGymEnvClient
     *
     * @param config - Configuration options
     * @param transport - Optional custom transport (defaults to BrowserWebSocketTransport)
     */
    constructor(config: AiInterfaceConfig, transport?: Transport) {
        super(config, transport);
    }

    // ==================== RL-Specific API ====================

    /**
     * Perform a single environment step (RL-style)
     *
     * Sends current observation to AI agent and waits for action.
     * This is the standard Gymnasium-style step method.
     *
     * @param observation - Current state observation
     * @param reward - Reward from previous action
     * @param done - Whether episode has ended
     * @param info - Additional info (optional)
     * @returns Promise resolving to action from AI agent
     */
    async step(
        observation: Observation,
        reward: number,
        done: boolean,
        info: Record<string, unknown> = {}
    ): Promise<Action> {
        this.ensureReady();

        // Strict step mode check
        if (this.config.strictStep && this.stepInProgress) {
            throw new ProtocolError(
                ErrorCodes.CONCURRENT_STEP,
                'Previous step not completed. Set strictStep=false to allow queuing.'
            );
        }

        this.stepInProgress = true;
        const startTime = Date.now();

        try {
            const requestId = this.protocol.generateRequestId();

            const stepData: StepData = {
                observation,
                reward,
                done,
                info,
            };

            const msg = this.protocol.createMessage('step', stepData, requestId);
            this.transport.send(this.protocol.serializeMessage(msg));

            const action = await this.protocol.registerRequest<Action>(
                requestId,
                this.config.timeoutMs
            );

            const latencyMs = Date.now() - startTime;
            this.emitEvent({
                type: 'query_complete',
                timestamp: Date.now(),
                data: { latencyMs, type: 'step' },
            });

            return action;
        } finally {
            this.stepInProgress = false;
        }
    }

    /**
     * Reset the environment (RL-style)
     *
     * Signals new episode start to AI agent.
     * This is the standard Gymnasium-style reset method.
     *
     * @param initialObservation - Initial observation after reset
     * @param info - Additional info (optional)
     * @returns Promise resolving to first action from AI agent
     */
    async reset(
        initialObservation: Observation,
        info: Record<string, unknown> = {}
    ): Promise<Action> {
        this.ensureReady();

        // Reset should not be called during an active step
        if (this.config.strictStep && this.stepInProgress) {
            throw new ProtocolError(
                ErrorCodes.CONCURRENT_STEP,
                'Cannot reset while step is in progress'
            );
        }

        this.stepInProgress = true;

        try {
            const requestId = this.protocol.generateRequestId();

            const resetData: ResetData = {
                observation: initialObservation,
                info,
            };

            const msg = this.protocol.createMessage('reset', resetData, requestId);
            this.transport.send(this.protocol.serializeMessage(msg));

            const action = await this.protocol.registerRequest<Action>(
                requestId,
                this.config.timeoutMs
            );

            this.emitEvent({
                type: 'query_complete',
                timestamp: Date.now(),
                data: { type: 'reset' },
            });

            return action;
        } finally {
            this.stepInProgress = false;
        }
    }

    // ==================== Convenience Methods ====================

    /**
     * Get observation space definition
     * @deprecated Use getInputSpace() instead
     */
    getObservationSpace(): Space {
        return this.getInputSpace();
    }

    /**
     * Get action space definition
     * @deprecated Use getOutputSpace() instead
     */
    getActionSpace(): Space {
        return this.getOutputSpace();
    }

    /**
     * Check if a step is currently in progress
     */
    isStepInProgress(): boolean {
        return this.stepInProgress;
    }
}

// ==================== Aliases ====================

/**
 * Recommended generic alias for iterative AI interactions
 *
 * Use this name for framework-agnostic code. Supports step/reset
 * pattern for iterative AI loops (compatible with Gymnasium-style RL).
 *
 * @example
 * ```typescript
 * import { IterativeAiClient } from 'symbion/ai';
 *
 * const client = new IterativeAiClient({
 *   inputSpace: { kind: 'box', shape: [10], low: -1, high: 1 },
 *   outputSpace: { kind: 'discrete', n: 5 },
 * });
 * ```
 */
export { UavGymEnvClient as IterativeAiClient };

/**
 * Alias for RL-specific semantic clarity
 * @deprecated Prefer IterativeAiClient for new code
 */
export { UavGymEnvClient as RlEnvironmentClient };

/**
 * Alias for Gymnasium-style usage
 * @deprecated Prefer IterativeAiClient for new code
 */
export { UavGymEnvClient as GymEnvClient };
