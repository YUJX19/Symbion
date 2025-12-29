/**
 * @module tasks/u2u-mcs/policy
 * @description Remote Decision Maker with full lifecycle support for Task2
 * 
 * Implements DecisionMaker interface with:
 * - reset(): Reinitialize agent state at episode start
 * - decide(): Get output from remote agent
 * - close(): Gracefully terminate agent
 */

import type { DecisionMaker } from '../../core/runner';
import type { SeededRandom, TaskConfig } from '../../core/repro';
import type { ProtocolHandler } from '../../ai/interface/protocol';
import type { StdioTransport } from './transport';
import {
    createResetMsg,
    createStepMsg,
    createCloseMsg,
    type ActionResponse
} from './protocol';
import { validateAction } from './schema';
import { timeoutError } from './errors';

// ==================== Types ====================

export interface RemotePolicyOptions {
    /** Protocol handler for message formatting */
    protocol: ProtocolHandler;
    /** Transport for sending/receiving messages */
    transport: StdioTransport;
    /** Task configuration (for seed) */
    taskConfig: TaskConfig;
    /** Request timeout in milliseconds */
    timeoutMs?: number;
    /** Enable logging of actions */
    enableLogging?: boolean;
}

// ==================== Remote Policy ====================

/**
 * Decision maker that delegates to a remote Python agent
 * 
 * Lifecycle:
 * 1. reset() - Called at episode start, sends reset message
 * 2. decide() - Called each step, sends step message
 * 3. close() - Called at end, sends close message
 */
export class RemotePolicy implements DecisionMaker<number[], number> {
    readonly name = 'remote-lstm-agent';

    private readonly protocol: ProtocolHandler;
    private readonly transport: StdioTransport;
    private readonly taskConfig: TaskConfig;
    private readonly timeoutMs: number;
    private readonly enableLogging: boolean;

    // State tracking
    private needsReset = true;
    private lastReward = 0;
    private lastDone = false;
    private stepCount = 0;

    constructor(options: RemotePolicyOptions) {
        this.protocol = options.protocol;
        this.transport = options.transport;
        this.taskConfig = options.taskConfig;
        this.timeoutMs = options.timeoutMs ?? 5000;
        this.enableLogging = options.enableLogging ?? false;
    }

    /**
     * Make a decision for the given input
     * 
     * If needsReset is true, sends reset message first.
     */
    async decide(input: number[], _rng: SeededRandom): Promise<number> {
        // Send reset if needed
        if (this.needsReset) {
            await this.sendReset();
            this.needsReset = false;
        }

        // Create and send step message
        const reqId = this.protocol.generateRequestId();
        const msg = createStepMsg(
            this.protocol,
            input,
            this.lastReward,
            this.lastDone,
            {},
            reqId
        );

        this.transport.send(this.protocol.serializeMessage(msg));
        this.stepCount++;

        try {
            const action = await this.protocol.registerRequest<number>(reqId, this.timeoutMs);

            // Validate and clamp action
            const validation = validateAction(action);
            if (!validation.valid) {
                if (this.enableLogging) {
                    console.warn(`[Policy] ${validation.error}, clamping to ${validation.clamped}`);
                }
            }

            return validation.clamped;
        } catch (e) {
            // Timeout or error - use fallback heuristic
            if (this.enableLogging) {
                console.error(`[Policy] Timeout on step ${this.stepCount}, using fallback`);
            }
            return this.fallbackOutput(input);
        }
    }

    /**
     * Update state with step result (for next step's feedback)
     */
    update(reward: number, done: boolean): void {
        this.lastReward = reward;
        this.lastDone = done;
    }

    /**
     * Reset the policy for a new episode
     */
    reset(): void {
        this.needsReset = true;
        this.lastReward = 0;
        this.lastDone = false;
        this.stepCount = 0;
    }

    /**
     * Send reset message to agent
     */
    private async sendReset(): Promise<void> {
        const reqId = this.protocol.generateRequestId();
        const msg = createResetMsg(this.protocol, this.taskConfig.seed, reqId);

        this.transport.send(this.protocol.serializeMessage(msg));

        try {
            // Agent responds with initial action (which we discard)
            await this.protocol.registerRequest<number>(reqId, this.timeoutMs);
            if (this.enableLogging) {
                console.log('[Policy] Reset completed');
            }
        } catch (e) {
            // Log but don't fail - agent might not support reset yet
            if (this.enableLogging) {
                console.warn('[Policy] Reset timeout, continuing anyway');
            }
        }
    }

    /**
     * Close the agent gracefully
     */
    async close(): Promise<void> {
        try {
            const msg = createCloseMsg(this.protocol, 'experiment_complete');
            this.transport.send(this.protocol.serializeMessage(msg));

            // Give agent time to cleanup
            await new Promise(resolve => setTimeout(resolve, 100));
        } catch (e) {
            // Ignore close errors
        }
    }

    /**
     * Fallback output when agent times out
     * 
     * Uses simple SINR-to-MCS mapping heuristic
     */
    private fallbackOutput(input: number[]): number {
        // input[0] is normalized SINR (divided by 30)
        const sinr = input[0] * 30;

        // Simple mapping: higher SINR -> higher MCS
        const mcs = Math.floor((sinr + 10) / 2);
        return Math.max(0, Math.min(22, mcs));
    }

    /**
     * Get statistics for logging
     */
    getStats(): { stepCount: number; lastAction?: number } {
        return {
            stepCount: this.stepCount
        };
    }
}

// ==================== Factory ====================

/**
 * Create a RemotePolicy for Task2
 */
export function createRemotePolicy(
    protocol: ProtocolHandler,
    transport: StdioTransport,
    taskConfig: TaskConfig,
    options?: { timeoutMs?: number; enableLogging?: boolean }
): RemotePolicy {
    return new RemotePolicy({
        protocol,
        transport,
        taskConfig,
        ...options
    });
}
