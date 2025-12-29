/**
 * @module tasks/u2u-mcs/protocol
 * @description Protocol types and message helpers for U2U-MCS Task2
 * 
 * Defines the communication protocol between TypeScript environment and Python agent.
 */

import type { ProtocolHandler } from '../../ai/interface/protocol';
import { TASK2_SCHEMA, TASK2_SCHEMA_HASH, TASK2_OBS_SPACE, TASK2_ACT_SPACE } from './schema';

// ==================== Request Types ====================

/**
 * Hello request for handshake
 */
export interface HelloRequest {
    observationSpace: typeof TASK2_OBS_SPACE;
    actionSpace: typeof TASK2_ACT_SPACE;
    schemaHash: string;
}

/**
 * Reset request to reinitialize agent state
 */
export interface ResetRequest {
    seed?: number;
}

/**
 * Step request with observation and feedback
 */
export interface StepRequest {
    observation: number[];
    reward: number;
    done: boolean;
    info: Record<string, unknown>;
}

/**
 * Close request to terminate agent
 */
export interface CloseRequest {
    reason?: string;
}

// ==================== Response Types ====================

/**
 * Hello acknowledgment response
 */
export interface HelloAckResponse {
    accepted: boolean;
    schemaHash: string;
    agentVersion?: string;
    agentName?: string;
}

/**
 * Action response from agent
 */
export interface ActionResponse {
    action: number;
}

/**
 * Error response from agent
 */
export interface ErrorResponse {
    code: string;
    message: string;
    details?: unknown;
}

// ==================== Message Types ====================

export type MsgType =
    | 'hello'
    | 'helloAck'
    | 'reset'
    | 'step'
    | 'action'
    | 'close'
    | 'error';

// ==================== Message Factories ====================

/**
 * Create hello message for handshake
 */
export function createHelloMsg(
    protocol: ProtocolHandler,
    reqId: string
) {
    const data: HelloRequest = {
        observationSpace: TASK2_OBS_SPACE,
        actionSpace: TASK2_ACT_SPACE,
        schemaHash: TASK2_SCHEMA_HASH
    };
    return protocol.createMessage('hello', data, reqId);
}

/**
 * Create reset message
 */
export function createResetMsg(
    protocol: ProtocolHandler,
    seed: number,
    reqId: string
) {
    const data: ResetRequest = { seed };
    return protocol.createMessage('reset', data, reqId);
}

/**
 * Create step message with observation
 */
export function createStepMsg(
    protocol: ProtocolHandler,
    observation: number[],
    reward: number,
    done: boolean,
    info: Record<string, unknown>,
    reqId: string
) {
    const data: StepRequest = { observation, reward, done, info };
    return protocol.createMessage('step', data, reqId);
}

/**
 * Create close message
 */
export function createCloseMsg(
    protocol: ProtocolHandler,
    reason?: string
) {
    const data: CloseRequest = { reason };
    // Type cast needed as 'close' is a custom message type for this task
    return protocol.createMessage('close' as any, data);
}

// ==================== Protocol Helpers ====================

/**
 * Send a message and wait for response
 */
export async function sendAndAwait<T>(
    transport: { send: (msg: string) => void },
    protocol: ProtocolHandler,
    message: ReturnType<typeof protocol.createMessage>,
    timeoutMs: number = 5000
): Promise<T> {
    const reqId = message.requestId;
    if (!reqId) {
        throw new Error('Message must have a requestId for request-response');
    }

    transport.send(protocol.serializeMessage(message));
    return protocol.registerRequest<T>(reqId, timeoutMs);
}

/**
 * Validate helloAck response
 */
export function validateHelloAck(response: HelloAckResponse): boolean {
    if (!response.accepted) return false;
    if (response.schemaHash !== TASK2_SCHEMA_HASH) return false;
    return true;
}
