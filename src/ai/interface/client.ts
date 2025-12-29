/**
 * @module ai/interface/client
 * @description Generic AI Interface Client - Framework-agnostic AI client base class.
 *
 * This module provides a generic interface for connecting Symbion to various AI backends,
 * including Reinforcement Learning agents, Large Language Models, Vision models, and more.
 *
 * ## Supported AI Types
 *
 * - **Reinforcement Learning**: Use `step()` / `reset()` methods (Gymnasium-style)
 * - **Query-Response**: Use `query()` method for single inference
 * - **Batch Processing**: Use `batch()` method for multiple inputs
 * - **Streaming**: Use `stream()` method for token-by-token generation
 *
 * @example
 * ```typescript
 * // Generic query (works for any AI type)
 * const client = new AiInterfaceClient(config);
 * const response = await client.query({ prompt: "Hello" });
 *
 * // RL-specific (via RlEnvironmentClient)
 * const rlClient = new RlEnvironmentClient(config);
 * const action = await rlClient.step(observation, reward, done);
 * ```
 */

import {
    type Space,
    type Observation,
    type Action,
    type AiInterfaceConfig,
    DEFAULT_AI_CONFIG,
    ErrorCodes,
    generateSessionId,
} from './interface';
import { ProtocolHandler, ProtocolError } from './protocol';
import type { Transport, TransportEvent } from './transport';
import { BrowserWebSocketTransport } from './transport/ws.browser';

// ==================== Generic Types ====================

/**
 * Generic input type for AI queries
 */
export type AiInput = unknown;

/**
 * Generic output type for AI responses
 */
export type AiOutput = unknown;

/**
 * Context/metadata for AI queries
 */
export interface QueryContext {
    /** Request timeout in milliseconds */
    timeoutMs?: number;
    /** Additional metadata for the AI backend */
    metadata?: Record<string, unknown>;
}

/**
 * Batch query result
 */
export interface BatchResult<T = AiOutput> {
    results: T[];
    errors: (Error | null)[];
    successCount: number;
    failureCount: number;
}

/**
 * Stream chunk event
 */
export interface StreamChunk<T = AiOutput> {
    chunk: T;
    index: number;
    done: boolean;
}

/**
 * Client connection state
 */
export type ConnectionState =
    | 'idle'
    | 'connecting'
    | 'connected'
    | 'handshaking'
    | 'ready'
    | 'disconnected'
    | 'error';

/**
 * Client event types
 */
export type AiClientEventType =
    | 'connected'
    | 'disconnected'
    | 'reconnecting'
    | 'error'
    | 'query_complete'
    | 'heartbeat';

/**
 * Client event payload
 */
export interface AiClientEvent {
    type: AiClientEventType;
    timestamp: number;
    data?: unknown;
}

/**
 * Event callback function type
 */
export type AiEventCallback = (event: AiClientEvent) => void;

// ==================== AiInterfaceClient ====================

/**
 * Generic AI Interface Client
 *
 * Provides a unified interface for connecting Symbion to any AI backend.
 * This is the base class for all AI clients in Symbion.
 *
 * ## Key Features
 *
 * - **Framework Agnostic**: Works with RL, LLM, Vision, and other AI types
 * - **Protocol v1**: Reliable request-response pairing with timeouts
 * - **Transport Abstraction**: Supports WebSocket, In-Memory, and custom transports
 * - **Event System**: Subscribe to client lifecycle events
 *
 * ## Usage Patterns
 *
 * ### 1. Query-Response (Generic)
 * ```typescript
 * const response = await client.query(input, { timeoutMs: 5000 });
 * ```
 *
 * ### 2. Batch Processing
 * ```typescript
 * const results = await client.batch([input1, input2, input3]);
 * ```
 *
 * ### 3. Streaming (for LLMs)
 * ```typescript
 * for await (const chunk of client.stream(prompt)) {
 *     process.stdout.write(chunk.chunk);
 * }
 * ```
 */
export class AiInterfaceClient {
    protected config: Required<AiInterfaceConfig>;
    protected transport: Transport;
    protected protocol: ProtocolHandler;
    protected sessionId: string;
    protected state: ConnectionState = 'idle';
    protected eventCallbacks: AiEventCallback[] = [];
    protected heartbeatTimer: ReturnType<typeof setInterval> | null = null;
    protected queryInProgress = false;

    /**
     * Create a new AiInterfaceClient
     *
     * @param config - Configuration options
     * @param transport - Optional custom transport (defaults to BrowserWebSocketTransport)
     */
    constructor(config: AiInterfaceConfig, transport?: Transport) {
        this.config = { ...DEFAULT_AI_CONFIG, ...config };
        this.sessionId = generateSessionId();
        this.protocol = new ProtocolHandler(
            this.sessionId,
            this.config.timeoutMs
        );

        // Use provided transport or create default
        this.transport =
            transport ??
            new BrowserWebSocketTransport({
                host: this.config.host,
                port: this.config.port,
                autoReconnect: this.config.autoReconnect,
                reconnectDelayMs: this.config.reconnectDelayMs,
            });

        this.setupTransport();
        this.setupProtocol();
    }

    // ==================== Public API ====================

    /**
     * Connect to the AI server
     *
     * @returns Promise that resolves when connected and handshake complete
     */
    async connect(): Promise<void> {
        if (this.state === 'ready' || this.state === 'connected') {
            return;
        }

        this.state = 'connecting';

        try {
            await this.transport.connect();
            this.state = 'connected';

            // Perform handshake
            this.state = 'handshaking';
            await this.handshake();

            this.state = 'ready';
            this.startHeartbeat();
            this.emitEvent({ type: 'connected', timestamp: Date.now() });
        } catch (e) {
            this.state = 'error';
            throw e;
        }
    }

    /**
     * Disconnect from the AI server
     */
    disconnect(): void {
        this.stopHeartbeat();

        // Reject all pending requests
        this.protocol.rejectAll(
            new ProtocolError(
                ErrorCodes.CONNECTION_ERROR,
                'Client disconnected'
            )
        );

        this.transport.disconnect();
        this.state = 'disconnected';
        this.queryInProgress = false;

        this.emitEvent({ type: 'disconnected', timestamp: Date.now() });
    }

    /**
     * Check if connected and ready
     */
    isConnected(): boolean {
        return this.state === 'ready';
    }

    /**
     * Get current connection state
     */
    getState(): ConnectionState {
        return this.state;
    }

    /**
     * Generic query method - send input to AI and get response
     *
     * This is the primary method for AI inference, suitable for:
     * - Single-shot RL decisions
     * - LLM completions
     * - Vision model predictions
     * - Any request-response AI pattern
     *
     * @param input - Input data for the AI model
     * @param context - Optional query context (timeout, metadata)
     * @returns Promise resolving to AI response
     */
    async query<I = AiInput, O = AiOutput>(
        input: I,
        context: QueryContext = {}
    ): Promise<O> {
        this.ensureReady();

        const timeoutMs = context.timeoutMs ?? this.config.timeoutMs;
        const startTime = Date.now();

        const requestId = this.protocol.generateRequestId();

        const queryData = {
            input,
            metadata: context.metadata,
        };

        const msg = this.protocol.createMessage('query', queryData, requestId);
        this.transport.send(this.protocol.serializeMessage(msg));

        const response = await this.protocol.registerRequest<O>(
            requestId,
            timeoutMs
        );

        const latencyMs = Date.now() - startTime;
        this.emitEvent({
            type: 'query_complete',
            timestamp: Date.now(),
            data: { latencyMs },
        });

        return response;
    }

    /**
     * Batch query method - process multiple inputs in parallel
     *
     * @param inputs - Array of inputs to process
     * @param context - Optional query context
     * @returns Promise resolving to batch results
     */
    async batch<I = AiInput, O = AiOutput>(
        inputs: I[],
        context: QueryContext = {}
    ): Promise<BatchResult<O>> {
        this.ensureReady();

        const timeoutMs = context.timeoutMs ?? this.config.timeoutMs;
        const requestId = this.protocol.generateRequestId();

        const batchData = {
            inputs,
            metadata: context.metadata,
        };

        const msg = this.protocol.createMessage('batch_query', batchData, requestId);
        this.transport.send(this.protocol.serializeMessage(msg));

        interface RawBatchResponse {
            results: O[];
            errors: (string | null)[];
        }

        const response = await this.protocol.registerRequest<RawBatchResponse>(
            requestId,
            timeoutMs
        );

        return {
            results: response.results,
            errors: response.errors.map((e) => (e ? new Error(e) : null)),
            successCount: response.errors.filter((e) => e === null).length,
            failureCount: response.errors.filter((e) => e !== null).length,
        };
    }

    /**
     * Stream response from AI (for LLMs and generative models)
     *
     * @param input - Input for the AI model
     * @param context - Optional query context
     * @yields Stream chunks from the AI
     */
    async *stream<I = AiInput, O = AiOutput>(
        input: I,
        context: QueryContext = {}
    ): AsyncGenerator<StreamChunk<O>> {
        this.ensureReady();

        const requestId = this.protocol.generateRequestId();

        const streamData = {
            input,
            metadata: context.metadata,
        };

        const msg = this.protocol.createMessage('stream_start', streamData, requestId);
        this.transport.send(this.protocol.serializeMessage(msg));

        // Create a stream handler
        let index = 0;
        let done = false;

        while (!done) {
            const chunk = await this.protocol.registerRequest<{
                chunk: O;
                done: boolean;
            }>(requestId, context.timeoutMs ?? this.config.timeoutMs);

            done = chunk.done;
            yield {
                chunk: chunk.chunk,
                index: index++,
                done,
            };
        }
    }

    /**
     * Subscribe to client events
     *
     * @param callback - Event callback
     */
    onEvent(callback: AiEventCallback): void {
        this.eventCallbacks.push(callback);
    }

    /**
     * Unsubscribe from client events
     *
     * @param callback - Event callback to remove
     */
    offEvent(callback: AiEventCallback): void {
        const index = this.eventCallbacks.indexOf(callback);
        if (index >= 0) {
            this.eventCallbacks.splice(index, 1);
        }
    }

    /**
     * Get input space definition
     */
    getInputSpace(): Space {
        return this.config.observationSpace;
    }

    /**
     * Get output space definition
     */
    getOutputSpace(): Space {
        return this.config.actionSpace;
    }

    /**
     * Get session ID
     */
    getSessionId(): string {
        return this.sessionId;
    }

    // ==================== Protected Methods ====================

    protected setupTransport(): void {
        this.transport.onEvent((event: TransportEvent) => {
            switch (event.type) {
                case 'message':
                    if (typeof event.data === 'string') {
                        this.handleMessage(event.data);
                    }
                    break;

                case 'disconnected':
                    if (this.state === 'ready') {
                        this.state = 'disconnected';
                        this.protocol.rejectAll(
                            new ProtocolError(
                                ErrorCodes.CONNECTION_ERROR,
                                'Connection lost'
                            )
                        );
                        this.emitEvent({
                            type: 'disconnected',
                            timestamp: Date.now(),
                        });
                    }
                    break;

                case 'reconnecting':
                    this.emitEvent({
                        type: 'reconnecting',
                        timestamp: Date.now(),
                    });
                    break;

                case 'error':
                    this.emitEvent({
                        type: 'error',
                        timestamp: Date.now(),
                        data: event.error,
                    });
                    break;
            }
        });
    }

    protected setupProtocol(): void {
        // Handle pong messages for heartbeat
        this.protocol.onMessage('pong', () => {
            this.emitEvent({ type: 'heartbeat', timestamp: Date.now() });
        });

        // Handle event messages from server
        this.protocol.onMessage('event', (msg) => {
            this.emitEvent({
                type: 'error',
                timestamp: Date.now(),
                data: msg.data,
            });
        });
    }

    protected handleMessage(raw: string): void {
        try {
            const msg = this.protocol.parseMessage(raw);
            this.protocol.handleMessage(msg);
        } catch (e) {
            console.error('[AiInterfaceClient] Failed to handle message:', e);
        }
    }

    protected async handshake(): Promise<void> {
        const requestId = this.protocol.generateRequestId();

        const helloData = {
            clientVersion: '1.0.0',
            observationSpace: this.config.observationSpace,
            actionSpace: this.config.actionSpace,
            capabilities: ['query', 'batch', 'stream', 'step', 'reset'],
        };

        const msg = this.protocol.createMessage('hello', helloData, requestId);
        this.transport.send(this.protocol.serializeMessage(msg));

        interface HelloAckResponse {
            serverVersion: string;
            accepted: boolean;
            error?: string;
            capabilities?: string[];
        }

        const response = await this.protocol.registerRequest<HelloAckResponse>(
            requestId,
            this.config.timeoutMs
        );

        if (!response.accepted) {
            throw new ProtocolError(
                ErrorCodes.CONNECTION_ERROR,
                `Server rejected connection: ${response.error ?? 'Unknown reason'}`
            );
        }
    }

    protected startHeartbeat(): void {
        if (this.config.heartbeatMs <= 0) return;

        this.heartbeatTimer = setInterval(() => {
            if (this.state === 'ready' && !this.queryInProgress) {
                const msg = this.protocol.createMessage('ping', {});
                this.transport.send(this.protocol.serializeMessage(msg));
            }
        }, this.config.heartbeatMs);
    }

    protected stopHeartbeat(): void {
        if (this.heartbeatTimer) {
            clearInterval(this.heartbeatTimer);
            this.heartbeatTimer = null;
        }
    }

    protected ensureReady(): void {
        if (this.state !== 'ready') {
            throw new ProtocolError(
                ErrorCodes.CONNECTION_ERROR,
                `Client not ready. Current state: ${this.state}`
            );
        }
    }

    protected emitEvent(event: AiClientEvent): void {
        for (const callback of this.eventCallbacks) {
            try {
                callback(event);
            } catch (e) {
                console.error('[AiInterfaceClient] Event callback error:', e);
            }
        }
    }
}

// ==================== Convenience Re-exports ====================

export { AiInterfaceClient as BaseAiClient };
