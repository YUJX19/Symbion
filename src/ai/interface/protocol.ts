/**
 * @module ai/protocol
 * @description Protocol layer for AI interface communication.
 *
 * Handles message encoding/decoding, request-response pairing,
 * timeout management, and handshake protocol.
 */

import {
    Msg,
    MsgType,
    ErrorCodes,
    generateRequestId,
    type HelloData,
    type HelloAckData,
    type AiInterfaceConfig,
    type Space,
} from './interface';

// ==================== Types ====================

/**
 * Pending request record
 */
interface PendingRequest<T = unknown> {
    resolve: (value: T) => void;
    reject: (error: Error) => void;
    timer: ReturnType<typeof setTimeout>;
    createdAt: number;
}

/**
 * Protocol error with code
 */
export class ProtocolError extends Error {
    constructor(
        public readonly code: string,
        message: string,
        public readonly details?: unknown
    ) {
        super(message);
        this.name = 'ProtocolError';
    }
}

/**
 * Message handler callback type
 */
export type MessageHandler = (msg: Msg) => void;

// ==================== Protocol Handler ====================

/**
 * Protocol handler for managing message encoding/decoding and request-response pairing.
 *
 * @example
 * ```typescript
 * const protocol = new ProtocolHandler('session-123');
 *
 * // Register a request and wait for response
 * const requestId = protocol.generateRequestId();
 * const promise = protocol.registerRequest<ActionData>(requestId, 5000);
 *
 * // Send message (via transport)
 * const msg = protocol.createMessage('step', stepData, requestId);
 * transport.send(JSON.stringify(msg));
 *
 * // When response arrives
 * protocol.handleMessage(responseMsg);
 * const action = await promise; // Resolves when matching action arrives
 * ```
 */
export class ProtocolHandler {
    private seq = 0;
    private pendingRequests = new Map<string, PendingRequest>();
    private messageHandlers = new Map<MsgType, MessageHandler[]>();

    constructor(
        private readonly sessionId: string,
        private readonly defaultTimeoutMs: number = 30000
    ) { }

    // ==================== Message Creation ====================

    /**
     * Create a protocol message
     */
    createMessage<T>(type: MsgType, data: T, requestId?: string): Msg<T> {
        return {
            v: 1,
            type,
            sessionId: this.sessionId,
            seq: this.seq++,
            timestamp: Date.now(),
            requestId,
            data,
        };
    }

    /**
     * Generate a unique request ID
     */
    generateRequestId(): string {
        return generateRequestId();
    }

    // ==================== Request-Response Management ====================

    /**
     * Register a pending request
     *
     * @param requestId - Unique request identifier
     * @param timeoutMs - Timeout in milliseconds
     * @returns Promise that resolves when response arrives or rejects on timeout
     */
    registerRequest<T>(
        requestId: string,
        timeoutMs: number = this.defaultTimeoutMs
    ): Promise<T> {
        return new Promise<T>((resolve, reject) => {
            const timer = setTimeout(() => {
                this.pendingRequests.delete(requestId);
                reject(
                    new ProtocolError(
                        ErrorCodes.TIMEOUT,
                        `Request ${requestId} timed out after ${timeoutMs}ms`
                    )
                );
            }, timeoutMs);

            this.pendingRequests.set(requestId, {
                resolve: resolve as (value: unknown) => void,
                reject,
                timer,
                createdAt: Date.now(),
            });
        });
    }

    /**
     * Resolve a pending request
     *
     * @param requestId - Request identifier to resolve
     * @param data - Response data
     * @returns True if request was found and resolved
     */
    resolveRequest(requestId: string, data: unknown): boolean {
        const pending = this.pendingRequests.get(requestId);
        if (pending) {
            clearTimeout(pending.timer);
            this.pendingRequests.delete(requestId);
            pending.resolve(data);
            return true;
        }
        return false;
    }

    /**
     * Reject a pending request
     *
     * @param requestId - Request identifier to reject
     * @param error - Error to reject with
     * @returns True if request was found and rejected
     */
    rejectRequest(requestId: string, error: Error): boolean {
        const pending = this.pendingRequests.get(requestId);
        if (pending) {
            clearTimeout(pending.timer);
            this.pendingRequests.delete(requestId);
            pending.reject(error);
            return true;
        }
        return false;
    }

    /**
     * Reject all pending requests
     *
     * Typically called on disconnect or fatal error.
     *
     * @param error - Error to reject with
     */
    rejectAll(error: Error): void {
        for (const [, pending] of this.pendingRequests) {
            clearTimeout(pending.timer);
            pending.reject(error);
        }
        this.pendingRequests.clear();
    }

    /**
     * Check if there are pending requests
     */
    hasPendingRequests(): boolean {
        return this.pendingRequests.size > 0;
    }

    /**
     * Get number of pending requests
     */
    getPendingCount(): number {
        return this.pendingRequests.size;
    }

    // ==================== Message Handling ====================

    /**
     * Register a message handler for a specific message type
     */
    onMessage(type: MsgType, handler: MessageHandler): void {
        const handlers = this.messageHandlers.get(type) ?? [];
        handlers.push(handler);
        this.messageHandlers.set(type, handlers);
    }

    /**
     * Remove a message handler
     */
    offMessage(type: MsgType, handler: MessageHandler): void {
        const handlers = this.messageHandlers.get(type);
        if (handlers) {
            const index = handlers.indexOf(handler);
            if (index >= 0) {
                handlers.splice(index, 1);
            }
        }
    }

    /**
     * Handle an incoming message
     *
     * @param msg - Parsed protocol message
     */
    handleMessage(msg: Msg): void {
        // Validate protocol version
        if (msg.v !== 1) {
            console.warn(
                `[Protocol] Unsupported protocol version: ${msg.v}`
            );
            return;
        }

        // Validate session ID (if set)
        if (msg.sessionId !== this.sessionId) {
            console.warn(
                `[Protocol] Session ID mismatch: expected ${this.sessionId}, got ${msg.sessionId}`
            );
            return;
        }

        // Handle request-response pairing for action messages
        if (msg.type === 'action' && msg.requestId) {
            interface ActionPayload {
                action: unknown;
            }
            const payload = msg.data as ActionPayload;
            this.resolveRequest(msg.requestId, payload.action);
        }

        // Handle hello_ack for handshake
        if (msg.type === 'hello_ack' && msg.requestId) {
            this.resolveRequest(msg.requestId, msg.data);
        }

        // Handle error messages
        if (msg.type === 'error' && msg.requestId) {
            interface ErrorPayload {
                code: string;
                message: string;
                details?: unknown;
            }
            const payload = msg.data as ErrorPayload;
            this.rejectRequest(
                msg.requestId,
                new ProtocolError(payload.code, payload.message, payload.details)
            );
        }

        // Dispatch to registered handlers
        const handlers = this.messageHandlers.get(msg.type);
        if (handlers) {
            for (const handler of handlers) {
                try {
                    handler(msg);
                } catch (e) {
                    console.error(
                        `[Protocol] Handler error for ${msg.type}:`,
                        e
                    );
                }
            }
        }
    }

    // ==================== Message Parsing ====================

    /**
     * Parse a raw message string into a protocol message
     *
     * @param raw - Raw JSON string
     * @returns Parsed message
     * @throws ProtocolError if parsing fails
     */
    parseMessage(raw: string): Msg {
        try {
            const msg = JSON.parse(raw) as Msg;

            if (typeof msg !== 'object' || msg === null) {
                throw new ProtocolError(
                    ErrorCodes.INVALID_MESSAGE,
                    'Message must be an object'
                );
            }

            if (msg.v !== 1) {
                throw new ProtocolError(
                    ErrorCodes.VERSION_MISMATCH,
                    `Unsupported protocol version: ${msg.v}`
                );
            }

            if (!msg.type || typeof msg.type !== 'string') {
                throw new ProtocolError(
                    ErrorCodes.INVALID_MESSAGE,
                    'Message type is required'
                );
            }

            return msg;
        } catch (e) {
            if (e instanceof ProtocolError) throw e;
            throw new ProtocolError(
                ErrorCodes.INVALID_MESSAGE,
                `Failed to parse message: ${e instanceof Error ? e.message : String(e)}`
            );
        }
    }

    /**
     * Serialize a message to JSON string
     */
    serializeMessage(msg: Msg): string {
        return JSON.stringify(msg);
    }

    // ==================== Getters ====================

    /**
     * Get current session ID
     */
    getSessionId(): string {
        return this.sessionId;
    }

    /**
     * Get current sequence number
     */
    getSeq(): number {
        return this.seq;
    }
}

// ==================== Handshake Helper ====================

/**
 * Compute schema hash for observation and action spaces
 * Used for reproducibility verification
 */
export function computeSchemaHash(obsSpace: Space, actionSpace: Space): string {
    const canonical = JSON.stringify({
        observation: JSON.stringify(obsSpace),
        action: JSON.stringify(actionSpace),
    });
    // Simple hash using string manipulation (no crypto for browser compat)
    let hash = 0;
    for (let i = 0; i < canonical.length; i++) {
        const char = canonical.charCodeAt(i);
        hash = ((hash << 5) - hash) + char;
        hash = hash & hash; // Convert to 32bit integer
    }
    return Math.abs(hash).toString(16).padStart(8, '0');
}

/**
 * Create a handshake hello message
 */
export function createHelloMessage(
    protocol: ProtocolHandler,
    config: AiInterfaceConfig,
    requestId: string
): Msg<HelloData> {
    // Use inputSpace/outputSpace if available, fallback to observationSpace/actionSpace
    const obsSpace = config.inputSpace ?? config.observationSpace ?? { kind: 'box' as const, shape: [1], low: -Infinity, high: Infinity };
    const actSpace = config.outputSpace ?? config.actionSpace ?? { kind: 'discrete' as const, n: 1 };

    const schemaHash = computeSchemaHash(obsSpace, actSpace);
    return protocol.createMessage(
        'hello',
        {
            clientVersion: '1.0.0',
            observationSpace: obsSpace,
            actionSpace: actSpace,
            schemaHash,
            capabilities: ['timeout', 'heartbeat', 'strictStep', 'query', 'batch', 'stream'],
        },
        requestId
    );
}

/**
 * Validate hello acknowledgment response
 *
 * @param data - HelloAck data from server
 * @param expectedSchemaHash - Expected schema hash (optional, for validation)
 * @throws ProtocolError if validation fails
 */
export function validateHelloAck(data: HelloAckData, expectedSchemaHash?: string): void {
    if (!data.accepted) {
        throw new ProtocolError(
            ErrorCodes.CONNECTION_ERROR,
            `Server rejected connection: ${data.error ?? 'Unknown reason'}`
        );
    }

    // Validate schema hash if both client and server provide it
    if (expectedSchemaHash && data.schemaHash && data.schemaHash !== expectedSchemaHash) {
        throw new ProtocolError(
            ErrorCodes.SCHEMA_MISMATCH,
            `Schema hash mismatch: client=${expectedSchemaHash}, server=${data.schemaHash}`,
            { clientHash: expectedSchemaHash, serverHash: data.schemaHash }
        );
    }
}

// ==================== Space Validation Helpers ====================

/**
 * Validate action against action space
 */
export function validateAction(action: unknown, actionSpace: Space): boolean {
    return validateSpaceValue(action, actionSpace);
}

/**
 * Validate observation against observation space
 */
export function validateObservation(
    observation: unknown,
    observationSpace: Space
): boolean {
    return validateSpaceValue(observation, observationSpace);
}

/**
 * Internal space value validation
 */
function validateSpaceValue(value: unknown, space: Space): boolean {
    switch (space.kind) {
        case 'discrete':
            return (
                typeof value === 'number' &&
                Number.isInteger(value) &&
                value >= 0 &&
                value < space.n
            );

        case 'box': {
            if (typeof value === 'number') {
                // Single value for shape [1]
                if (space.shape.length !== 1 || space.shape[0] !== 1)
                    return false;
                const low =
                    typeof space.low === 'number' ? space.low : space.low[0];
                const high =
                    typeof space.high === 'number' ? space.high : space.high[0];
                return value >= low && value <= high;
            }
            if (!Array.isArray(value)) return false;
            const flat = flattenArray(value);
            const expectedSize = space.shape.reduce((a, b) => a * b, 1);
            if (flat.length !== expectedSize) return false;
            return flat.every((v, i) => {
                if (typeof v !== 'number') return false;
                const low =
                    typeof space.low === 'number'
                        ? space.low
                        : space.low[i] ?? -Infinity;
                const high =
                    typeof space.high === 'number'
                        ? space.high
                        : space.high[i] ?? Infinity;
                return v >= low && v <= high;
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
                key in obj ? validateSpaceValue(obj[key], subSpace) : false
            );
        }

        case 'tuple': {
            if (!Array.isArray(value)) return false;
            if (value.length !== space.spaces.length) return false;
            return space.spaces.every((subSpace, i) =>
                validateSpaceValue(value[i], subSpace)
            );
        }

        default:
            return false;
    }
}

/**
 * Flatten nested array to 1D.
 *
 * Recursively traverses nested arrays and extracts all numeric values
 * into a flat 1D array. Non-numeric values are ignored.
 *
 * @param arr - Nested array containing numbers or sub-arrays
 * @returns Flat array of all numeric values
 *
 * @example
 * ```typescript
 * flattenArray([[1, 2], [3, 4]]) // => [1, 2, 3, 4]
 * flattenArray([[[1]], 2, [3]]) // => [1, 2, 3]
 * ```
 */
function flattenArray(arr: unknown[]): number[] {
    const result: number[] = [];
    for (const item of arr) {
        if (Array.isArray(item)) {
            result.push(...flattenArray(item));
        } else if (typeof item === 'number') {
            result.push(item);
        }
    }
    return result;
}
