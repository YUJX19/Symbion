/**
 * @module ai/transport/transport
 * @description Transport interface for AI communication.
 *
 * Defines the abstract transport layer that can be implemented
 * for different environments (Browser WebSocket, Node.js ws, Memory for testing).
 */

// ==================== Transport Interface ====================

/**
 * Transport connection options
 */
export interface TransportOptions {
    /** Host address */
    host: string;
    /** Port number */
    port: number;
    /** Auto-reconnect on disconnect */
    autoReconnect?: boolean;
    /** Reconnection delay in ms */
    reconnectDelayMs?: number;
    /** Connection timeout in ms */
    connectionTimeoutMs?: number;
}

/**
 * Transport state
 */
export type TransportState =
    | 'disconnected'
    | 'connecting'
    | 'connected'
    | 'reconnecting'
    | 'error';

/**
 * Transport event types
 */
export type TransportEventType =
    | 'connected'
    | 'disconnected'
    | 'message'
    | 'error'
    | 'reconnecting';

/**
 * Transport event
 */
export interface TransportEvent {
    type: TransportEventType;
    data?: unknown;
    error?: Error;
}

/**
 * Transport event handler
 */
export type TransportEventHandler = (event: TransportEvent) => void;

/**
 * Abstract transport interface
 *
 * All transport implementations must implement this interface.
 * This allows the protocol layer to be transport-agnostic.
 */
export interface Transport {
    /**
     * Connect to the remote endpoint
     * @returns Promise that resolves when connected
     */
    connect(): Promise<void>;

    /**
     * Disconnect from the remote endpoint
     */
    disconnect(): void;

    /**
     * Send a message
     * @param message - String message to send
     */
    send(message: string): void;

    /**
     * Register event handler
     * @param handler - Event handler callback
     */
    onEvent(handler: TransportEventHandler): void;

    /**
     * Remove event handler
     * @param handler - Event handler to remove
     */
    offEvent(handler: TransportEventHandler): void;

    /**
     * Check if connected
     */
    isConnected(): boolean;

    /**
     * Get current state
     */
    getState(): TransportState;
}

// ==================== Base Transport Class ====================

/**
 * Base transport class with common functionality
 */
export abstract class BaseTransport implements Transport {
    protected state: TransportState = 'disconnected';
    protected eventHandlers: TransportEventHandler[] = [];
    protected options: Required<TransportOptions>;

    constructor(options: TransportOptions) {
        this.options = {
            autoReconnect: true,
            reconnectDelayMs: 1000,
            connectionTimeoutMs: 10000,
            ...options,
        };
    }

    abstract connect(): Promise<void>;
    abstract disconnect(): void;
    abstract send(message: string): void;

    onEvent(handler: TransportEventHandler): void {
        this.eventHandlers.push(handler);
    }

    offEvent(handler: TransportEventHandler): void {
        const index = this.eventHandlers.indexOf(handler);
        if (index >= 0) {
            this.eventHandlers.splice(index, 1);
        }
    }

    isConnected(): boolean {
        return this.state === 'connected';
    }

    getState(): TransportState {
        return this.state;
    }

    protected emit(event: TransportEvent): void {
        for (const handler of this.eventHandlers) {
            try {
                handler(event);
            } catch (e) {
                console.error('[Transport] Event handler error:', e);
            }
        }
    }

    protected setState(state: TransportState): void {
        this.state = state;
    }
}
