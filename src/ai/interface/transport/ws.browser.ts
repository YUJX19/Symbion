/**
 * @module ai/transport/ws.browser
 * @description Browser WebSocket transport implementation.
 */

import {
    BaseTransport,
    type TransportOptions,
} from './transport';

/**
 * Browser WebSocket transport
 *
 * Uses the native browser WebSocket API.
 *
 * @example
 * ```typescript
 * const transport = new BrowserWebSocketTransport({
 *   host: 'localhost',
 *   port: 8765,
 *   autoReconnect: true,
 * });
 *
 * transport.onEvent((event) => {
 *   if (event.type === 'message') {
 *     console.log('Received:', event.data);
 *   }
 * });
 *
 * await transport.connect();
 * transport.send('{"type":"hello"}');
 * ```
 */
export class BrowserWebSocketTransport extends BaseTransport {
    private socket: WebSocket | null = null;
    private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
    private shouldReconnect = false;

    constructor(options: TransportOptions) {
        super(options);
    }

    /**
     * Connect to the WebSocket server
     */
    async connect(): Promise<void> {
        if (this.state === 'connected' || this.state === 'connecting') {
            return;
        }

        this.shouldReconnect = true;
        return this.doConnect();
    }

    private doConnect(): Promise<void> {
        return new Promise((resolve, reject) => {
            this.setState('connecting');

            const url = `ws://${this.options.host}:${this.options.port}`;

            // Check if WebSocket is available
            if (typeof WebSocket === 'undefined') {
                const error = new Error(
                    'WebSocket not available in this environment. Use ws.node.ts for Node.js.'
                );
                this.setState('error');
                this.emit({ type: 'error', error });
                reject(error);
                return;
            }

            // Connection timeout
            const timeoutId = setTimeout(() => {
                if (this.state === 'connecting') {
                    this.socket?.close();
                    const error = new Error(
                        `Connection timeout after ${this.options.connectionTimeoutMs}ms`
                    );
                    this.setState('error');
                    this.emit({ type: 'error', error });
                    reject(error);
                }
            }, this.options.connectionTimeoutMs);

            try {
                this.socket = new WebSocket(url);

                this.socket.onopen = () => {
                    clearTimeout(timeoutId);
                    this.setState('connected');
                    this.emit({ type: 'connected' });
                    resolve();
                };

                this.socket.onmessage = (event) => {
                    if (typeof event.data === 'string') {
                        this.emit({ type: 'message', data: event.data });
                    }
                };

                this.socket.onerror = () => {
                    clearTimeout(timeoutId);
                    const error = new Error('WebSocket error');
                    this.emit({ type: 'error', error });
                    if (this.state === 'connecting') {
                        reject(error);
                    }
                };

                this.socket.onclose = () => {
                    clearTimeout(timeoutId);
                    const wasConnected = this.state === 'connected';
                    this.setState('disconnected');
                    this.emit({ type: 'disconnected' });

                    // Auto-reconnect if enabled and not intentionally disconnected
                    if (
                        wasConnected &&
                        this.shouldReconnect &&
                        this.options.autoReconnect
                    ) {
                        this.scheduleReconnect();
                    }
                };
            } catch (e) {
                clearTimeout(timeoutId);
                const error =
                    e instanceof Error ? e : new Error(String(e));
                this.setState('error');
                this.emit({ type: 'error', error });
                reject(error);
            }
        });
    }

    /**
     * Disconnect from the WebSocket server
     */
    disconnect(): void {
        this.shouldReconnect = false;
        this.cancelReconnect();

        if (this.socket) {
            this.socket.close();
            this.socket = null;
        }

        this.setState('disconnected');
    }

    /**
     * Send a message
     */
    send(message: string): void {
        if (!this.socket || this.state !== 'connected') {
            throw new Error('Not connected');
        }
        this.socket.send(message);
    }

    private scheduleReconnect(): void {
        if (this.reconnectTimer) return;

        this.setState('reconnecting');
        this.emit({ type: 'reconnecting' });

        this.reconnectTimer = setTimeout(() => {
            this.reconnectTimer = null;
            this.doConnect().catch(() => {
                // Reconnect failed, try again
                if (this.shouldReconnect && this.options.autoReconnect) {
                    this.scheduleReconnect();
                }
            });
        }, this.options.reconnectDelayMs);
    }

    private cancelReconnect(): void {
        if (this.reconnectTimer) {
            clearTimeout(this.reconnectTimer);
            this.reconnectTimer = null;
        }
    }
}
