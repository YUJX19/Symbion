/**
 * @module ai/transport/memory
 * @description In-memory transport for testing.
 *
 * Provides a transport implementation that works entirely in-memory,
 * allowing tests to run without any network I/O.
 */

import {
    BaseTransport,
    type Transport,
    type TransportOptions,
} from './transport';

/**
 * In-memory transport for testing
 *
 * Creates a pair of connected transports that can communicate
 * without any network I/O.
 *
 * @example
 * ```typescript
 * // Create a connected pair
 * const [clientTransport, serverTransport] = InMemoryTransport.createPair();
 *
 * // Messages sent on one side appear on the other
 * serverTransport.onEvent((e) => {
 *   if (e.type === 'message') {
 *     console.log('Server received:', e.data);
 *     serverTransport.send('{"type":"response"}');
 *   }
 * });
 *
 * clientTransport.onEvent((e) => {
 *   if (e.type === 'message') {
 *     console.log('Client received:', e.data);
 *   }
 * });
 *
 * await clientTransport.connect();
 * await serverTransport.connect();
 *
 * clientTransport.send('{"type":"request"}');
 * ```
 */
export class InMemoryTransport extends BaseTransport {
    private peer: InMemoryTransport | null = null;
    private messageQueue: string[] = [];
    private connectDelay = 0;
    private messageDelay = 0;

    constructor(options?: Partial<TransportOptions>) {
        super({
            host: 'memory',
            port: 0,
            ...options,
        });
    }

    /**
     * Create a connected pair of transports
     *
     * @returns Tuple of [client, server] transports
     */
    static createPair(): [InMemoryTransport, InMemoryTransport] {
        const client = new InMemoryTransport();
        const server = new InMemoryTransport();
        client.peer = server;
        server.peer = client;
        return [client, server];
    }

    /**
     * Set connection delay for simulating network latency
     */
    setConnectDelay(ms: number): this {
        this.connectDelay = ms;
        return this;
    }

    /**
     * Set message delay for simulating network latency
     */
    setMessageDelay(ms: number): this {
        this.messageDelay = ms;
        return this;
    }

    /**
     * Connect (simulated)
     */
    async connect(): Promise<void> {
        if (this.state === 'connected') return;

        this.setState('connecting');

        if (this.connectDelay > 0) {
            await this.delay(this.connectDelay);
        }

        this.setState('connected');
        this.emit({ type: 'connected' });

        // Deliver any queued messages
        await this.flushQueue();
    }

    /**
     * Disconnect
     */
    disconnect(): void {
        this.setState('disconnected');
        this.emit({ type: 'disconnected' });
    }

    /**
     * Send a message to the peer
     */
    send(message: string): void {
        if (this.state !== 'connected') {
            throw new Error('Not connected');
        }

        if (!this.peer) {
            throw new Error('No peer connected');
        }

        // Queue message on peer
        this.peer.receiveMessage(message);
    }

    /**
     * Simulate receiving a message (used by peer.send)
     */
    private receiveMessage(message: string): void {
        if (this.state !== 'connected') {
            // Queue message for delivery after connect
            this.messageQueue.push(message);
            return;
        }

        if (this.messageDelay > 0) {
            setTimeout(() => {
                this.emit({ type: 'message', data: message });
            }, this.messageDelay);
        } else {
            // Use queueMicrotask for deterministic ordering in tests
            queueMicrotask(() => {
                this.emit({ type: 'message', data: message });
            });
        }
    }

    /**
     * Simulate receiving a message from external source (for testing)
     *
     * Useful for simulating server responses in tests
     */
    simulateReceive(message: string): void {
        this.receiveMessage(message);
    }

    /**
     * Simulate a connection error
     */
    simulateError(error: Error): void {
        this.emit({ type: 'error', error });
    }

    /**
     * Simulate a disconnect
     */
    simulateDisconnect(): void {
        this.setState('disconnected');
        this.emit({ type: 'disconnected' });
    }

    private async flushQueue(): Promise<void> {
        while (this.messageQueue.length > 0) {
            const message = this.messageQueue.shift();
            if (message === undefined) continue;
            if (this.messageDelay > 0) {
                await this.delay(this.messageDelay);
            }
            this.emit({ type: 'message', data: message });
        }
    }

    private delay(ms: number): Promise<void> {
        return new Promise((resolve) => setTimeout(resolve, ms));
    }
}

// ==================== Fake Agent Server ====================

/**
 * Fake agent server for testing
 *
 * Simulates a Python AI agent that responds to step/reset messages
 * with actions.
 *
 * @example
 * ```typescript
 * const [clientTransport, serverTransport] = InMemoryTransport.createPair();
 * const fakeServer = new FakeAgentServer(serverTransport);
 *
 * // Configure action response strategy
 * fakeServer.setActionStrategy((stepData) => {
 *   // Return action based on observation
 *   return 5;
 * });
 *
 * await clientTransport.connect();
 * await serverTransport.connect();
 *
 * // Client sends step, fake server responds with action
 * ```
 */
export class FakeAgentServer {
    private transport: Transport;
    private actionStrategy: (data: StepData) => unknown = () => 0;
    private sessionId = '';
    private noResponse = false;

    constructor(transport: Transport) {
        this.transport = transport;
        this.transport.onEvent((event) => {
            if (event.type === 'message' && typeof event.data === 'string') {
                this.handleMessage(event.data);
            }
        });
    }

    /**
     * Set the action generation strategy
     */
    setActionStrategy(strategy: (data: StepData) => unknown): this {
        this.actionStrategy = strategy;
        return this;
    }

    /**
     * Set no-response mode (for timeout testing)
     */
    setNoResponse(enabled: boolean): this {
        this.noResponse = enabled;
        return this;
    }

    /**
     * Start the fake server (connect the transport)
     */
    async start(): Promise<void> {
        await this.transport.connect();
    }

    /**
     * Stop the fake server
     */
    stop(): void {
        this.transport.disconnect();
    }

    private handleMessage(raw: string): void {
        try {
            const msg = JSON.parse(raw) as {
                v: number;
                type: string;
                sessionId: string;
                seq: number;
                timestamp: number;
                requestId?: string;
                data: unknown;
            };

            // Store session ID from first message
            if (!this.sessionId) {
                this.sessionId = msg.sessionId;
            }

            switch (msg.type) {
                case 'hello':
                    this.handleHello(msg);
                    break;
                case 'reset':
                case 'step':
                    this.handleStep(msg);
                    break;
                case 'ping':
                    this.handlePing(msg);
                    break;
            }
        } catch (e) {
            console.error('[FakeAgentServer] Failed to handle message:', e);
        }
    }

    private handleHello(msg: { requestId?: string }): void {
        const response = {
            v: 1,
            type: 'hello_ack',
            sessionId: this.sessionId,
            seq: 0,
            timestamp: Date.now(),
            requestId: msg.requestId,
            data: {
                serverVersion: '1.0.0',
                accepted: true,
            },
        };
        this.transport.send(JSON.stringify(response));
    }

    private async handleStep(msg: { requestId?: string; data: unknown }): Promise<void> {
        // If noResponse mode, don't send anything
        if (this.noResponse) {
            return;
        }

        const stepData = msg.data as StepData;
        const action = await Promise.resolve(this.actionStrategy(stepData));

        const response = {
            v: 1,
            type: 'action',
            sessionId: this.sessionId,
            seq: 0,
            timestamp: Date.now(),
            requestId: msg.requestId,
            data: { action },
        };
        this.transport.send(JSON.stringify(response));
    }

    private handlePing(msg: { requestId?: string }): void {
        const response = {
            v: 1,
            type: 'pong',
            sessionId: this.sessionId,
            seq: 0,
            timestamp: Date.now(),
            requestId: msg.requestId,
            data: {},
        };
        this.transport.send(JSON.stringify(response));
    }
}

interface StepData {
    observation: Record<string, unknown>;
    reward?: number;
    done?: boolean;
    info?: Record<string, unknown>;
}
