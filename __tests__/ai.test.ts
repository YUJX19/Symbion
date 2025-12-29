/**
 * AI Module Tests
 * Tests for AI interface protocol, message handling, and space validation
 */

import { describe, it, expect, beforeEach, vi } from 'vitest';
import {
    // Protocol
    ProtocolHandler,
    ProtocolError,
    computeSchemaHash,
    createHelloMessage,
    validateHelloAck,
    validateAction,
    validateObservation,
    // Types and constants
    Msg,
    MsgType,
    ErrorCodes,
    generateRequestId,
    type Space,
    type AiInterfaceConfig,
} from '../src/ai';

// ==================== Test Utilities ====================

function createTestSpace(kind: 'box' | 'discrete' | 'multiDiscrete'): Space {
    switch (kind) {
        case 'box':
            return { kind: 'box', shape: [4], low: -1, high: 1 };
        case 'discrete':
            return { kind: 'discrete', n: 5 };
        case 'multiDiscrete':
            return { kind: 'multiDiscrete', nvec: [3, 4, 5] };
    }
}

function createTestConfig(): AiInterfaceConfig {
    return {
        observationSpace: { kind: 'box', shape: [10], low: -Infinity, high: Infinity },
        actionSpace: { kind: 'box', shape: [3], low: -1, high: 1 },
    };
}

// ==================== ProtocolHandler Tests ====================

describe('ProtocolHandler', () => {
    let protocol: ProtocolHandler;
    const sessionId = 'test-session-123';

    beforeEach(() => {
        protocol = new ProtocolHandler(sessionId);
    });

    describe('constructor', () => {
        it('should create handler with session ID', () => {
            expect(protocol.getSessionId()).toBe(sessionId);
        });

        it('should start with sequence number 0', () => {
            expect(protocol.getSeq()).toBe(0);
        });
    });

    describe('createMessage', () => {
        it('should create message with correct structure', () => {
            const msg = protocol.createMessage('step', { obs: [1, 2, 3] });

            expect(msg.v).toBe(1);
            expect(msg.type).toBe('step');
            expect(msg.sessionId).toBe(sessionId);
            expect(msg.seq).toBe(0);
            expect(msg.data).toEqual({ obs: [1, 2, 3] });
            expect(msg.timestamp).toBeGreaterThan(0);
        });

        it('should increment sequence number', () => {
            protocol.createMessage('step', {});
            protocol.createMessage('step', {});
            const msg = protocol.createMessage('step', {});

            expect(msg.seq).toBe(2);
        });

        it('should include request ID when provided', () => {
            const reqId = 'req-123';
            const msg = protocol.createMessage('step', {}, reqId);

            expect(msg.requestId).toBe(reqId);
        });
    });

    describe('generateRequestId', () => {
        it('should generate unique IDs', () => {
            const ids = new Set<string>();
            for (let i = 0; i < 100; i++) {
                ids.add(protocol.generateRequestId());
            }
            expect(ids.size).toBe(100);
        });
    });

    describe('registerRequest / resolveRequest', () => {
        it('should resolve pending request', async () => {
            const reqId = 'req-test';
            const promise = protocol.registerRequest<number>(reqId, 5000);

            expect(protocol.hasPendingRequests()).toBe(true);
            expect(protocol.getPendingCount()).toBe(1);

            protocol.resolveRequest(reqId, 42);

            const result = await promise;
            expect(result).toBe(42);
            expect(protocol.hasPendingRequests()).toBe(false);
        });

        it('should return false for unknown request', () => {
            const resolved = protocol.resolveRequest('unknown-id', 42);
            expect(resolved).toBe(false);
        });

        it('should timeout pending requests', async () => {
            vi.useFakeTimers();

            const reqId = 'req-timeout';
            const promise = protocol.registerRequest(reqId, 100);

            vi.advanceTimersByTime(150);

            await expect(promise).rejects.toThrow(ProtocolError);
            await expect(promise).rejects.toThrow(/timed out/);

            vi.useRealTimers();
        });
    });

    describe('rejectRequest', () => {
        it('should reject pending request', async () => {
            const reqId = 'req-reject';
            const promise = protocol.registerRequest(reqId, 5000);

            protocol.rejectRequest(reqId, new Error('Test error'));

            await expect(promise).rejects.toThrow('Test error');
        });
    });

    describe('rejectAll', () => {
        it('should reject all pending requests', async () => {
            const promise1 = protocol.registerRequest('req-1', 5000);
            const promise2 = protocol.registerRequest('req-2', 5000);

            protocol.rejectAll(new Error('Connection lost'));

            await expect(promise1).rejects.toThrow('Connection lost');
            await expect(promise2).rejects.toThrow('Connection lost');
            expect(protocol.getPendingCount()).toBe(0);
        });
    });

    describe('handleMessage', () => {
        it('should resolve action message with matching request ID', () => {
            const reqId = 'req-action';
            const promise = protocol.registerRequest<number[]>(reqId, 5000);

            const msg: Msg = {
                v: 1,
                type: 'action',
                sessionId,
                seq: 0,
                timestamp: Date.now(),
                requestId: reqId,
                data: { action: [0.5, -0.5] },
            };

            protocol.handleMessage(msg);

            return expect(promise).resolves.toEqual([0.5, -0.5]);
        });

        it('should reject on version mismatch', () => {
            const consoleSpy = vi.spyOn(console, 'warn').mockImplementation(() => { });

            const msg: Msg = {
                v: 2 as 1, // Wrong version
                type: 'step',
                sessionId,
                seq: 0,
                timestamp: Date.now(),
                data: {},
            };

            protocol.handleMessage(msg);

            expect(consoleSpy).toHaveBeenCalledWith(expect.stringContaining('Unsupported protocol version'));
            consoleSpy.mockRestore();
        });

        it('should reject on session ID mismatch', () => {
            const consoleSpy = vi.spyOn(console, 'warn').mockImplementation(() => { });

            const msg: Msg = {
                v: 1,
                type: 'step',
                sessionId: 'wrong-session',
                seq: 0,
                timestamp: Date.now(),
                data: {},
            };

            protocol.handleMessage(msg);

            expect(consoleSpy).toHaveBeenCalledWith(expect.stringContaining('Session ID mismatch'));
            consoleSpy.mockRestore();
        });

        it('should handle error messages', async () => {
            const reqId = 'req-error';
            const promise = protocol.registerRequest(reqId, 5000);

            const msg: Msg = {
                v: 1,
                type: 'error',
                sessionId,
                seq: 0,
                timestamp: Date.now(),
                requestId: reqId,
                data: {
                    code: ErrorCodes.INVALID_ACTION,
                    message: 'Invalid action format',
                },
            };

            protocol.handleMessage(msg);

            await expect(promise).rejects.toThrow(ProtocolError);
            await expect(promise).rejects.toThrow('Invalid action format');
        });
    });

    describe('onMessage / offMessage', () => {
        it('should register and call message handlers', () => {
            const handler = vi.fn();
            protocol.onMessage('step', handler);

            const msg: Msg = {
                v: 1,
                type: 'step',
                sessionId,
                seq: 0,
                timestamp: Date.now(),
                data: { test: true },
            };

            protocol.handleMessage(msg);

            expect(handler).toHaveBeenCalledWith(msg);
        });

        it('should remove message handlers', () => {
            const handler = vi.fn();
            protocol.onMessage('step', handler);
            protocol.offMessage('step', handler);

            const msg: Msg = {
                v: 1,
                type: 'step',
                sessionId,
                seq: 0,
                timestamp: Date.now(),
                data: {},
            };

            protocol.handleMessage(msg);

            expect(handler).not.toHaveBeenCalled();
        });
    });

    describe('parseMessage', () => {
        it('should parse valid JSON message', () => {
            const raw = JSON.stringify({
                v: 1,
                type: 'action',
                sessionId,
                seq: 0,
                timestamp: Date.now(),
                data: { action: [1] },
            });

            const msg = protocol.parseMessage(raw);
            expect(msg.type).toBe('action');
        });

        it('should throw on invalid JSON', () => {
            expect(() => protocol.parseMessage('not json')).toThrow(ProtocolError);
        });

        it('should throw on wrong version', () => {
            const raw = JSON.stringify({ v: 99, type: 'step' });
            expect(() => protocol.parseMessage(raw)).toThrow(ProtocolError);
            expect(() => protocol.parseMessage(raw)).toThrow(/Unsupported protocol version/);
        });

        it('should throw on missing type', () => {
            const raw = JSON.stringify({ v: 1 });
            expect(() => protocol.parseMessage(raw)).toThrow(ProtocolError);
            expect(() => protocol.parseMessage(raw)).toThrow(/type is required/);
        });
    });

    describe('serializeMessage', () => {
        it('should serialize message to JSON', () => {
            const msg = protocol.createMessage('step', { obs: [1, 2] });
            const json = protocol.serializeMessage(msg);

            expect(JSON.parse(json)).toEqual(msg);
        });
    });
});

// ==================== Schema Hash Tests ====================

describe('computeSchemaHash', () => {
    it('should compute consistent hash for same spaces', () => {
        const obs: Space = { kind: 'box', shape: [4], low: -1, high: 1 };
        const action: Space = { kind: 'discrete', n: 3 };

        const hash1 = computeSchemaHash(obs, action);
        const hash2 = computeSchemaHash(obs, action);

        expect(hash1).toBe(hash2);
    });

    it('should compute different hash for different spaces', () => {
        const obs: Space = { kind: 'box', shape: [4], low: -1, high: 1 };
        const action1: Space = { kind: 'discrete', n: 3 };
        const action2: Space = { kind: 'discrete', n: 5 };

        const hash1 = computeSchemaHash(obs, action1);
        const hash2 = computeSchemaHash(obs, action2);

        expect(hash1).not.toBe(hash2);
    });

    it('should return 8-character hex string', () => {
        const hash = computeSchemaHash(
            { kind: 'box', shape: [10], low: 0, high: 1 },
            { kind: 'box', shape: [3], low: -1, high: 1 }
        );

        expect(hash).toMatch(/^[0-9a-f]{8}$/);
    });
});

// ==================== Handshake Tests ====================

describe('createHelloMessage', () => {
    it('should create hello message with correct structure', () => {
        const protocol = new ProtocolHandler('test-session');
        const config = createTestConfig();
        const reqId = 'hello-req';

        const msg = createHelloMessage(protocol, config, reqId);

        expect(msg.type).toBe('hello');
        expect(msg.requestId).toBe(reqId);
        expect(msg.data.clientVersion).toBe('1.0.0');
        expect(msg.data.observationSpace).toEqual(config.observationSpace);
        expect(msg.data.actionSpace).toEqual(config.actionSpace);
        expect(msg.data.schemaHash).toBeDefined();
        expect(msg.data.capabilities).toContain('timeout');
    });
});

describe('validateHelloAck', () => {
    it('should pass for accepted connection', () => {
        expect(() =>
            validateHelloAck({ accepted: true, serverVersion: '1.0.0' })
        ).not.toThrow();
    });

    it('should throw for rejected connection', () => {
        expect(() =>
            validateHelloAck({
                accepted: false,
                serverVersion: '1.0.0',
                error: 'Server busy',
            })
        ).toThrow(ProtocolError);
    });

    it('should throw for schema mismatch', () => {
        expect(() =>
            validateHelloAck(
                { accepted: true, serverVersion: '1.0.0', schemaHash: 'abcd1234' },
                'different123'
            )
        ).toThrow(ProtocolError);
        expect(() =>
            validateHelloAck(
                { accepted: true, serverVersion: '1.0.0', schemaHash: 'abcd1234' },
                'different123'
            )
        ).toThrow(/Schema hash mismatch/);
    });
});

// ==================== Space Validation Tests ====================

describe('validateAction', () => {
    describe('discrete space', () => {
        const space: Space = { kind: 'discrete', n: 5 };

        it('should accept valid discrete action', () => {
            expect(validateAction(0, space)).toBe(true);
            expect(validateAction(4, space)).toBe(true);
        });

        it('should reject out of range action', () => {
            expect(validateAction(5, space)).toBe(false);
            expect(validateAction(-1, space)).toBe(false);
        });

        it('should reject non-integer action', () => {
            expect(validateAction(1.5, space)).toBe(false);
        });
    });

    describe('box space', () => {
        const space: Space = { kind: 'box', shape: [3], low: -1, high: 1 };

        it('should accept valid box action', () => {
            expect(validateAction([0, 0.5, -0.5], space)).toBe(true);
        });

        it('should reject out of bounds values', () => {
            expect(validateAction([0, 0, 1.5], space)).toBe(false);
            expect(validateAction([0, -2, 0], space)).toBe(false);
        });

        it('should reject wrong length array', () => {
            expect(validateAction([0, 0], space)).toBe(false);
            expect(validateAction([0, 0, 0, 0], space)).toBe(false);
        });
    });

    describe('multiDiscrete space', () => {
        const space: Space = { kind: 'multiDiscrete', nvec: [3, 4, 2] };

        it('should accept valid multiDiscrete action', () => {
            expect(validateAction([0, 0, 0], space)).toBe(true);
            expect(validateAction([2, 3, 1], space)).toBe(true);
        });

        it('should reject out of range values', () => {
            expect(validateAction([3, 0, 0], space)).toBe(false);
            expect(validateAction([0, 4, 0], space)).toBe(false);
        });
    });
});

describe('validateObservation', () => {
    it('should validate box observation', () => {
        const space: Space = { kind: 'box', shape: [2, 3], low: 0, high: 10 };

        expect(
            validateObservation(
                [
                    [1, 2, 3],
                    [4, 5, 6],
                ],
                space
            )
        ).toBe(true);

        expect(
            validateObservation(
                [
                    [1, 2, 15],
                    [4, 5, 6],
                ],
                space
            )
        ).toBe(false);
    });

    it('should validate discrete observation', () => {
        const space: Space = { kind: 'discrete', n: 5 };

        expect(validateObservation(0, space)).toBe(true);
        expect(validateObservation(4, space)).toBe(true);
        expect(validateObservation(5, space)).toBe(false);
        expect(validateObservation(-1, space)).toBe(false);
        expect(validateObservation(2.5, space)).toBe(false);
    });

    it('should validate multiDiscrete observation', () => {
        const space: Space = { kind: 'multiDiscrete', nvec: [3, 4, 5] };

        expect(validateObservation([0, 0, 0], space)).toBe(true);
        expect(validateObservation([2, 3, 4], space)).toBe(true);
        expect(validateObservation([3, 0, 0], space)).toBe(false); // 3 >= nvec[0]
        expect(validateObservation([0, 4, 0], space)).toBe(false); // 4 >= nvec[1]
        expect(validateObservation([0, 0], space)).toBe(false); // wrong length
    });

    it('should validate dict observation', () => {
        const space: Space = {
            kind: 'dict',
            spaces: {
                position: { kind: 'box', shape: [3], low: 0, high: 100 },
                velocity: { kind: 'box', shape: [3], low: -10, high: 10 },
                status: { kind: 'discrete', n: 3 },
            },
        };

        expect(
            validateObservation(
                {
                    position: [50, 50, 50],
                    velocity: [0, 0, 0],
                    status: 1,
                },
                space
            )
        ).toBe(true);

        // Missing key
        expect(
            validateObservation(
                {
                    position: [50, 50, 50],
                    velocity: [0, 0, 0],
                },
                space
            )
        ).toBe(false);

        // Out of range value
        expect(
            validateObservation(
                {
                    position: [150, 50, 50], // 150 > 100
                    velocity: [0, 0, 0],
                    status: 1,
                },
                space
            )
        ).toBe(false);
    });

    it('should validate tuple observation', () => {
        const space: Space = {
            kind: 'tuple',
            spaces: [
                { kind: 'box', shape: [2], low: -1, high: 1 },
                { kind: 'discrete', n: 5 },
            ],
        };

        expect(validateObservation([[0.5, -0.5], 3], space)).toBe(true);
        expect(validateObservation([[0, 0], 0], space)).toBe(true);
        expect(validateObservation([[2, 0], 0], space)).toBe(false); // 2 > 1
        expect(validateObservation([[0, 0], 5], space)).toBe(false); // 5 >= n
        expect(validateObservation([[0, 0]], space)).toBe(false); // wrong length
    });

    it('should validate nested dict observation', () => {
        const space: Space = {
            kind: 'dict',
            spaces: {
                uav: {
                    kind: 'dict',
                    spaces: {
                        pos: { kind: 'box', shape: [3], low: 0, high: 1000 },
                        vel: { kind: 'box', shape: [3], low: -20, high: 20 },
                    },
                },
                comm: {
                    kind: 'dict',
                    spaces: {
                        sinr: { kind: 'box', shape: [1], low: -20, high: 40 },
                        mcs: { kind: 'discrete', n: 29 },
                    },
                },
            },
        };

        expect(
            validateObservation(
                {
                    uav: {
                        pos: [100, 200, 50],
                        vel: [5, -2, 0],
                    },
                    comm: {
                        sinr: [15.5],
                        mcs: 12,
                    },
                },
                space
            )
        ).toBe(true);

        // Invalid nested value
        expect(
            validateObservation(
                {
                    uav: {
                        pos: [100, 200, 50],
                        vel: [5, -2, 0],
                    },
                    comm: {
                        sinr: [50], // 50 > 40
                        mcs: 12,
                    },
                },
                space
            )
        ).toBe(false);
    });

    it('should validate 1D box observation with scalar bounds', () => {
        const space: Space = { kind: 'box', shape: [4], low: -1, high: 1 };

        expect(validateObservation([0, 0.5, -0.5, 1], space)).toBe(true);
        expect(validateObservation([0, 0.5, -0.5, 1.1], space)).toBe(false);
        expect(validateObservation([0, 0.5, -0.5], space)).toBe(false); // wrong length
    });

    it('should validate box with per-dimension bounds', () => {
        const space: Space = {
            kind: 'box',
            shape: [3],
            low: [0, -10, -100],
            high: [100, 10, 100],
        };

        expect(validateObservation([50, 0, 0], space)).toBe(true);
        expect(validateObservation([0, -10, -100], space)).toBe(true);
        expect(validateObservation([100, 10, 100], space)).toBe(true);
        expect(validateObservation([-1, 0, 0], space)).toBe(false); // -1 < low[0]
        expect(validateObservation([0, 11, 0], space)).toBe(false); // 11 > high[1]
    });
});

// ==================== generateRequestId Tests ====================

describe('generateRequestId', () => {
    it('should generate unique IDs', () => {
        const ids = new Set<string>();
        for (let i = 0; i < 1000; i++) {
            ids.add(generateRequestId());
        }
        expect(ids.size).toBe(1000);
    });

    it('should be non-empty string', () => {
        const id = generateRequestId();
        expect(typeof id).toBe('string');
        expect(id.length).toBeGreaterThan(0);
    });
});

// ==================== ProtocolError Tests ====================

describe('ProtocolError', () => {
    it('should have correct properties', () => {
        const error = new ProtocolError('TEST_CODE', 'Test message', { extra: 'data' });

        expect(error.name).toBe('ProtocolError');
        expect(error.code).toBe('TEST_CODE');
        expect(error.message).toBe('Test message');
        expect(error.details).toEqual({ extra: 'data' });
    });

    it('should be instanceof Error', () => {
        const error = new ProtocolError('CODE', 'msg');
        expect(error instanceof Error).toBe(true);
    });
});

// ==================== InMemoryTransport Tests ====================

// Import InMemoryTransport and FakeAgentServer dynamically for testing
describe('InMemoryTransport', () => {
    let InMemoryTransport: any;
    let FakeAgentServer: any;

    beforeEach(async () => {
        const transportModule = await import('../src/ai/interface/transport/memory');
        InMemoryTransport = transportModule.InMemoryTransport;
        FakeAgentServer = transportModule.FakeAgentServer;
    });

    describe('createPair', () => {
        it('should create connected transport pair', () => {
            const [client, server] = InMemoryTransport.createPair();

            expect(client).toBeDefined();
            expect(server).toBeDefined();
        });

        it('should allow communication between pair', async () => {
            const [client, server] = InMemoryTransport.createPair();
            const receivedMessages: string[] = [];

            server.onEvent((e: any) => {
                if (e.type === 'message') {
                    receivedMessages.push(e.data);
                }
            });

            await client.connect();
            await server.connect();

            client.send('{"type":"test","data":"hello"}');
            await client.flushQueue();

            expect(receivedMessages).toContain('{"type":"test","data":"hello"}');
        });
    });

    describe('connect/disconnect', () => {
        it('should emit connect event on connect', async () => {
            const [client] = InMemoryTransport.createPair();
            const events: string[] = [];

            client.onEvent((e: any) => {
                events.push(e.type);
            });

            await client.connect();

            expect(events).toContain('connected');
        });

        it('should emit close event on disconnect', async () => {
            const [client] = InMemoryTransport.createPair();
            const events: string[] = [];

            client.onEvent((e: any) => {
                events.push(e.type);
            });

            await client.connect();
            client.disconnect();

            expect(events).toContain('connected');
            expect(events).toContain('disconnected');
        });
    });

    describe('latency simulation', () => {
        it('should support connection delay', async () => {
            const [client] = InMemoryTransport.createPair();
            client.setConnectDelay(10);

            const start = Date.now();
            await client.connect();
            const elapsed = Date.now() - start;

            expect(elapsed).toBeGreaterThanOrEqual(9); // Allow small timing variance
        });

        it('should support message delay configuration', async () => {
            const [client, server] = InMemoryTransport.createPair();

            // Verify setMessageDelay is chainable and doesn't throw
            const result = client.setMessageDelay(10);
            expect(result).toBe(client);

            const receivedMessages: string[] = [];

            server.onEvent((e: any) => {
                if (e.type === 'message') {
                    receivedMessages.push(e.data);
                }
            });

            await client.connect();
            await server.connect();

            client.send('test-message');
            await client.flushQueue();

            // Message should be received (timing is not precisely testable in all environments)
            expect(receivedMessages).toContain('test-message');
        });
    });

    describe('error simulation', () => {
        it('should simulate errors', async () => {
            const [client] = InMemoryTransport.createPair();
            const errors: Error[] = [];

            client.onEvent((e: any) => {
                if (e.type === 'error' && e.error) {
                    errors.push(e.error);
                }
            });

            await client.connect();
            client.simulateError(new Error('Test error'));

            expect(errors).toHaveLength(1);
            expect(errors[0].message).toBe('Test error');
        });

        it('should simulate disconnect', async () => {
            const [client] = InMemoryTransport.createPair();
            const events: string[] = [];

            client.onEvent((e: any) => {
                events.push(e.type);
            });

            await client.connect();
            client.simulateDisconnect();

            expect(events).toContain('disconnected');
        });
    });
});

// ==================== FakeAgentServer Tests ====================

describe('FakeAgentServer', () => {
    let InMemoryTransport: any;
    let FakeAgentServer: any;

    beforeEach(async () => {
        const transportModule = await import('../src/ai/interface/transport/memory');
        InMemoryTransport = transportModule.InMemoryTransport;
        FakeAgentServer = transportModule.FakeAgentServer;
    });

    it('should respond to hello messages', async () => {
        const [client, server] = InMemoryTransport.createPair();
        const fakeServer = new FakeAgentServer(server);

        const responses: string[] = [];
        client.onEvent((e: any) => {
            if (e.type === 'message') {
                responses.push(e.data);
            }
        });

        await fakeServer.start();
        await client.connect();

        // Send hello message
        const helloMsg = JSON.stringify({
            v: 1,
            type: 'hello',
            sessionId: 'test',
            seq: 0,
            timestamp: Date.now(),
            requestId: 'req-1',
            data: {
                clientVersion: '1.0.0',
                observationSpace: { kind: 'box', shape: [4], low: -1, high: 1 },
                actionSpace: { kind: 'discrete', n: 5 },
            },
        });

        client.send(helloMsg);
        await client.flushQueue();
        await server.flushQueue();

        // Check for hello_ack response
        const ackResponse = responses.find(r => {
            const parsed = JSON.parse(r);
            return parsed.type === 'hello_ack';
        });

        expect(ackResponse).toBeDefined();
        const ack = JSON.parse(ackResponse!);
        expect(ack.data.accepted).toBe(true);

        fakeServer.stop();
    });

    it('should respond to step messages with action', async () => {
        const [client, server] = InMemoryTransport.createPair();
        const fakeServer = new FakeAgentServer(server);

        // Set custom action strategy
        fakeServer.setActionStrategy((data: any) => {
            return data.observation && data.observation.value ? data.observation.value * 2 : 42;
        });

        const responses: string[] = [];
        client.onEvent((e: any) => {
            if (e.type === 'message') {
                responses.push(e.data);
            }
        });

        await fakeServer.start();
        await client.connect();

        // Send step message
        const stepMsg = JSON.stringify({
            v: 1,
            type: 'step',
            sessionId: 'test',
            seq: 0,
            timestamp: Date.now(),
            requestId: 'req-step',
            data: {
                observation: { value: 10 },
                reward: 1.0,
                done: false,
            },
        });

        client.send(stepMsg);
        await client.flushQueue();
        await server.flushQueue();

        // Wait a bit for async processing
        await new Promise(resolve => setTimeout(resolve, 10));

        // Check for action response
        const actionResponse = responses.find(r => {
            const parsed = JSON.parse(r);
            return parsed.type === 'action';
        });

        expect(actionResponse).toBeDefined();
        const action = JSON.parse(actionResponse!);
        expect(action.data.action).toBe(20); // 10 * 2

        fakeServer.stop();
    });

    it('should respond to ping messages', async () => {
        const [client, server] = InMemoryTransport.createPair();
        const fakeServer = new FakeAgentServer(server);

        const responses: string[] = [];
        client.onEvent((e: any) => {
            if (e.type === 'message') {
                responses.push(e.data);
            }
        });

        await fakeServer.start();
        await client.connect();

        // Send ping message
        const pingMsg = JSON.stringify({
            v: 1,
            type: 'ping',
            sessionId: 'test',
            seq: 0,
            timestamp: Date.now(),
            requestId: 'req-ping',
            data: {},
        });

        client.send(pingMsg);
        await client.flushQueue();
        await server.flushQueue();

        // Check for pong response
        const pongResponse = responses.find(r => {
            const parsed = JSON.parse(r);
            return parsed.type === 'pong';
        });

        expect(pongResponse).toBeDefined();

        fakeServer.stop();
    });

    it('should support no-response mode for timeout testing', async () => {
        const [client, server] = InMemoryTransport.createPair();
        const fakeServer = new FakeAgentServer(server);
        fakeServer.setNoResponse(true);

        const responses: string[] = [];
        client.onEvent((e: any) => {
            if (e.type === 'message') {
                responses.push(e.data);
            }
        });

        await fakeServer.start();
        await client.connect();

        // Send step message
        const stepMsg = JSON.stringify({
            v: 1,
            type: 'step',
            sessionId: 'test',
            seq: 0,
            timestamp: Date.now(),
            requestId: 'req-no-response',
            data: { observation: {}, reward: 0, done: false },
        });

        client.send(stepMsg);
        await client.flushQueue();
        await server.flushQueue();

        // Should not receive action response
        const actionResponse = responses.find(r => {
            const parsed = JSON.parse(r);
            return parsed.type === 'action' && parsed.requestId === 'req-no-response';
        });

        expect(actionResponse).toBeUndefined();

        fakeServer.stop();
    });
});

// ==================== RL Observation Utilities Tests ====================

describe('RL Observation Utilities', () => {
    let defaultUavObservation: any;
    let minimalUavObservation: any;
    let fullUavObservation: any;
    let normalizeObservation: any;
    let ObservationBuilder: any;

    beforeEach(async () => {
        const obsModule = await import('../src/ai/interface/env/rlObservation');
        defaultUavObservation = obsModule.defaultUavObservation;
        minimalUavObservation = obsModule.minimalUavObservation;
        fullUavObservation = obsModule.fullUavObservation;
        normalizeObservation = obsModule.normalizeObservation;
        ObservationBuilder = obsModule.ObservationBuilder;
    });

    describe('defaultUavObservation', () => {
        it('should extract default observation from UavState', () => {
            const state = {
                uav: { position: [100, 200, 50], velocity: [5, -2, 0], battery: 0.85 },
                channel: { sinrDb: 15.5, rate: 50.2, bler: 0.01 },
                mission: { target: [500, 500, 100], waypointError: [10, 5, 2] },
                network: { servingBsId: 3 },
            };

            const obs = defaultUavObservation(state);

            expect(obs.uav_pos).toEqual([100, 200, 50]);
            expect(obs.uav_vel).toEqual([5, -2, 0]);
            expect(obs.battery).toBe(0.85);
            expect(obs.sinr_db).toBe(15.5);
            expect(obs.rate_mbps).toBe(50.2);
            expect(obs.bler).toBe(0.01);
            expect(obs.wp_error).toEqual([10, 5, 2]);
            expect(obs.bs_id).toBe(3);
        });

        it('should use default values for missing fields', () => {
            const state = {
                uav: { position: [0, 0, 0], velocity: [0, 0, 0] },
                channel: { sinrDb: 10, rate: 20 },
            };

            const obs = defaultUavObservation(state);

            expect(obs.battery).toBe(1.0);
            expect(obs.bler).toBe(0);
            expect(obs.wp_error).toEqual([0, 0, 0]);
            expect(obs.bs_id).toBe(0);
        });
    });

    describe('minimalUavObservation', () => {
        it('should extract minimal observation', () => {
            const state = {
                uav: { position: [100, 200, 50], velocity: [5, -2, 0], battery: 0.85 },
                channel: { sinrDb: 15.5, rate: 50.2, bler: 0.01 },
            };

            const obs = minimalUavObservation(state);

            expect(obs.pos).toEqual([100, 200, 50]);
            expect(obs.sinr).toBe(15.5);
            expect(Object.keys(obs)).toHaveLength(2);
        });
    });

    describe('normalizeObservation', () => {
        it('should normalize box observation to [-1, 1]', () => {
            const space = { kind: 'box' as const, shape: [3], low: 0, high: 100 };
            const obs = [0, 50, 100];

            const normalized = normalizeObservation(obs, space);

            expect(normalized[0]).toBeCloseTo(-1);
            expect(normalized[1]).toBeCloseTo(0);
            expect(normalized[2]).toBeCloseTo(1);
        });

        it('should normalize discrete observation to [0, 1]', () => {
            const space = { kind: 'discrete' as const, n: 5 };

            expect(normalizeObservation(0, space)).toBeCloseTo(0);
            expect(normalizeObservation(4, space)).toBeCloseTo(1);
            expect(normalizeObservation(2, space)).toBeCloseTo(0.5);
        });

        it('should normalize dict observation recursively', () => {
            const space = {
                kind: 'dict' as const,
                spaces: {
                    pos: { kind: 'box' as const, shape: [3], low: 0, high: 100 },
                    status: { kind: 'discrete' as const, n: 3 },
                },
            };

            const obs = { pos: [50, 50, 50], status: 1 };
            const normalized = normalizeObservation(obs, space);

            expect(normalized.pos[0]).toBeCloseTo(0);
            expect(normalized.pos[1]).toBeCloseTo(0);
            expect(normalized.pos[2]).toBeCloseTo(0);
            expect(normalized.status).toBeCloseTo(0.5);
        });
    });

    describe('ObservationBuilder', () => {
        it('should build custom observation extractor', () => {
            const extractor = new ObservationBuilder()
                .add('position', (s: { x: number; y: number }) => [s.x, s.y])
                .add('sum', (s: { x: number; y: number }) => s.x + s.y)
                .build();

            const state = { x: 10, y: 20 };
            const obs = extractor(state);

            expect(obs.position).toEqual([10, 20]);
            expect(obs.sum).toBe(30);
        });
    });
});

// ==================== RL Action Utilities Tests ====================

describe('RL Action Utilities', () => {
    let defaultUavActionMapper: any;
    let clampAction: any;
    let getMcsSpectralEfficiency: any;
    let findOptimalMcs: any;
    let ActionBuilder: any;

    beforeEach(async () => {
        const actModule = await import('../src/ai/interface/env/rlAction');
        defaultUavActionMapper = actModule.defaultUavActionMapper;
        clampAction = actModule.clampAction;
        getMcsSpectralEfficiency = actModule.getMcsSpectralEfficiency;
        findOptimalMcs = actModule.findOptimalMcs;
        ActionBuilder = actModule.ActionBuilder;
    });

    describe('defaultUavActionMapper', () => {
        it('should map dict action to UavCommand', () => {
            const action = {
                flight: [0.5, -0.2, 0.1],
                txPower: [0.8],
                mcs: 15,
            };

            const cmd = defaultUavActionMapper(action);

            expect(cmd.flight.velocityDelta).toEqual([2.5, -1, 0.5]);
            expect(cmd.comm.txPowerDbm).toBeCloseTo(18.4);
            expect(cmd.comm.mcsIndex).toBe(15);
        });

        it('should map array action to flight command', () => {
            const action = [0.5, -0.5, 1.0];

            const cmd = defaultUavActionMapper(action);

            expect(cmd.flight.velocityDelta).toEqual([2.5, -2.5, 5]);
            expect(cmd.comm).toBeUndefined();
        });

        it('should map discrete action to velocity', () => {
            // 0: hover, 1: forward, 2: backward, etc.
            const hoverCmd = defaultUavActionMapper(0);
            expect(hoverCmd.flight.velocityDelta).toEqual([0, 0, 0]);

            const forwardCmd = defaultUavActionMapper(1);
            expect(forwardCmd.flight.velocityDelta).toEqual([5, 0, 0]);

            const upCmd = defaultUavActionMapper(5);
            expect(upCmd.flight.velocityDelta).toEqual([0, 0, 5]);
        });

        it('should apply custom config', () => {
            const action = { flight: [1, 1, 1] };
            const config = { maxVelocityDelta: 10 };

            const cmd = defaultUavActionMapper(action, config);

            expect(cmd.flight.velocityDelta).toEqual([10, 10, 10]);
        });
    });

    describe('clampAction', () => {
        it('should clamp discrete action', () => {
            const space = { kind: 'discrete' as const, n: 5 };

            expect(clampAction(-1, space)).toBe(0);
            expect(clampAction(10, space)).toBe(4);
            expect(clampAction(2.7, space)).toBe(2);
        });

        it('should clamp box action', () => {
            const space = { kind: 'box' as const, shape: [3], low: -1, high: 1 };

            expect(clampAction([0, 2, -3], space)).toEqual([0, 1, -1]);
        });

        it('should clamp multiDiscrete action', () => {
            const space = { kind: 'multiDiscrete' as const, nvec: [3, 5] };

            expect(clampAction([5, 10], space)).toEqual([2, 4]);
            expect(clampAction([-1, -1], space)).toEqual([0, 0]);
        });

        it('should clamp dict action recursively', () => {
            const space = {
                kind: 'dict' as const,
                spaces: {
                    move: { kind: 'box' as const, shape: [2], low: -1, high: 1 },
                    act: { kind: 'discrete' as const, n: 3 },
                },
            };

            const action = { move: [2, -2], act: 5 };
            const clamped = clampAction(action, space);

            expect(clamped.move).toEqual([1, -1]);
            expect(clamped.act).toBe(2);
        });
    });

    describe('getMcsSpectralEfficiency', () => {
        it('should return spectral efficiency for valid MCS', () => {
            expect(getMcsSpectralEfficiency(0)).toBeCloseTo(0.2344, 3);
            expect(getMcsSpectralEfficiency(10)).toBeCloseTo(2.5703, 3);
            expect(getMcsSpectralEfficiency(22)).toBeCloseTo(5.8906, 3);
        });

        it('should clamp out of range MCS', () => {
            expect(getMcsSpectralEfficiency(-1)).toBeCloseTo(0.2344, 3);
            expect(getMcsSpectralEfficiency(100)).toBeCloseTo(5.8906, 3);
        });
    });

    describe('findOptimalMcs', () => {
        it('should find optimal MCS for given SINR', () => {
            expect(findOptimalMcs(-10)).toBe(0);
            expect(findOptimalMcs(5)).toBeGreaterThan(0);
            expect(findOptimalMcs(20)).toBeGreaterThan(10);
        });

        it('should apply margin for lower BLER target', () => {
            const mcsDefaultBler = findOptimalMcs(10, 0.1);
            const mcsLowBler = findOptimalMcs(10, 0.001);

            expect(mcsLowBler).toBeLessThan(mcsDefaultBler);
        });
    });

    describe('ActionBuilder', () => {
        it('should build custom action mapper', () => {
            const mapper = new ActionBuilder()
                .map('vel', (v: unknown) => ({ velocity: (v as number[]).map((x: number) => x * 10) }))
                .map('pow', (v: unknown) => ({ power: (v as number) * 23 }))
                .build();

            const action = { vel: [0.5, 0.5, 0.5], pow: 0.8 };
            const cmd = mapper(action);

            expect(cmd.velocity).toEqual([5, 5, 5]);
            expect(cmd.power).toBeCloseTo(18.4);
        });
    });
});

// ==================== UavGymEnvClient Integration Tests ====================

describe('UavGymEnvClient Integration', () => {
    let UavGymEnvClient: any;
    let InMemoryTransport: any;
    let FakeAgentServer: any;

    beforeEach(async () => {
        const envModule = await import('../src/ai/interface/env/rlEnvClient');
        const transportModule = await import('../src/ai/interface/transport/memory');
        UavGymEnvClient = envModule.UavGymEnvClient;
        InMemoryTransport = transportModule.InMemoryTransport;
        FakeAgentServer = transportModule.FakeAgentServer;
    });

    it('should complete full step cycle', async () => {
        const [clientTransport, serverTransport] = InMemoryTransport.createPair();
        const fakeServer = new FakeAgentServer(serverTransport);
        fakeServer.setActionStrategy(() => ({ move: [0.1, 0.2, 0.3], mcs: 10 }));

        const client = new UavGymEnvClient(
            {
                host: 'memory',
                port: 0,
                observationSpace: { kind: 'box', shape: [6], low: -1, high: 1 },
                actionSpace: {
                    kind: 'dict',
                    spaces: {
                        move: { kind: 'box', shape: [3], low: -1, high: 1 },
                        mcs: { kind: 'discrete', n: 29 },
                    },
                },
            },
            clientTransport
        );

        await fakeServer.start();
        await client.connect();

        // Reset
        const initialObs = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6];
        const firstAction = await client.reset(initialObs);
        expect(firstAction).toBeDefined();
        expect(firstAction.mcs).toBe(10);

        // Step
        const obs = [0.2, 0.3, 0.4, 0.5, 0.6, 0.7];
        const action = await client.step(obs, 1.5, false);
        expect(action).toBeDefined();
        expect(action.move).toEqual([0.1, 0.2, 0.3]);

        // Final step (done)
        const finalAction = await client.step(obs, 2.0, true);
        expect(finalAction).toBeDefined();

        client.disconnect();
        fakeServer.stop();
    });

    it('should throw on concurrent steps in strict mode', async () => {
        const [clientTransport, serverTransport] = InMemoryTransport.createPair();
        const fakeServer = new FakeAgentServer(serverTransport);
        // Delay response to simulate slow agent
        fakeServer.setActionStrategy(() => {
            return new Promise((resolve) => setTimeout(() => resolve(0), 50));
        });

        const client = new UavGymEnvClient(
            {
                host: 'memory',
                port: 0,
                observationSpace: { kind: 'box', shape: [3], low: -1, high: 1 },
                actionSpace: { kind: 'discrete', n: 5 },
                strictStep: true,
            },
            clientTransport
        );

        await fakeServer.start();
        await client.connect();

        // Reset first
        await client.reset([0, 0, 0]);

        // Start first step (don't await)
        const step1Promise = client.step([0.1, 0.2, 0.3], 1.0, false);

        // Try second step immediately
        await expect(client.step([0.4, 0.5, 0.6], 1.0, false)).rejects.toThrow(
            /Previous step not completed/
        );

        // Wait for first step to complete
        await step1Promise;

        client.disconnect();
        fakeServer.stop();
    });

    it('should report step in progress correctly', async () => {
        const [clientTransport, serverTransport] = InMemoryTransport.createPair();
        const fakeServer = new FakeAgentServer(serverTransport);

        const client = new UavGymEnvClient(
            {
                host: 'memory',
                port: 0,
                observationSpace: { kind: 'box', shape: [3], low: -1, high: 1 },
                actionSpace: { kind: 'discrete', n: 5 },
            },
            clientTransport
        );

        await fakeServer.start();
        await client.connect();

        expect(client.isStepInProgress()).toBe(false);

        await client.reset([0, 0, 0]);
        expect(client.isStepInProgress()).toBe(false);

        await client.step([0.1, 0.2, 0.3], 1.0, false);
        expect(client.isStepInProgress()).toBe(false);

        client.disconnect();
        fakeServer.stop();
    });

    it('should provide observation and action space accessors', async () => {
        const [clientTransport] = InMemoryTransport.createPair();

        const obsSpace = { kind: 'box' as const, shape: [6], low: -1, high: 1 };
        const actSpace = { kind: 'discrete' as const, n: 10 };

        const client = new UavGymEnvClient(
            {
                host: 'memory',
                port: 0,
                observationSpace: obsSpace,
                actionSpace: actSpace,
            },
            clientTransport
        );

        // Use deprecated methods (should still work)
        expect(client.getObservationSpace()).toEqual(obsSpace);
        expect(client.getActionSpace()).toEqual(actSpace);

        // Use new methods
        expect(client.getInputSpace()).toEqual(obsSpace);
        expect(client.getOutputSpace()).toEqual(actSpace);
    });

    it('should emit events correctly', async () => {
        const [clientTransport, serverTransport] = InMemoryTransport.createPair();
        const fakeServer = new FakeAgentServer(serverTransport);

        const client = new UavGymEnvClient(
            {
                host: 'memory',
                port: 0,
                observationSpace: { kind: 'box', shape: [3], low: -1, high: 1 },
                actionSpace: { kind: 'discrete', n: 5 },
            },
            clientTransport
        );

        const events: string[] = [];
        client.onEvent((e: any) => events.push(e.type));

        await fakeServer.start();
        await client.connect();

        expect(events).toContain('connected');

        await client.reset([0, 0, 0]);
        expect(events).toContain('query_complete');

        client.disconnect();
        expect(events).toContain('disconnected');

        fakeServer.stop();
    });
});
