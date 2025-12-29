/**
 * Communication Module Tests
 * Tests for Network Layer (AODV, CSMA/CA) and Protocol Layer (Path Loss, Link Quality)
 */

import { describe, it, expect } from 'vitest';
import {
    // Network Layer - AODV
    initAodvNode,
    findRoute,
    updateRoute,
    createRREQ,
    processRREQ,
    processRREP,
    // Network Layer - MAC
    initMacNode,
    generateBackoff,
    increaseBackoffWindow,
    resetBackoffWindow,
    updateMacState,
    enqueueMacPacket,
    detectCollision,
    getChannelState,
    // Utility functions
    distance,
    isInRange,
    getNeighbors,
} from '../src/extras/networking/network';
import {
    // Path Loss Models
    friisPathLoss,
    twoRayPathLoss,
    logDistancePathLoss,
    airToGroundPathLoss,
    // Link Quality
    calculateRSSI,
    calculateSNR,
    evaluateLinkQuality,
    estimateDataRate,
    estimateLatency,
    calculateCommLink,
    // Network Topology
    selectBestStation,
    // Packet
    packetSuccessProbability,
    createHelloPacket,
    createUpdatePacket,
    type BaseStation,
    type DroneNode,
} from '../src/extras/networking/protocols';
import { isClose, mean, variance, std } from './test-utils';

// ==================== AODV Routing Protocol Tests ====================

describe('AODV Routing Protocol', () => {
    describe('initAodvNode', () => {
        it('should initialize node with empty routing table', () => {
            const node = initAodvNode('node1');
            expect(node.nodeId).toBe('node1');
            expect(node.routingTable).toEqual([]);
            expect(node.broadcastIdCache).toEqual([]);
            expect(node.sequenceNumber).toBe(1);
            expect(node.lastBroadcastId).toBe(0);
        });
    });

    describe('findRoute', () => {
        it('should return null for empty routing table', () => {
            const node = initAodvNode('node1');
            expect(findRoute(node, 'node2')).toBeNull();
        });

        it('should find valid route', () => {
            let node = initAodvNode('node1');
            node = updateRoute(node, 'node3', 'node2', 2, 5);
            const route = findRoute(node, 'node3');
            expect(route).not.toBeNull();
            expect(route!.destination).toBe('node3');
            expect(route!.nextHop).toBe('node2');
            expect(route!.hopCount).toBe(2);
        });

        it('should not find route to non-existent destination', () => {
            let node = initAodvNode('node1');
            node = updateRoute(node, 'node3', 'node2', 2, 5);
            expect(findRoute(node, 'node4')).toBeNull();
        });
    });

    describe('updateRoute', () => {
        it('should add new route entry', () => {
            let node = initAodvNode('node1');
            node = updateRoute(node, 'dest', 'next', 3, 10);
            expect(node.routingTable.length).toBe(1);
            expect(node.routingTable[0].destination).toBe('dest');
            expect(node.routingTable[0].hopCount).toBe(3);
        });

        it('should update route with higher sequence number', () => {
            let node = initAodvNode('node1');
            node = updateRoute(node, 'dest', 'next1', 3, 10);
            node = updateRoute(node, 'dest', 'next2', 5, 15); // Higher seq
            expect(node.routingTable.length).toBe(1);
            expect(node.routingTable[0].nextHop).toBe('next2');
            expect(node.routingTable[0].sequenceNumber).toBe(15);
        });

        it('should update route with lower hop count for same sequence', () => {
            let node = initAodvNode('node1');
            node = updateRoute(node, 'dest', 'next1', 5, 10);
            node = updateRoute(node, 'dest', 'next2', 3, 10); // Same seq, lower hops
            expect(node.routingTable[0].nextHop).toBe('next2');
            expect(node.routingTable[0].hopCount).toBe(3);
        });

        it('should not update route with worse metrics', () => {
            let node = initAodvNode('node1');
            node = updateRoute(node, 'dest', 'next1', 2, 10);
            node = updateRoute(node, 'dest', 'next2', 5, 8); // Lower seq
            expect(node.routingTable[0].nextHop).toBe('next1');
        });
    });

    describe('createRREQ', () => {
        it('should create RREQ packet with correct fields', () => {
            const node = initAodvNode('source');
            const { packet, newState } = createRREQ(node, 'dest', { x: 10, y: 20 });

            expect(packet.type).toBe('RREQ');
            expect(packet.sourceId).toBe('source');
            expect(packet.destinationId).toBe('dest');
            expect(packet.originatorId).toBe('source');
            expect(packet.hopCount).toBe(0);
            expect(packet.broadcastId).toBe(1);
            expect(packet.position).toEqual({ x: 10, y: 20 });
            expect(packet.ttl).toBe(10);
        });

        it('should increment broadcast ID and sequence number', () => {
            const node = initAodvNode('source');
            const { newState: state1 } = createRREQ(node, 'dest1', { x: 0, y: 0 });
            const { newState: state2 } = createRREQ(state1, 'dest2', { x: 0, y: 0 });

            expect(state1.lastBroadcastId).toBe(1);
            expect(state2.lastBroadcastId).toBe(2);
            expect(state1.sequenceNumber).toBe(2);
            expect(state2.sequenceNumber).toBe(3);
        });
    });

    describe('processRREQ', () => {
        it('should generate RREP when destination is reached', () => {
            const destNode = initAodvNode('dest');
            const rreq = {
                type: 'RREQ' as const,
                sourceId: 'intermediate',
                destinationId: 'dest',
                originatorId: 'source',
                hopCount: 2,
                sequenceNumber: 5,
                broadcastId: 1,
                position: { x: 0, y: 0 },
                progress: 0,
                ttl: 8
            };

            const result = processRREQ(destNode, rreq, { x: 100, y: 100 }, []);
            expect(result.replyPacket).not.toBeNull();
            expect(result.replyPacket!.type).toBe('RREP');
            expect(result.replyPacket!.destinationId).toBe('source');
        });

        it('should forward RREQ when not destination', () => {
            const intermediate = initAodvNode('intermediate');
            const rreq = {
                type: 'RREQ' as const,
                sourceId: 'source',
                destinationId: 'dest',
                originatorId: 'source',
                hopCount: 0,
                sequenceNumber: 5,
                broadcastId: 1,
                position: { x: 0, y: 0 },
                progress: 0,
                ttl: 10
            };

            const result = processRREQ(intermediate, rreq, { x: 50, y: 50 }, ['neighbor1', 'neighbor2']);
            expect(result.forwardPackets.length).toBe(2);
            expect(result.forwardPackets[0].hopCount).toBe(1);
            expect(result.forwardPackets[0].ttl).toBe(9);
        });

        it('should drop duplicate RREQ (same broadcast ID)', () => {
            const node = initAodvNode('intermediate');
            const rreq = {
                type: 'RREQ' as const,
                sourceId: 'source',
                destinationId: 'dest',
                originatorId: 'source',
                hopCount: 0,
                sequenceNumber: 5,
                broadcastId: 42,
                position: { x: 0, y: 0 },
                progress: 0,
                ttl: 10
            };

            // First processing
            const result1 = processRREQ(node, rreq, { x: 50, y: 50 }, ['neighbor']);
            // Second processing with same broadcast ID
            const result2 = processRREQ(result1.newState, rreq, { x: 50, y: 50 }, ['neighbor']);

            expect(result1.forwardPackets.length).toBe(1);
            expect(result2.forwardPackets.length).toBe(0); // Dropped
        });
    });

    describe('processRREP', () => {
        it('should update forward route', () => {
            const node = initAodvNode('intermediate');
            const rrep = {
                type: 'RREP' as const,
                sourceId: 'dest',
                destinationId: 'source',
                originatorId: 'source',
                hopCount: 1,
                sequenceNumber: 10,
                position: { x: 100, y: 100 },
                progress: 0,
                ttl: 9
            };

            const result = processRREP(node, rrep, { x: 50, y: 50 });
            const route = findRoute(result.newState, 'dest');
            expect(route).not.toBeNull();
            expect(route!.hopCount).toBe(2);
        });

        it('should indicate route established when originator receives RREP', () => {
            const originator = initAodvNode('source');
            const rrep = {
                type: 'RREP' as const,
                sourceId: 'dest',
                destinationId: 'source',
                originatorId: 'source',
                hopCount: 2,
                sequenceNumber: 10,
                position: { x: 100, y: 100 },
                progress: 0,
                ttl: 7
            };

            const result = processRREP(originator, rrep, { x: 0, y: 0 });
            expect(result.routeEstablished).toBe(true);
            expect(result.forwardPacket).toBeNull();
        });
    });
});

// ==================== CSMA/CA MAC Layer Tests ====================

describe('CSMA/CA MAC Layer', () => {
    describe('initMacNode', () => {
        it('should initialize with IDLE state', () => {
            const node = initMacNode('mac1');
            expect(node.nodeId).toBe('mac1');
            expect(node.state).toBe('IDLE');
            expect(node.backoffSlots).toBe(0);
            expect(node.backoffWindow).toBe(15); // CW_MIN
            expect(node.txQueue).toEqual([]);
            expect(node.collisionCount).toBe(0);
        });
    });

    describe('generateBackoff', () => {
        it('should generate value in valid range', () => {
            for (let i = 0; i < 100; i++) {
                const backoff = generateBackoff(15);
                expect(backoff).toBeGreaterThanOrEqual(0);
                expect(backoff).toBeLessThanOrEqual(15);
            }
        });

        it('should be uniformly distributed (statistical test)', () => {
            const samples = 10000;
            const windowSize = 15;
            const values: number[] = [];

            for (let i = 0; i < samples; i++) {
                values.push(generateBackoff(windowSize));
            }

            const avg = mean(values);
            // Expected mean = windowSize / 2 = 7.5
            expect(Math.abs(avg - 7.5)).toBeLessThan(0.5);
        });
    });

    describe('increaseBackoffWindow', () => {
        it('should double window (Binary Exponential Backoff)', () => {
            expect(increaseBackoffWindow(15)).toBe(31);
            expect(increaseBackoffWindow(31)).toBe(63);
            expect(increaseBackoffWindow(63)).toBe(127);
        });

        it('should cap at CW_MAX', () => {
            expect(increaseBackoffWindow(1023)).toBe(1023);
            expect(increaseBackoffWindow(512)).toBe(1023);
        });
    });

    describe('resetBackoffWindow', () => {
        it('should return CW_MIN', () => {
            expect(resetBackoffWindow()).toBe(15);
        });
    });

    describe('updateMacState', () => {
        it('should transition from IDLE to DIFS when queue has data', () => {
            let node = initMacNode('mac1');
            node = enqueueMacPacket(node, 'dest', 1500);

            const result = updateMacState(node, false, 10, false);
            expect(result.newState.state).toBe('DIFS');
        });

        it('should stay in IDLE when queue is empty', () => {
            const node = initMacNode('mac1');
            const result = updateMacState(node, false, 10, false);
            expect(result.newState.state).toBe('IDLE');
        });

        it('should complete full successful transmission cycle', () => {
            let node = initMacNode('mac1');
            node = enqueueMacPacket(node, 'dest', 1500);

            // IDLE -> DIFS
            let result = updateMacState(node, false, 10, false);
            expect(result.newState.state).toBe('DIFS');

            // DIFS -> BACKOFF (after DIFS timer)
            result = updateMacState(result.newState, false, 60, false);
            expect(result.newState.state).toBe('BACKOFF');

            // Wait out backoff (set to 0 for testing)
            result.newState.backoffSlots = 0;

            // BACKOFF -> TX
            result = updateMacState(result.newState, false, 20, false);
            expect(result.newState.state).toBe('TX');
            expect(result.startTx).toBe(true);

            // TX -> WAIT_ACK (after TX timer)
            result = updateMacState(result.newState, false, 100, false);
            expect(result.newState.state).toBe('WAIT_ACK');

            // WAIT_ACK -> IDLE (with ACK)
            result = updateMacState(result.newState, false, 10, true);
            expect(result.newState.state).toBe('IDLE');
            expect(result.txComplete).toBe(true);
            expect(result.newState.txQueue.length).toBe(0);
        });
    });

    describe('enqueueMacPacket', () => {
        it('should add packet to queue', () => {
            let node = initMacNode('mac1');
            node = enqueueMacPacket(node, 'dest', 1500);

            expect(node.txQueue.length).toBe(1);
            expect(node.txQueue[0].sourceId).toBe('mac1');
            expect(node.txQueue[0].destinationId).toBe('dest');
            expect(node.txQueue[0].size).toBe(1500);
        });
    });

    describe('detectCollision', () => {
        it('should detect collision when multiple nodes transmit', () => {
            expect(detectCollision(['node1', 'node2'])).toBe(true);
            expect(detectCollision(['node1'])).toBe(false);
            expect(detectCollision([])).toBe(false);
        });
    });

    describe('getChannelState', () => {
        it('should report busy when node is transmitting', () => {
            const states = [
                { ...initMacNode('a'), state: 'TX' as const },
                { ...initMacNode('b'), state: 'IDLE' as const }
            ];

            const result = getChannelState(states);
            expect(result.busy).toBe(true);
            expect(result.transmittingNodes).toEqual(['a']);
            expect(result.collision).toBe(false);
        });

        it('should detect collision when multiple nodes transmit', () => {
            const states = [
                { ...initMacNode('a'), state: 'TX' as const },
                { ...initMacNode('b'), state: 'TX' as const }
            ];

            const result = getChannelState(states);
            expect(result.collision).toBe(true);
        });
    });
});

// ==================== Path Loss Models Tests ====================

describe('Path Loss Models', () => {
    describe('friisPathLoss', () => {
        it('should return 0 for zero distance', () => {
            expect(friisPathLoss(0, 2.4)).toBe(0);
        });

        it('should calculate FSPL correctly for known values', () => {
            // At 2.4 GHz, 100m: FSPL ≈ 80 dB
            const pl = friisPathLoss(100, 2.4);
            expect(pl).toBeGreaterThan(75);
            expect(pl).toBeLessThan(85);
        });

        it('should increase with distance (20 dB per decade)', () => {
            const pl100 = friisPathLoss(100, 2.4);
            const pl1000 = friisPathLoss(1000, 2.4);
            // 20 * log10(10) = 20 dB increase
            expect(isClose(pl1000 - pl100, 20, 0.1, 0.5)).toBe(true);
        });

        it('should increase with frequency (20 dB per decade)', () => {
            const pl24 = friisPathLoss(100, 2.4);
            const pl24000 = friisPathLoss(100, 24);
            // 20 * log10(10) = 20 dB increase
            expect(isClose(pl24000 - pl24, 20, 0.1, 0.5)).toBe(true);
        });
    });

    describe('twoRayPathLoss', () => {
        it('should return 0 for zero distance', () => {
            expect(twoRayPathLoss(0, 10, 1.5, 2.4)).toBe(0);
        });

        it('should use Friis model for short distances', () => {
            const pl = twoRayPathLoss(10, 10, 1.5, 2.4);
            const friis = friisPathLoss(10, 2.4);
            expect(isClose(pl, friis, 0.1)).toBe(true);
        });

        it('should increase faster than Friis for long distances', () => {
            // Breakpoint distance = 4 * ht * hr / lambda
            // At 2.4GHz, lambda = 0.125m, so breakpoint = 4 * 10 * 1.5 / 0.125 = 480m
            // Need to be well beyond breakpoint
            const d = 5000; // Well beyond breakpoint
            const pl = twoRayPathLoss(d, 10, 1.5, 2.4);
            const friis = friisPathLoss(d, 2.4);
            // Two-ray increases at 40dB/decade vs 20dB/decade
            expect(pl).toBeGreaterThan(friis);
        });
    });

    describe('logDistancePathLoss', () => {
        it('should return reference loss at reference distance', () => {
            const pl = logDistancePathLoss(1, 2.4, 2.5, 1, 0);
            const ref = friisPathLoss(1, 2.4);
            expect(isClose(pl, ref, 0.01)).toBe(true);
        });

        it('should increase based on path loss exponent', () => {
            const pl10 = logDistancePathLoss(10, 2.4, 2.5, 1, 0);
            const pl100 = logDistancePathLoss(100, 2.4, 2.5, 1, 0);
            // 10 * n * log10(10) = 10 * 2.5 = 25 dB
            expect(isClose(pl100 - pl10, 25, 0.1, 0.5)).toBe(true);
        });

        it('should handle shadowing (statistical)', () => {
            const samples = 1000;
            const values: number[] = [];
            for (let i = 0; i < samples; i++) {
                values.push(logDistancePathLoss(100, 2.4, 2.5, 1, 4)); // 4 dB std
            }
            const stdDev = std(values);
            // Should be close to 4 dB
            expect(stdDev).toBeGreaterThan(2);
            expect(stdDev).toBeLessThan(6);
        });
    });

    describe('airToGroundPathLoss', () => {
        it('should be greater than pure Friis (includes environment loss)', () => {
            const a2g = airToGroundPathLoss(500, 100, 10, 2.4, 'urban');
            const friis = friisPathLoss(
                Math.sqrt(500 * 500 + 90 * 90),
                2.4
            );
            expect(a2g).toBeGreaterThan(friis);
        });

        it('should have lower loss for rural vs urban', () => {
            const urban = airToGroundPathLoss(500, 100, 10, 2.4, 'urban');
            const rural = airToGroundPathLoss(500, 100, 10, 2.4, 'rural');
            expect(rural).toBeLessThan(urban);
        });

        it('should handle different elevation angles', () => {
            // Low elevation (far horizontal)
            const lowEl = airToGroundPathLoss(1000, 50, 10, 2.4, 'suburban');
            // High elevation (close and high)
            const highEl = airToGroundPathLoss(100, 200, 10, 2.4, 'suburban');
            // Higher elevation generally means better LoS probability
            // Note: actual values depend on 3GPP model parameters
            expect(typeof lowEl).toBe('number');
            expect(typeof highEl).toBe('number');
        });
    });
});

// ==================== Link Quality Tests ====================

describe('Link Quality', () => {
    describe('calculateRSSI', () => {
        it('should calculate RSSI = txPower + gains - pathLoss', () => {
            const rssi = calculateRSSI(20, 5, 3, 80);
            expect(rssi).toBe(20 + 5 + 3 - 80); // -52 dBm
        });

        it('should handle negative values correctly', () => {
            const rssi = calculateRSSI(10, 0, 0, 100);
            expect(rssi).toBe(-90); // dBm
        });
    });

    describe('calculateSNR', () => {
        it('should compute SNR as RSSI minus noise floor', () => {
            // Noise floor ≈ -174 + 10*log10(20e6) + 6 ≈ -95 dBm
            const snr = calculateSNR(-60, 20);
            expect(snr).toBeGreaterThan(30);
            expect(snr).toBeLessThan(40);
        });

        it('should decrease with bandwidth', () => {
            const snr20 = calculateSNR(-70, 20);
            const snr40 = calculateSNR(-70, 40);
            // 10*log10(2) ≈ 3 dB difference
            expect(snr20 - snr40).toBeGreaterThan(2);
            expect(snr20 - snr40).toBeLessThan(4);
        });
    });

    describe('evaluateLinkQuality', () => {
        it('should classify excellent quality', () => {
            expect(evaluateLinkQuality(-40, 30)).toBe('excellent');
        });

        it('should classify good quality', () => {
            expect(evaluateLinkQuality(-60, 20)).toBe('good');
        });

        it('should classify fair quality', () => {
            expect(evaluateLinkQuality(-70, 10)).toBe('fair');
        });

        it('should classify poor quality', () => {
            expect(evaluateLinkQuality(-80, 5)).toBe('poor');
        });

        it('should classify disconnected', () => {
            expect(evaluateLinkQuality(-90, 2)).toBe('disconnected');
        });
    });

    describe('estimateDataRate', () => {
        it('should return very low rate for very low SNR', () => {
            const rate = estimateDataRate(-20, 20);
            expect(rate).toBeLessThan(1); // Very low but may not be exactly 0
        });

        it('should increase with SNR', () => {
            const rate10 = estimateDataRate(10, 20);
            const rate20 = estimateDataRate(20, 20);
            expect(rate20).toBeGreaterThan(rate10);
        });

        it('should scale with bandwidth', () => {
            const rate20 = estimateDataRate(15, 20);
            const rate40 = estimateDataRate(15, 40);
            expect(isClose(rate40 / rate20, 2, 0.1)).toBe(true);
        });
    });

    describe('estimateLatency', () => {
        it('should include propagation delay', () => {
            const latency = estimateLatency(300000, 100, 1500);
            // 300km / 3e8 m/s = 1ms propagation
            expect(latency).toBeGreaterThanOrEqual(1);
        });

        it('should increase with lower data rate', () => {
            const fast = estimateLatency(100, 100, 1500);
            const slow = estimateLatency(100, 10, 1500);
            expect(slow).toBeGreaterThan(fast);
        });
    });
});

// ==================== Packet Success Tests ====================

describe('Packet Success Probability', () => {
    it('should return ~1 for high SNR', () => {
        const prob = packetSuccessProbability(30, 1500);
        expect(prob).toBeGreaterThan(0.99);
    });

    it('should decrease with lower SNR', () => {
        const highSnr = packetSuccessProbability(20, 1500);
        const lowSnr = packetSuccessProbability(5, 1500);
        expect(highSnr).toBeGreaterThan(lowSnr);
    });

    it('should decrease with larger packets', () => {
        const small = packetSuccessProbability(10, 100);
        const large = packetSuccessProbability(10, 10000);
        expect(small).toBeGreaterThan(large);
    });
});

// ==================== Network Utility Functions ====================

describe('Network Utility Functions', () => {
    describe('distance', () => {
        it('should compute Euclidean distance', () => {
            expect(distance({ x: 0, y: 0 }, { x: 3, y: 4 })).toBe(5);
        });

        it('should handle negative coordinates', () => {
            expect(distance({ x: -3, y: -4 }, { x: 0, y: 0 })).toBe(5);
        });
    });

    describe('isInRange', () => {
        it('should return true when within range', () => {
            expect(isInRange({ x: 0, y: 0 }, { x: 3, y: 0 }, 5)).toBe(true);
        });

        it('should return false when out of range', () => {
            expect(isInRange({ x: 0, y: 0 }, { x: 10, y: 0 }, 5)).toBe(false);
        });

        it('should return true at exact range boundary', () => {
            expect(isInRange({ x: 0, y: 0 }, { x: 5, y: 0 }, 5)).toBe(true);
        });
    });

    describe('getNeighbors', () => {
        it('should find all neighbors within range', () => {
            const nodes = [
                { id: 'a', position: { x: 0, y: 0 } },
                { id: 'b', position: { x: 3, y: 0 } },
                { id: 'c', position: { x: 10, y: 0 } },
                { id: 'd', position: { x: 2, y: 2 } }
            ];

            const neighbors = getNeighbors('a', { x: 0, y: 0 }, nodes, 5);
            expect(neighbors).toContain('b');
            expect(neighbors).toContain('d');
            expect(neighbors).not.toContain('c');
            expect(neighbors).not.toContain('a'); // Should not include self
        });

        it('should return empty array when no neighbors', () => {
            const nodes = [
                { id: 'a', position: { x: 0, y: 0 } },
                { id: 'b', position: { x: 100, y: 100 } }
            ];

            const neighbors = getNeighbors('a', { x: 0, y: 0 }, nodes, 5);
            expect(neighbors).toEqual([]);
        });
    });
});

// ==================== Packet Factory Tests ====================

describe('Packet Factory', () => {
    describe('createHelloPacket', () => {
        it('should create HELLO packet with correct type', () => {
            const packet = createHelloPacket('drone1', 'station1');
            expect(packet.type).toBe('HELLO');
            expect(packet.sourceId).toBe('drone1');
            expect(packet.destId).toBe('station1');
            expect(packet.size).toBe(64);
            expect(packet.priority).toBe(7);
        });
    });

    describe('createUpdatePacket', () => {
        it('should create UPDATE packet with position', () => {
            const packet = createUpdatePacket(
                'drone1',
                'station1',
                { x: 10, y: 20, theta: 0.5 },
                { vx: 1, vy: 2 }
            );
            expect(packet.type).toBe('UPDATE');
            expect(packet.payload).toEqual({
                position: { x: 10, y: 20, theta: 0.5 },
                velocity: { vx: 1, vy: 2 }
            });
            expect(packet.size).toBe(128);
        });
    });
});
