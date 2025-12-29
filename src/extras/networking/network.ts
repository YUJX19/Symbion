/**
 * @module communication/network
 * @description Network Layer and MAC Layer Simulation Algorithms
 * 
 * Inspired by ns-3 network simulation:
 * - AODV (Ad-hoc On-demand Distance Vector) Routing Protocol
 * - CSMA/CA (Carrier Sense Multiple Access with Collision Avoidance) MAC Layer
 */

// ==================== Type Definitions ====================

/**
 * AODV Routing Table Entry
 */
export interface AodvRouteEntry {
    /** Destination node ID */
    destination: string;
    /** Next hop node ID along the path */
    nextHop: string;
    /** Number of hops to destination */
    hopCount: number;
    /** Destination sequence number for freshness */
    sequenceNumber: number;
    /** Route lifetime in milliseconds */
    lifetime: number;
    /** Whether the route is currently active */
    isValid: boolean;
}

/**
 * AODV Control Packet Types
 */
export type AodvPacketType = 'RREQ' | 'RREP' | 'RERR' | 'DATA';

/**
 * AODV Routing Packet
 */
export interface AodvPacket {
    type: AodvPacketType;
    sourceId: string;
    destinationId: string;
    /** Original node that initiated the RREQ */
    originatorId: string;
    hopCount: number;
    sequenceNumber: number;
    /** Unique ID for RREQ flood control */
    broadcastId?: number;
    /** Current position for visualization */
    position: { x: number; y: number };
    /** Final target position */
    targetPosition?: { x: number; y: number };
    /** Transmission progress [0-1] */
    progress: number;
    /** Hop count limit */
    ttl: number;
}

/**
 * Internal state maintained by an AODV-enabled node
 */
export interface AodvNodeState {
    nodeId: string;
    routingTable: AodvRouteEntry[];
    /** Cache of processed broadcast IDs to prevent loops */
    broadcastIdCache: number[];
    sequenceNumber: number;
    lastBroadcastId: number;
}

/**
 * 802.11 style MAC Layer States
 */
export type MacState = 'IDLE' | 'DIFS' | 'BACKOFF' | 'TX' | 'RX' | 'WAIT_ACK' | 'COLLISION';

/**
 * MAC Layer State for a network node
 */
export interface MacNodeState {
    nodeId: string;
    state: MacState;
    /** Remaining backoff slots before transmission */
    backoffSlots: number;
    /** Current size of the contention window */
    backoffWindow: number;
    /** Buffer for pending transmissions */
    txQueue: MacPacket[];
    /** Number of sequential collisions for current packet */
    collisionCount: number;
    /** Timer for DIFS/SIFS waiting */
    difsTimer: number;
    /** Active transmission timer */
    txTimer: number;
}

/**
 * MAC Layer Data Packet
 */
export interface MacPacket {
    id: string;
    sourceId: string;
    destinationId: string;
    /** Size in bytes */
    size: number;
    isAck: boolean;
    isRts?: boolean;
    isCts?: boolean;
    timestamp: number;
}

/**
 * Channel monitoring event for visualization timelines
 */
export interface ChannelEvent {
    timestamp: number;
    type: 'idle' | 'busy' | 'collision' | 'success';
    nodeId?: string;
    duration: number;
}

// ==================== AODV Routing Protocol ====================

/**
 * Initializes a new AODV node state
 */
export function initAodvNode(nodeId: string): AodvNodeState {
    return {
        nodeId,
        routingTable: [],
        broadcastIdCache: [],
        sequenceNumber: 1,
        lastBroadcastId: 0
    };
}

/**
 * Looks up a valid route to the destination in the routing table
 */
export function findRoute(
    state: AodvNodeState,
    destinationId: string
): AodvRouteEntry | null {
    const route = state.routingTable.find(
        r => r.destination === destinationId && r.isValid
    );
    return route || null;
}

/**
 * Adds or updates a route entry if a better/fresher path is found
 */
export function updateRoute(
    state: AodvNodeState,
    destination: string,
    nextHop: string,
    hopCount: number,
    sequenceNumber: number
): AodvNodeState {
    const existingIndex = state.routingTable.findIndex(
        r => r.destination === destination
    );

    const newEntry: AodvRouteEntry = {
        destination,
        nextHop,
        hopCount,
        sequenceNumber,
        lifetime: 10000,  // 10 second route validity
        isValid: true
    };

    const newTable = [...state.routingTable];
    if (existingIndex >= 0) {
        // Update only if the new sequence number is higher (fresher) 
        // OR the hop count is lower for the same sequence number
        const existing = newTable[existingIndex];
        if (sequenceNumber > existing.sequenceNumber ||
            (sequenceNumber === existing.sequenceNumber && hopCount < existing.hopCount)) {
            newTable[existingIndex] = newEntry;
        }
    } else {
        newTable.push(newEntry);
    }

    return { ...state, routingTable: newTable };
}

/**
 * Creates a Route Request (RREQ) packet to discover a path
 */
export function createRREQ(
    state: AodvNodeState,
    destinationId: string,
    position: { x: number; y: number }
): { packet: AodvPacket; newState: AodvNodeState } {
    const newBroadcastId = state.lastBroadcastId + 1;
    const newSeqNum = state.sequenceNumber + 1;

    const packet: AodvPacket = {
        type: 'RREQ',
        sourceId: state.nodeId,
        destinationId,
        originatorId: state.nodeId,
        hopCount: 0,
        sequenceNumber: newSeqNum,
        broadcastId: newBroadcastId,
        position,
        progress: 0,
        ttl: 10
    };

    return {
        packet,
        newState: {
            ...state,
            lastBroadcastId: newBroadcastId,
            sequenceNumber: newSeqNum,
            broadcastIdCache: [...state.broadcastIdCache, newBroadcastId]
        }
    };
}

/**
 * Processes an incoming RREQ packet
 */
export function processRREQ(
    state: AodvNodeState,
    packet: AodvPacket,
    myPosition: { x: number; y: number },
    neighbors: string[]
): {
    newState: AodvNodeState;
    forwardPackets: AodvPacket[];
    replyPacket: AodvPacket | null;
} {
    // Prevent broadcast storms by checking if RREQ was already handled
    if (packet.broadcastId && state.broadcastIdCache.includes(packet.broadcastId)) {
        return { newState: state, forwardPackets: [], replyPacket: null };
    }

    // Add to cache
    let newState = {
        ...state,
        broadcastIdCache: packet.broadcastId
            ? [...state.broadcastIdCache.slice(-50), packet.broadcastId]  // Keep last 50 IDs
            : state.broadcastIdCache
    };

    // Update reverse route back to the RREQ originator
    newState = updateRoute(
        newState,
        packet.originatorId,
        packet.sourceId,
        packet.hopCount + 1,
        packet.sequenceNumber
    );

    // If I am the target, generate a Route Reply (RREP)
    if (state.nodeId === packet.destinationId) {
        const rrep: AodvPacket = {
            type: 'RREP',
            sourceId: state.nodeId,
            destinationId: packet.originatorId,
            originatorId: packet.originatorId,
            hopCount: 0,
            sequenceNumber: state.sequenceNumber + 1,
            position: myPosition,
            targetPosition: packet.position,
            progress: 0,
            ttl: 10
        };
        return {
            newState: { ...newState, sequenceNumber: newState.sequenceNumber + 1 },
            forwardPackets: [],
            replyPacket: rrep
        };
    }

    // Otherwise, forward RREQ if TTL allows
    if (packet.ttl > 1) {
        const forwardPackets: AodvPacket[] = neighbors.map(neighborId => ({
            ...packet,
            sourceId: state.nodeId,
            hopCount: packet.hopCount + 1,
            position: myPosition,
            progress: 0,
            ttl: packet.ttl - 1
        }));

        return { newState, forwardPackets, replyPacket: null };
    }

    return { newState, forwardPackets: [], replyPacket: null };
}

/**
 * Processes an incoming RREP packet
 */
export function processRREP(
    state: AodvNodeState,
    packet: AodvPacket,
    myPosition: { x: number; y: number }
): {
    newState: AodvNodeState;
    forwardPacket: AodvPacket | null;
    routeEstablished: boolean;
} {
    // Add forward route to the requested destination
    const newState = updateRoute(
        state,
        packet.sourceId,
        packet.sourceId,
        packet.hopCount + 1,
        packet.sequenceNumber
    );

    // If I am the originator, the path is now ready for use
    if (state.nodeId === packet.destinationId) {
        return { newState, forwardPacket: null, routeEstablished: true };
    }

    // Otherwise, forward the RREP towards the originator
    const route = findRoute(newState, packet.destinationId);
    if (route && packet.ttl > 1) {
        const forwardPacket: AodvPacket = {
            ...packet,
            sourceId: state.nodeId,
            hopCount: packet.hopCount + 1,
            position: myPosition,
            progress: 0,
            ttl: packet.ttl - 1
        };
        return { newState, forwardPacket, routeEstablished: false };
    }

    return { newState, forwardPacket: null, routeEstablished: false };
}

// ==================== CSMA/CA MAC Layer ====================

const SLOT_TIME = 20;        // ms
const SIFS = 10;             // Short Inter-frame Space (ms)
const DIFS = SIFS + 2 * SLOT_TIME; // Distributed Inter-frame Space (ms)
const CW_MIN = 15;           // Minimum Contention Window
const CW_MAX = 1023;         // Maximum Contention Window
const TX_TIME = 100;         // Packet transmission time (ms)
const ACK_TIMEOUT = 50;      // Propagation delay limit for ACK (ms)

/**
 * Initializes a MAC layer node state
 */
export function initMacNode(nodeId: string): MacNodeState {
    return {
        nodeId,
        state: 'IDLE',
        backoffSlots: 0,
        backoffWindow: CW_MIN,
        txQueue: [],
        collisionCount: 0,
        difsTimer: 0,
        txTimer: 0
    };
}

/**
 * Generates a random backoff count from the current window
 */
export function generateBackoff(windowSize: number): number {
    return Math.floor(Math.random() * (windowSize + 1));
}

/**
 * Doubles the contention window size (Binary Exponential Backoff)
 */
export function increaseBackoffWindow(currentWindow: number): number {
    return Math.min(CW_MAX, (currentWindow + 1) * 2 - 1);
}

/**
 * Resets the contention window to CW_MIN after successful transmission
 */
export function resetBackoffWindow(): number {
    return CW_MIN;
}

/**
 * Updates the MAC layer state following IEEE 802.11 CSMA/CA logic
 */
export function updateMacState(
    state: MacNodeState,
    channelBusy: boolean,
    deltaTime: number,
    receivedAck: boolean
): {
    newState: MacNodeState;
    startTx: boolean;
    txComplete: boolean;
    collision: boolean;
} {
    const newState = { ...state };
    let startTx = false;
    let txComplete = false;
    let collision = false;

    switch (state.state) {
        case 'IDLE':
            if (state.txQueue.length > 0) {
                // Pending data found, enter DIFS sensing phase
                newState.state = 'DIFS';
                newState.difsTimer = DIFS;
            }
            break;

        case 'DIFS':
            if (channelBusy) {
                // Channel became busy, restart DIFS timer
                newState.difsTimer = DIFS;
            } else {
                newState.difsTimer -= deltaTime;
                if (newState.difsTimer <= 0) {
                    // DIFS passed, enter random backoff phase
                    newState.state = 'BACKOFF';
                    if (newState.backoffSlots === 0) {
                        newState.backoffSlots = generateBackoff(newState.backoffWindow);
                    }
                }
            }
            break;

        case 'BACKOFF':
            if (!channelBusy) {
                // Decrement backoff counter only when channel is idle
                const slotsToDecrement = Math.floor(deltaTime / SLOT_TIME);
                newState.backoffSlots = Math.max(0, newState.backoffSlots - slotsToDecrement);

                if (newState.backoffSlots === 0) {
                    // Backoff reached zero, start active transmission
                    newState.state = 'TX';
                    newState.txTimer = TX_TIME;
                    startTx = true;
                }
            }
            break;

        case 'TX':
            newState.txTimer -= deltaTime;
            if (newState.txTimer <= 0) {
                // Transmission finished, now wait for ACK within timeout
                newState.state = 'WAIT_ACK';
                newState.txTimer = ACK_TIMEOUT;
            }
            break;

        case 'WAIT_ACK':
            if (receivedAck) {
                // SUCCESS: Acknowledgement received
                newState.state = 'IDLE';
                newState.txQueue = newState.txQueue.slice(1);
                newState.backoffWindow = resetBackoffWindow();
                newState.backoffSlots = 0;
                txComplete = true;
            } else {
                newState.txTimer -= deltaTime;
                if (newState.txTimer <= 0) {
                    // FAILURE: ACK timeout reached, assume collision
                    newState.state = 'COLLISION';
                    collision = true;
                    newState.collisionCount++;
                    newState.backoffWindow = increaseBackoffWindow(newState.backoffWindow);
                    newState.backoffSlots = generateBackoff(newState.backoffWindow);
                }
            }
            break;

        case 'COLLISION':
            // Briefly show collision state before attempting retransmission with backoff
            newState.state = 'DIFS';
            newState.difsTimer = DIFS;
            break;
    }

    return { newState, startTx, txComplete, collision };
}

/**
 * Enqueues a new data packet at the MAC layer
 */
export function enqueueMacPacket(
    state: MacNodeState,
    destinationId: string,
    size: number = 1500
): MacNodeState {
    const packet: MacPacket = {
        id: `pkt-${Date.now()}-${Math.random().toString(36).slice(2, 8)}`,
        sourceId: state.nodeId,
        destinationId,
        size,
        isAck: false,
        timestamp: Date.now()
    };

    return {
        ...state,
        txQueue: [...state.txQueue, packet]
    };
}

/**
 * Detects if a collision occurred in the shared medium
 */
export function detectCollision(transmittingNodes: string[]): boolean {
    return transmittingNodes.length > 1;
}

/**
 * Retrieves aggregate channel state for visualization
 */
export function getChannelState(
    macStates: MacNodeState[]
): { busy: boolean; transmittingNodes: string[]; collision: boolean } {
    const transmittingNodes = macStates
        .filter(s => s.state === 'TX')
        .map(s => s.nodeId);

    return {
        busy: transmittingNodes.length > 0,
        transmittingNodes,
        collision: transmittingNodes.length > 1
    };
}

// ==================== Utility Functions ====================

/**
 * Computes Euclidean distance between two points
 */
export function distance(
    p1: { x: number; y: number },
    p2: { x: number; y: number }
): number {
    return Math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2);
}

/**
 * Checks if a destination node is within transmission range
 */
export function isInRange(
    p1: { x: number; y: number },
    p2: { x: number; y: number },
    commRange: number
): boolean {
    return distance(p1, p2) <= commRange;
}

/**
 * Gets a list of neighbor IDs within the specified communication range
 */
export function getNeighbors(
    nodeId: string,
    nodePosition: { x: number; y: number },
    allNodes: { id: string; position: { x: number; y: number } }[],
    commRange: number
): string[] {
    return allNodes
        .filter(n => n.id !== nodeId && isInRange(nodePosition, n.position, commRange))
        .map(n => n.id);
}

