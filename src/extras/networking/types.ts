/**
 * @module communication/types
 * @description Type definitions for the Communication module
 */

/**
 * 2D Vector for position representation
 */
export interface Vector2 {
    x: number;
    y: number;
}

/**
 * Pose with position and orientation
 */
export interface Pose {
    x: number;
    y: number;
    theta: number;
}

/**
 * Communication Message
 */
export interface Message {
    id: string;
    from: string;
    to: string;
    data: unknown;
    timestamp: number;
    /** Time to live (hop count limit) */
    ttl?: number;
}

/**
 * Network Node representation in a communication graph
 */
export interface NetworkNode {
    id: string;
    position: { x: number; y: number };
    transmitRange: number;
    neighbors: string[];
}

/**
 * Routing Table Entry
 */
export interface RouteEntry {
    destination: string;
    nextHop: string;
    hopCount: number;
    metric: number;
}

/**
 * Supported Media Access Control (MAC) protocols
 */
export type CommunicationProtocol = 'TDMA' | 'CSMA' | 'ALOHA' | 'CUSTOM';

/**
 * Network Topology representation
 */
export interface NetworkTopology {
    nodes: NetworkNode[];
    /** List of active links between nodes */
    edges: [string, string][];
    /** Adjacency matrix for graph analysis */
    adjacencyMatrix: number[][];
}

