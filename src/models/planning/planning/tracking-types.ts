/**
 * @module planning/tracking-types
 * @description Type definitions for aerial tracking and landing algorithms.
 * 
 * Provides types for target prediction, visibility-constrained path planning,
 * and tracking trajectory optimization.
 */

import type { Vector3Array } from '../trajectory/types';

// ==================== Occupancy Grid ====================

/**
 * 3D Occupancy Grid Map for collision detection and path planning
 */
export interface OccupancyGrid {
  /** Grid resolution in meters */
  resolution: number;
  /** Grid dimensions [x, y, z] in cells */
  dimensions: [number, number, number];
  /** Origin position in world coordinates */
  origin: Vector3Array;
  /** Occupancy data: 1 = occupied, 0 = free, -1 = unknown */
  data: Int8Array | number[];
}

/**
 * Create an empty occupancy grid
 */
export function createOccupancyGrid(
  dimensions: [number, number, number],
  resolution: number,
  origin: Vector3Array = [0, 0, 0]
): OccupancyGrid {
  const totalCells = dimensions[0] * dimensions[1] * dimensions[2];
  return {
    resolution,
    dimensions,
    origin,
    data: new Int8Array(totalCells).fill(0)
  };
}

/**
 * Convert world position to grid index
 */
export function positionToIndex(
  position: Vector3Array,
  grid: OccupancyGrid
): [number, number, number] {
  return [
    Math.floor((position[0] - grid.origin[0]) / grid.resolution),
    Math.floor((position[1] - grid.origin[1]) / grid.resolution),
    Math.floor((position[2] - grid.origin[2]) / grid.resolution)
  ];
}

/**
 * Convert grid index to world position (cell center)
 */
export function indexToPosition(
  index: [number, number, number],
  grid: OccupancyGrid
): Vector3Array {
  return [
    grid.origin[0] + (index[0] + 0.5) * grid.resolution,
    grid.origin[1] + (index[1] + 0.5) * grid.resolution,
    grid.origin[2] + (index[2] + 0.5) * grid.resolution
  ];
}

/**
 * Check if a grid index is valid
 */
export function isValidIndex(
  index: [number, number, number],
  grid: OccupancyGrid
): boolean {
  return (
    index[0] >= 0 && index[0] < grid.dimensions[0] &&
    index[1] >= 0 && index[1] < grid.dimensions[1] &&
    index[2] >= 0 && index[2] < grid.dimensions[2]
  );
}

/**
 * Get occupancy value at grid index
 */
export function getOccupancy(
  index: [number, number, number],
  grid: OccupancyGrid
): number {
  if (!isValidIndex(index, grid)) return 1; // Out of bounds = occupied
  const flatIndex = index[0] +
    index[1] * grid.dimensions[0] +
    index[2] * grid.dimensions[0] * grid.dimensions[1];
  return grid.data[flatIndex];
}

/**
 * Set occupancy value at grid index
 */
export function setOccupancy(
  index: [number, number, number],
  value: number,
  grid: OccupancyGrid
): void {
  if (!isValidIndex(index, grid)) return;
  const flatIndex = index[0] +
    index[1] * grid.dimensions[0] +
    index[2] * grid.dimensions[0] * grid.dimensions[1];
  grid.data[flatIndex] = value;
}

/**
 * Check if a world position is occupied
 */
export function isOccupied(
  position: Vector3Array,
  grid: OccupancyGrid
): boolean {
  const index = positionToIndex(position, grid);
  return getOccupancy(index, grid) > 0;
}

// ==================== Target State ====================

/**
 * Target state for tracking
 */
export interface TargetState {
  /** Current position [x, y, z] in meters */
  position: Vector3Array;
  /** Current velocity [vx, vy, vz] in m/s */
  velocity: Vector3Array;
  /** Optional: orientation quaternion [w, x, y, z] */
  orientation?: [number, number, number, number];
  /** Timestamp in seconds */
  timestamp?: number;
}

// ==================== Target Prediction ====================

/**
 * Options for target motion prediction
 */
export interface TargetPredictionOptions {
  /** Time step for prediction in seconds (default: 0.2) */
  timeStep?: number;
  /** Maximum acceleration magnitude in m/s² (default: 3.0) */
  maxAcceleration?: number;
  /** Maximum velocity magnitude in m/s (default: 5.0) */
  maxVelocity?: number;
  /** Penalty weight for acceleration changes (default: 1.0) */
  accelerationPenalty?: number;
  /** Maximum computation time in seconds (default: 0.1) */
  maxComputationTime?: number;
  /** Height constraint for ground vehicles (optional) */
  groundHeight?: number;
}

/**
 * Result of target motion prediction
 */
export interface TargetPredictionResult {
  /** Predicted positions over time */
  positions: Vector3Array[];
  /** Predicted velocities over time */
  velocities: Vector3Array[];
  /** Time stamps for each prediction point */
  times: number[];
  /** Whether prediction was successful */
  success: boolean;
  /** Error message if failed */
  errorMessage?: string;
}

// ==================== Visibility Path Planning ====================

/**
 * Options for visibility-constrained path planning
 */
export interface VisibilityPathOptions {
  /** Tolerance for distance to target (default: 0.5 m) */
  toleranceDistance?: number;
  /** Maximum search time in seconds (default: 0.2) */
  maxSearchTime?: number;
  /** Angular clearance for visibility checks in radians (default: 0.1) */
  angularClearance?: number;
}

/**
 * Result of visibility-constrained path planning
 */
export interface VisibilityPathResult {
  /** Complete path from start to observation points */
  path: Vector3Array[];
  /** Key waypoints at each target observation position */
  waypoints: Vector3Array[];
  /** Whether planning was successful */
  success: boolean;
  /** Error message if failed */
  errorMessage?: string;
}

// ==================== Visibility Region ====================

/**
 * Fan-shaped visibility region around a target
 */
export interface VisibilityRegion {
  /** Target position (center of the visibility circle) */
  center: Vector3Array;
  /** Point in the center of the visible fan sector */
  visiblePoint: Vector3Array;
  /** Half-angle of the visible sector in radians */
  halfAngle: number;
  /** Distance from center to visiblePoint */
  distance: number;
}

/**
 * Options for visibility region computation
 */
export interface VisibilityRegionOptions {
  /** Angular resolution for boundary search in radians (default: 0.02) */
  angularResolution?: number;
  /** Minimum clearance angle from boundaries in radians (default: 0.1) */
  minClearanceAngle?: number;
}

// ==================== Tracking Planner ====================

/**
 * Configuration for the tracking planner
 */
export interface TrackingPlannerConfig {
  /** Desired tracking distance in meters */
  trackingDistance: number;
  /** Distance tolerance in meters */
  distanceTolerance: number;
  /** Prediction duration in seconds */
  predictionDuration: number;
  /** Prediction time step in seconds */
  predictionTimeStep: number;
  /** Weight for tracking cost in trajectory optimization */
  trackingWeight: number;
  /** Weight for visibility cost in trajectory optimization */
  visibilityWeight: number;
  /** Maximum drone velocity in m/s */
  maxVelocity: number;
  /** Maximum drone acceleration in m/s² */
  maxAcceleration: number;
  /** Planning frequency in Hz */
  planningFrequency: number;
  /** Safe flight corridor half-width in meters */
  corridorHalfWidth: number;
  /** Safe flight corridor half-height in meters */
  corridorHalfHeight: number;
}

/**
 * Default tracking planner configuration
 */
export const DEFAULT_TRACKING_CONFIG: TrackingPlannerConfig = {
  trackingDistance: 5.0,
  distanceTolerance: 0.5,
  predictionDuration: 3.0,
  predictionTimeStep: 0.2,
  trackingWeight: 1000.0,
  visibilityWeight: 10000.0,
  maxVelocity: 3.0,
  maxAcceleration: 2.0,
  planningFrequency: 10.0,
  corridorHalfWidth: 2.0,
  corridorHalfHeight: 2.0,
};

// ==================== Landing Configuration ====================

/**
 * Configuration for aerial landing
 */
export interface LandingConfig {
  /** Offset from vehicle center to landing point in vehicle frame */
  landingOffset: Vector3Array;
  /** Approach speed in m/s */
  approachSpeed: number;
  /** Final descent speed in m/s */
  descentSpeed: number;
  /** Height threshold to trigger final descent */
  descentThreshold: number;
  /** Velocity threshold for successful landing */
  landingVelocityThreshold: number;
}

/**
 * Default landing configuration
 */
export const DEFAULT_LANDING_CONFIG: LandingConfig = {
  landingOffset: [0, 0, 0.3],  // 30cm above vehicle center
  approachSpeed: 1.0,
  descentSpeed: 0.3,
  descentThreshold: 0.5,
  landingVelocityThreshold: 0.2
};

// ==================== Cost Functions ====================

/**
 * Result of cost computation (value and gradient)
 */
export interface CostGradient {
  /** Cost value */
  cost: number;
  /** Gradient with respect to position */
  gradient: Vector3Array;
}

// ==================== Utility Types ====================

/**
 * A* search node for path planning
 */
export interface SearchNode {
  /** Grid index */
  index: [number, number, number];
  /** Cost from start */
  g: number;
  /** Heuristic cost to goal */
  h: number;
  /** Parent node index (for path reconstruction) */
  parentKey?: string;
  /** Node state: 0=unvisited, 1=open, 2=closed */
  state: number;
}

// PriorityQueueEntry moved to utils.ts - import from there

/**
 * Convert index to string key for hash map
 */
export function indexToKey(index: [number, number, number]): string {
  return `${index[0]},${index[1]},${index[2]}`;
}

/**
 * Convert string key back to index
 */
export function keyToIndex(key: string): [number, number, number] {
  const parts = key.split(',').map(Number);
  return [parts[0], parts[1], parts[2]];
}
