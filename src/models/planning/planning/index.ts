/**
 * @module planning
 * @description Spatial Planning and Navigation Module
 * 
 * Provides algorithms for path planning and trajectory generation:
 * - Navigation: Path planning and reactive behaviors (RVO, ORCA, Dash, etc.)
 * - Collision Detection: Object avoidance and spatial queries
 * - Optimal Trajectory: A* path planning with simple waypoint following
 * - MINCO: Minimum control trajectory optimization (jerk/snap minimization)
 * - Corridor: Safe Flight Corridor generation (FIRI algorithm)
 * - Tracking: Target prediction and visibility-constrained path planning
 * - Aerial Tracking: Complete UAV tracking with corridor optimization
 * - Root Finding: Polynomial root finding for trajectory constraints
 * - Geometric Utilities: Half-space polytope operations
 * - QuickHull: 3D convex hull computation
 */

export * from './navigation';
export * from './collision';
export * from './optimal-trajectory';
export * from './types';
export * from './utils';
export * from './minco';
export * from './minco-time-optimizer';
export * from './minco-corridor-optimizer';
export * from './corridor';

export * from './tracking-types';
export * from './target-prediction';
export * from './visibility-path';
export * from './visibility-region';
export * from './minco-constraints';
export * from './tracking-planner';
export * from './los-constraints';
export * from './aerial-tracking-config';

// New modules from Fast-Racing
export * from './root-finder';
export * from './geoutils';
export * from './quickhull';

// GCOPTER modules
export * from './firi';
