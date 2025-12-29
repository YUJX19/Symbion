/**
 * @module tasks/isac-trajectory
 * @description ISAC LoS-aware URLLC Trajectory Optimization Task
 *
 * This task implements UAV trajectory optimization with:
 * - LoS (Line-of-Sight) persistence maintenance
 * - URLLC reliability constraints
 * - Sensing-communication trade-off
 *
 * ## Key Features
 * - Free-space + Rician channel model
 * - Discrete trajectory planning (waypoint selection)
 * - Multi-objective: throughput + LoS + energy + URLLC
 * - 3 baseline planners for comparison
 *
 * ## Usage
 * ```typescript
 * import { tasks } from 'symbion';
 *
 * const config = tasks.isacTrajectory.createConfig({
 *   seed: 42,
 *   numWaypoints: 20,
 *   urllcTarget: 1e-5,
 * });
 *
 * const env = tasks.isacTrajectory.createEnvironment(config);
 * const planner = tasks.isacTrajectory.createPlanner('proposed');
 * ```
 */

export * from './config';
export * from './scenario';
export * from './observation';
export * from './reward';
export * from './policies';
export * from './environment';
export * from './report';

// Task1: Direct Optimization (MINCO + L-BFGS)
export * from './types';
export * from './utils';
export * from './uav-comm-model';
export * from './objective';
export * from './constraints';
export * from './optimizer';
export { runTask1 } from './task1';
