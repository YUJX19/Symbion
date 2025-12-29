/**
 * @module tasks/u2u-mcs
 * @description U2U Sidelink Dynamic MCS Selection Task
 *
 * This task implements the 5G NR Sidelink MCS adaptive selection problem
 * aligned with ns3-uavlink simulations.
 *
 * ## Key Features
 * - Dynamic MCS selection based on SINR, distance, and recent BLER
 * - URLLC-oriented reward with throughput and reliability trade-off
 * - Multiple baseline policies for comparison
 * - Standardized logging schema for reproducibility
 *
 * ## Usage
 * ```typescript
 * import { runTask2 } from 'symbion/tasks';
 *
 * // Run with defaults
 * const result = await runTask2();
 *
 * // Run with custom config
 * const result = await runTask2({
 *   seed: 42,
 *   totalEpisodes: 10,
 *   enableJsonlFile: true
 * });
 * ```
 */

// Core exports
export * from './config';
export * from './scenario';
export * from './observation';
export * from './reward';
export * from './policies';
export * from './environment';

// Task 2 exports (AI-Driven Dynamic MCS)
export * from './schema';
export * from './protocol';
export * from './transport';
export * from './policy';
export * from './errors';
export { runTask2, type Task2Result } from './task2';
