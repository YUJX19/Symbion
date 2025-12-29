/**
 * @module src/isac
 * @description ISAC (Integrated Sensing and Communication) Module
 *
 * Domain layer for ISAC-specific functionality. This module provides high-level
 * abstractions for ISAC scenarios while leveraging models/planning for generic
 * trajectory optimization algorithms.
 * 
 * Architecture:
 * - isac/planner: ISAC-specific planner with LoS/URLLC awareness
 * - isac/metrics: ISAC performance metrics (LoS, throughput, URLLC)
 * - isac/constraints: Flight and communication constraints (ConstraintSpec factories)
 * - models/planning: Generic trajectory algorithms (MINCO, RRT, etc.) - used internally
 * 
 * Contents:
 * - planner.ts: ISAC trajectory planner
 * - metrics/: Performance metrics (LoS, throughput, URLLC)
 * - constraints/: Flight and communication constraints
 * - envAdapter.ts: Environment adapter for AI training
 * - report.ts: Report generation
 */

export * from './planner';
export * as metrics from './metrics';
export * as constraints from './constraints';
export * from './envAdapter';
export * from './report';
