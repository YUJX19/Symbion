/**
 * @packageDocumentation
 * @module symbion
 *
 * Symbion: Integrated Sensing and Communication (ISAC) Research Library
 *
 * A comprehensive open-source TypeScript library for ISAC research,
 * bridging wireless communication theory and robotics dynamics.
 *
 * ## Core Modules (Default Export)
 *
 * ### ISAC Framework
 * - `isac` - ISAC-specific algorithms (planner, metrics, constraints, report)
 * - `core` - Reproducible experiment framework (space, objective, constraint, runner, logging)
 *
 * ### Research Tasks
 * - `tasks` - Paper reproduction tasks (u2u-mcs, isac-trajectory)
 *
 * ### Physical Models
 * - `channel` - Channel modeling (AWGN, Rayleigh, Rician, GSCM)
 * - `beamforming` - Array processing and MIMO
 * - `sensing` - Radar/LiDAR sensing
 * - `robotics` - Dynamics + Control (merged)
 * - `planning` - Path + Trajectory planning (merged)
 * - `phy` - Physical layer (modulation + coding + spreading)
 * - `numeric` - Numerical methods (math + optimization)
 *
 * ## Usage Example
 * ```typescript
 * import { isac, channel, planning, robotics, phy, numeric } from 'symbion';
 *
 * // Create ISAC planner
 * const planner = isac.createPlanner(config);
 *
 * // Calculate channel capacity
 * const capacity = channel.awgn.shannonCapacity(10, 20e6);
 *
 * // Use dynamics
 * const state = robotics.dynamics.quadrotorDynamics(...)
 *
 * // Use modulation
 * const constellation = phy.modulation.generateConstellation(16);
 * ```
 *
 * @license MIT
 */

// ==================== ISAC Framework (Primary) ====================
export * as isac from './src/isac';
export * as core from './src/core';

// ==================== Research Tasks ====================
export * as tasks from './src/tasks';

// ==================== AI Interface ====================
export * as ai from './src/ai';

// ==================== Physical Models ====================
export * as channel from './src/models/channel';
export * as beamforming from './src/models/beamforming';
export * as sensing from './src/models/sensing';

// Merged: robotics = dynamics + control + drone utilities
export * as robotics from './src/models/robotics';

// Merged: planning = planning + trajectory
export * as planning from './src/models/planning';

// ==================== Physical Layer ====================
// Merged: phy = modulation + coding + spreading
export * as phy from './src/models/phy';

// ==================== Numerical Methods ====================
// Merged: numeric = math + optimization
export * as numeric from './src/models/numeric';

// ==================== Extras (Sub-path export recommended) ====================
// These are also available via sub-path: import { ... } from 'symbion/extras/...'
export * as extras from './src/extras';

// ==================== Version ====================
export const VERSION = '1.0.0';
