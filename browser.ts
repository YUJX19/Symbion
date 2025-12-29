/**
 * @packageDocumentation
 * @module symbion/browser
 *
 * Browser-compatible entry point for Symbion library.
 *
 * This entry point excludes Node.js-dependent modules (tasks) to ensure
 * compatibility with browser environments like Vite, Webpack, and other
 * browser bundlers.
 *
 * ## Available Modules
 *
 * ### ISAC Framework
 * - `isac` - ISAC-specific algorithms (planner, metrics, constraints, report)
 * - `core` - Reproducible experiment framework (space, objective, constraint, runner, logging)
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
 * ### Extras
 * - `extras` - Experimental modules (networking, phy-coding, sandbox2d)
 *
 * ## Usage Example
 * ```typescript
 * // Explicit browser import
 * import { channel, planning, robotics, phy, numeric } from 'symbion/browser';
 *
 * // Or automatic resolution via package.json exports
 * import { channel, planning } from 'symbion';
 *
 * const capacity = channel.awgn.shannonCapacity(10, 20e6);
 * ```
 *
 * ## Not Available in Browser
 * - `tasks` module (requires Node.js: fs, child_process, readline)
 *
 * @license MIT
 */

// ==================== ISAC Framework (Primary) ====================
export * as isac from './src/isac';
export * as core from './src/core';

// ==================== AI Interface (browser-compatible parts only) ====================
// Note: ai.tasks is NOT exported here as it depends on Node.js tasks module
export * as aiInterface from './src/ai/interface';

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

// ==================== Extras ====================
export * as extras from './src/extras';

// ==================== Version ====================
export const VERSION = '1.0.0';
