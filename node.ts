/**
 * @packageDocumentation
 * @module symbion/node
 *
 * Node.js entry point for Symbion library.
 *
 * This entry point includes all modules, including Node.js-dependent
 * tasks (u2u-mcs, isac-trajectory) that require file I/O and subprocess
 * capabilities.
 *
 * ## All Available Modules
 *
 * ### ISAC Framework
 * - `isac` - ISAC-specific algorithms (planner, metrics, constraints, report)
 * - `core` - Reproducible experiment framework (space, objective, constraint, runner, logging)
 *
 * ### Research Tasks (Node.js only)
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
 * ### Extras
 * - `extras` - Experimental modules (networking, phy-coding, sandbox2d)
 *
 * ## Usage Example
 * ```typescript
 * // Explicit Node.js import
 * import { tasks, channel, isac } from 'symbion/node';
 *
 * // Run U2U-MCS task
 * const result = await tasks.u2uMcs.runTask2({
 *   seed: 42,
 *   totalEpisodes: 10,
 * });
 * ```
 *
 * @license MIT
 */

// Re-export everything from the main index
export * from './index';
