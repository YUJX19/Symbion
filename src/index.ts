/**
 * @module src
 * @description Symbion Source Module Entry Point
 *
 * New organized structure:
 * - core/: Core framework (space, objective, constraint, runner, logging, errors)
 * - ai/: AI interface and tasks
 * - isac/: ISAC-specific modules (planner, metrics, constraints)
 * - models/: Physical models (channel, beamforming, sensing, robotics, planning)
 * - tasks/: Research tasks (u2u-mcs, isac-trajectory)
 * - extras/: Experimental modules (networking, phy-coding, sandbox2d)
 */

// ==================== Core Framework ====================
export * as core from './core';

// ==================== AI Interface ====================
export * as ai from './ai';

// ==================== ISAC Module ====================
export * as isac from './isac';

// ==================== Physical Models ====================
export * as models from './models';

// ==================== Research Tasks ====================
export * as tasks from './tasks';

// ==================== Extras ====================
export * as extras from './extras';
