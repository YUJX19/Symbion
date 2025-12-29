/**
 * @module src/models/planning
 * @description Path and Trajectory Planning
 *
 * Merged from: planning + trajectory
 *
 * Contains:
 * - Trajectory representation: Polynomial, PiecewiseTrajectory
 * - Path planning: A*, RRT*, MINCO
 * - Safety corridors, collision detection
 */

import * as planning from './planning';
import * as trajectory from './trajectory';

// Re-export with namespaces
export { planning, trajectory };

// Direct exports from planning (primary module)
export * from './planning';
