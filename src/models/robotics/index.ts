/**
 * @module src/models/robotics
 * @description Robotics: Dynamics, Control, and UAV utilities
 *
 * Merged from: dynamics + control + drone (utility parts)
 *
 * Contains:
 * - Dynamics: Quadrotor dynamics, kinematics, state representation
 * - Control: Differential flatness mapping, cascade controller, quaternion utils
 * - UAV: Distance, angle normalization utilities
 *
 * Note: 2D sandbox components (Robot, Obstacle, LiDAR) moved to extras/sandbox2d
 */

import * as dynamics from './dynamics';
import * as control from './control';
import * as drone from './drone';

// Re-export as namespaces
export { dynamics, control, drone };
