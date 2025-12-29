/**
 * @module src/models/numeric
 * @description Numerical Methods and Optimization
 *
 * Merged from: math + optimization
 *
 * Contains:
 * - Linear algebra: vec, mat operations
 * - Geometry: distance, angles, transforms
 * - Optimization: L-BFGS, SDLP
 */

import * as math from './math';
import * as optimization from './optimization';

// Re-export as namespaces
export { math, optimization };

// Direct exports for common functions
export * from './optimization';
