/**
 * @module src/models
 * @description Physical Models for ISAC Simulation
 *
 * Organized into logical categories:
 * - channel/: Channel models (AWGN, Rayleigh, Rician, GSCM, MIMO)
 * - beamforming/: Array processing and beam patterns
 * - sensing/: Radar and LiDAR sensing
 * - robotics/: Dynamics + Control + UAV utilities (merged)
 * - planning/: Path + Trajectory planning (merged)
 * - phy/: Physical layer (modulation + coding + spreading) (merged)
 * - numeric/: Numerical methods (math + optimization) (merged)
 */

// Core models (always needed for ISAC)
export * as channel from './channel';
export * as beamforming from './beamforming';
export * as sensing from './sensing';

// Robotics & Planning (merged modules)
export * as robotics from './robotics';
export * as planning from './planning';

// Physical Layer (merged: modulation + coding + spreading)
export * as phy from './phy';

// Numerical Methods (merged: math + optimization)
export * as numeric from './numeric';
