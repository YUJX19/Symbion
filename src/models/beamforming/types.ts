/**
 * @module beamforming/types
 * @description Type definitions for beamforming algorithms
 */

/**
 * Array configuration for antenna systems
 */
export interface ArrayConfig {
    /** Number of antenna elements */
    numAntennas: number;
    /** Element spacing in wavelengths */
    spacing: number;
    /** Steering angle in radians */
    steeringAngle?: number;
}

/**
 * Beam pattern data point
 */
export interface BeamPatternPoint {
    angle: number;
    magnitude: number;
    magnitudeDb: number;
}

/**
 * Polarization state
 */
export interface PolarizationState {
    /** Horizontal component magnitude */
    horizontal: number;
    /** Vertical component magnitude */
    vertical: number;
    /** Phase difference in radians */
    phaseDiff: number;
}
