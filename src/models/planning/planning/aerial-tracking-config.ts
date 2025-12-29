/**
 * @module planning/aerial-tracking-config
 * @description Configuration interface for aerial target tracking simulation.
 * 
 * Provides configuration types and defaults for UAV-based target tracking
 * with visibility constraints and trajectory optimization.
 */

import type { Vector3Array } from '../trajectory/types';

// ==================== Configuration Types ====================

/**
 * Configuration for aerial target tracking with visibility constraints.
 */
export interface AerialTrackingConfig {
    // ===== Tracking Parameters =====
    /** Desired tracking distance from target (meters) */
    trackingDistance: number;
    /** Prediction duration for target trajectory (seconds) */
    trackingDuration: number;
    /** Tolerance for tracking distance before triggering replan (meters) */
    toleranceDistance: number;
    /** Angular clearance for visibility margin (radians) */
    thetaClearance: number;

    // ===== Drone Dynamics =====
    /** Maximum velocity (m/s) */
    maxVelocity: number;
    /** Maximum acceleration (m/s²) */
    maxAcceleration: number;
    /** Cruise altitude for tracking (meters) */
    cruiseAltitude: number;

    // ===== Planning Frequencies =====
    /** Replanning frequency (Hz) */
    planHz: number;

    // ===== Optimization Weights =====
    /** Visibility constraint penalty weight */
    rhoVisibility: number;
    /** Tracking distance penalty weight */
    rhoTracking: number;
    /** Trajectory smoothness penalty (jerk minimization) */
    smoothnessPenalty: number;

    // ===== Corridor Parameters =====
    /** Safe flight corridor half-width (meters) */
    corridorHalfWidth: number;
    /** Safe flight corridor half-height (meters) */
    corridorHalfHeight: number;
}

/**
 * Result of a single tracking planning iteration.
 */
export interface TrackingPlanResult {
    /** Whether planning succeeded */
    success: boolean;
    /** Planned trajectory waypoints */
    path: Vector3Array[];
    /** Predicted target trajectory */
    targetPrediction: Vector3Array[];
    /** Visibility waypoints (observation points) */
    visibilityWaypoints: Vector3Array[];
    /** Visibility angles at each waypoint */
    visibilityAngles: number[];
    /** Safe flight corridor half-planes */
    corridors: number[][][];
    /** Desired yaw angle (toward target) */
    yaw: number;
    /** Total trajectory duration */
    duration: number;
    /** Planning status message */
    message: string;
}

// ==================== Default Configuration ====================

/**
 * Default configuration for aerial tracking with visibility constraints.
 */
export const DEFAULT_AERIAL_TRACKING_CONFIG: AerialTrackingConfig = {
    // Tracking parameters
    trackingDistance: 5.0,
    trackingDuration: 2.0,
    toleranceDistance: 0.5,
    thetaClearance: 0.2, // ~11 degrees

    // Drone dynamics
    maxVelocity: 4.0,
    maxAcceleration: 3.0,
    cruiseAltitude: 3.0,

    // Planning frequency
    planHz: 10,

    // Optimization weights
    rhoVisibility: 10000.0,
    rhoTracking: 1000.0,
    smoothnessPenalty: 100.0,

    // Corridor parameters
    corridorHalfWidth: 2.0,
    corridorHalfHeight: 2.0,
};

/**
 * Configuration with visibility constraints disabled.
 * Useful for comparison experiments.
 */
export const NO_VISIBILITY_CONFIG: Partial<AerialTrackingConfig> = {
    rhoVisibility: 0.0, // Disable visibility constraint
    rhoTracking: 10000.0, // Prioritize tracking distance only
};

// ==================== Experiment Parameters ====================

/**
 * Parameters exposed in experiment UI for interactive tuning.
 */
export interface TrackingExperimentParams {
    trackingDistance: number;
    targetSpeed: number;
    enableVisibility: number; // 0 or 1
    cruiseAltitude: number;
    obstacleDensity: number;
    maxVelocity: number;
}

/**
 * Default experiment parameters.
 */
export const DEFAULT_EXPERIMENT_PARAMS: TrackingExperimentParams = {
    trackingDistance: 5,
    targetSpeed: 3,
    enableVisibility: 1,
    cruiseAltitude: 3,
    obstacleDensity: 15,
    maxVelocity: 6,
};

// ==================== Helper Functions ====================

/**
 * Create AerialTrackingConfig from experiment UI parameters.
 */
export function createConfigFromExperiment(
    params: Partial<TrackingExperimentParams>
): AerialTrackingConfig {
    const p = { ...DEFAULT_EXPERIMENT_PARAMS, ...params };

    return {
        ...DEFAULT_AERIAL_TRACKING_CONFIG,
        trackingDistance: p.trackingDistance,
        maxVelocity: p.maxVelocity,
        cruiseAltitude: p.cruiseAltitude,
        rhoVisibility: p.enableVisibility ? 10000.0 : 0.0,
    };
}

/**
 * Validate configuration parameters.
 */
export function validateConfig(config: AerialTrackingConfig): { valid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (config.trackingDistance <= 0) {
        errors.push('Tracking distance must be positive');
    }
    if (config.maxVelocity <= 0) {
        errors.push('Max velocity must be positive');
    }
    if (config.maxAcceleration <= 0) {
        errors.push('Max acceleration must be positive');
    }
    if (config.planHz <= 0 || config.planHz > 100) {
        errors.push('Planning frequency must be between 0 and 100 Hz');
    }
    if (config.cruiseAltitude < 0.5) {
        errors.push('Cruise altitude must be at least 0.5m');
    }

    return {
        valid: errors.length === 0,
        errors
    };
}


