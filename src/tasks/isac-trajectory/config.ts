/**
 * @module tasks/isac-trajectory/config
 * @description ISAC Trajectory Task Configuration
 */

// ==================== Types ====================

/**
 * 3D Position
 */
export interface Position3D {
    x: number;
    y: number;
    z: number;
}

/**
 * Ground User configuration
 */
export interface GroundUserConfig {
    /** User ID */
    id: string;
    /** Position (x, y, 0) */
    position: Position3D;
    /** Minimum required rate (bps) */
    minRate: number;
    /** Maximum tolerable latency (ms) */
    maxLatencyMs: number;
    /** URLLC reliability requirement (1 - BLER) */
    reliabilityTarget: number;
    /** Whether user is URLLC or eMBB */
    isUrllc: boolean;
}

/**
 * Obstacle configuration (for LoS blocking)
 */
export interface ObstacleConfig {
    /** Obstacle ID */
    id: string;
    /** Center position */
    position: Position3D;
    /** Obstacle dimensions (width, depth, height) */
    dimensions: [number, number, number];
}

/**
 * UAV kinematics constraints
 */
export interface KinematicsConfig {
    /** Maximum speed (m/s) */
    maxSpeed: number;
    /** Maximum acceleration (m/s²) */
    maxAcceleration: number;
    /** Minimum altitude (m) */
    minAltitude: number;
    /** Maximum altitude (m) */
    maxAltitude: number;
    /** Hover power consumption (W) */
    hoverPower: number;
    /** Propulsion efficiency */
    propulsionEfficiency: number;
}

/**
 * Communication parameters
 */
export interface CommConfig {
    /** Carrier frequency (Hz) */
    carrierFrequencyHz: number;
    /** Bandwidth (Hz) */
    bandwidthHz: number;
    /** Transmit power (dBm) */
    txPowerDbm: number;
    /** Noise figure (dB) */
    noiseFigureDb: number;
    /** LoS probability model: 'urban' | 'suburban' | 'rural' */
    losModel: 'urban' | 'suburban' | 'rural';
    /** Path loss exponent for LoS */
    plExponentLos: number;
    /** Path loss exponent for NLoS */
    plExponentNlos: number;
    /** Rician K-factor for LoS (dB) */
    ricianKLosDb: number;
}

/**
 * Sensing parameters
 */
export interface SensingConfig {
    /** Radar cross section (m²) */
    radarCrossSection: number;
    /** Sensing SINR threshold (dB) */
    sensingSinrThresholdDb: number;
    /** Field of view angle (degrees) */
    fieldOfViewDeg: number;
}

/**
 * Reward weights
 */
export interface IsacRewardWeights {
    /** Weight for throughput */
    throughput: number;
    /** Weight for LoS percentage */
    losPersistence: number;
    /** Penalty for energy consumption */
    energy: number;
    /** Penalty for URLLC violations */
    urllcPenalty: number;
    /** Penalty for sensing coverage loss */
    sensingPenalty: number;
}

/**
 * Full ISAC Trajectory Task Configuration
 */
export interface IsacTrajectoryConfig {
    /** Random seed */
    seed: number;
    /** Number of planning steps (waypoints) */
    numWaypoints: number;
    /** Time per waypoint (seconds) */
    waypointDurationS: number;
    /** Flight area bounds [minX, minY, maxX, maxY] */
    areaBounds: [number, number, number, number];
    /** Start position */
    startPosition: Position3D;
    /** End position (optional, for point-to-point missions) */
    endPosition?: Position3D;
    /** Ground users */
    users: GroundUserConfig[];
    /** Obstacles */
    obstacles: ObstacleConfig[];
    /** UAV kinematics */
    kinematics: KinematicsConfig;
    /** Communication parameters */
    comm: CommConfig;
    /** Sensing parameters */
    sensing: SensingConfig;
    /** Reward weights */
    rewardWeights: IsacRewardWeights;
    /** URLLC BLER target */
    urllcBlerTarget: number;
    /** Episode horizon (steps) */
    episodeHorizon: number;
}

// ==================== Default Configuration ====================

export const DEFAULT_ISAC_CONFIG: IsacTrajectoryConfig = {
    seed: 42,
    numWaypoints: 20,
    waypointDurationS: 1.0,
    areaBounds: [-500, -500, 500, 500],
    startPosition: { x: 0, y: 0, z: 100 },
    users: [
        { id: 'u1', position: { x: 100, y: 0, z: 0 }, minRate: 1e6, maxLatencyMs: 1, reliabilityTarget: 0.99999, isUrllc: true },
        { id: 'u2', position: { x: -100, y: 50, z: 0 }, minRate: 5e6, maxLatencyMs: 100, reliabilityTarget: 0.99, isUrllc: false },
        { id: 'u3', position: { x: 0, y: 150, z: 0 }, minRate: 1e6, maxLatencyMs: 1, reliabilityTarget: 0.99999, isUrllc: true },
    ],
    obstacles: [
        { id: 'b1', position: { x: 50, y: 50, z: 15 }, dimensions: [30, 30, 30] },
        { id: 'b2', position: { x: -50, y: 100, z: 20 }, dimensions: [40, 40, 40] },
    ],
    kinematics: {
        maxSpeed: 20, // m/s
        maxAcceleration: 5, // m/s²
        minAltitude: 50, // m
        maxAltitude: 200, // m
        hoverPower: 100, // W
        propulsionEfficiency: 0.8,
    },
    comm: {
        carrierFrequencyHz: 3.5e9, // 3.5 GHz
        bandwidthHz: 20e6, // 20 MHz
        txPowerDbm: 30, // 1W
        noiseFigureDb: 9,
        losModel: 'suburban',
        plExponentLos: 2.0,
        plExponentNlos: 3.5,
        ricianKLosDb: 10,
    },
    sensing: {
        radarCrossSection: 1.0,
        sensingSinrThresholdDb: 10,
        fieldOfViewDeg: 60,
    },
    rewardWeights: {
        throughput: 1.0,
        losPersistence: 0.5,
        energy: 0.1,
        urllcPenalty: 10.0,
        sensingPenalty: 0.3,
    },
    urllcBlerTarget: 1e-5,
    episodeHorizon: 50,
};

// ==================== Factory Functions ====================

/**
 * Create configuration with overrides
 */
export function createConfig(overrides: Partial<IsacTrajectoryConfig> = {}): IsacTrajectoryConfig {
    return {
        ...DEFAULT_ISAC_CONFIG,
        ...overrides,
        kinematics: {
            ...DEFAULT_ISAC_CONFIG.kinematics,
            ...(overrides.kinematics ?? {}),
        },
        comm: {
            ...DEFAULT_ISAC_CONFIG.comm,
            ...(overrides.comm ?? {}),
        },
        sensing: {
            ...DEFAULT_ISAC_CONFIG.sensing,
            ...(overrides.sensing ?? {}),
        },
        rewardWeights: {
            ...DEFAULT_ISAC_CONFIG.rewardWeights,
            ...(overrides.rewardWeights ?? {}),
        },
    };
}

/**
 * LoS probability models (ITU-R P.1410-5 inspired)
 */
export const LOS_PROBABILITY_PARAMS = {
    urban: { a: 0.3, b: 500 },
    suburban: { a: 0.1, b: 750 },
    rural: { a: 0.05, b: 1000 },
} as const;

/**
 * Get LoS probability based on elevation angle
 */
export function getLosProbability(
    elevationAngleDeg: number,
    model: 'urban' | 'suburban' | 'rural'
): number {
    const { a, b } = LOS_PROBABILITY_PARAMS[model];
    // Simplified model: P_LoS = 1 / (1 + a * exp(-b * (theta - a)))
    const theta = Math.max(0, Math.min(90, elevationAngleDeg));
    return 1 / (1 + a * Math.exp(-b * (theta / 90 - a)));
}

// ==================== Task1 Configuration (Direct Optimization) ====================

import type { Vector3Array } from '../../models/planning/trajectory/types';

/**
 * Sphere obstacle for trajectory optimization (Task1)
 * Different from box-based ObstacleConfig used in RL environment
 */
export interface SphereObstacle {
    center: Vector3Array;
    radius: number;
}

/**
 * Task1 Configuration for MINCO + L-BFGS trajectory optimization
 * 
 * This is separate from IsacTrajectoryConfig which is used for RL environment.
 */
export interface Task1Config {
    // ===== Task Metadata =====
    taskName: string;
    seed: number;

    // ===== Scenario =====
    startPos: Vector3Array;
    endPos: Vector3Array;
    totalTime: number;
    numWaypoints: number;
    numTargetPoints: number;
    obstacles: SphereObstacle[];

    // ===== Dynamic Constraints =====
    maxVelocity: number;        // v_m (m/s)
    maxAcceleration: number;    // a_m (m/s²)
    safetyMargin: number;       // Obstacle safety margin (m)

    // ===== Communication Constraints (Eq. 37-38) =====
    minCommDistance: number;    // d_l (m)
    maxCommDistance: number;    // d_u (m)
    losConeAngle: number;       // θ_k (rad)

    // ===== RF Parameters =====
    fcGHz: number;              // Carrier frequency (GHz)
    bandwidthHz: number;        // Bandwidth (Hz)
    txPowerDbm: number;         // Transmit power (dBm)
    noisePowerDbm: number;      // Noise power (dBm)

    // ===== URLLC Parameters =====
    urllcLosThreshold: number;  // Min LoS probability for URLLC

    // ===== Optimization Weights =====
    weights: {
        smooth: number;         // W_o: Jerk energy
        obstacle: number;       // P1 obstacle
        dynamics: number;       // P1 velocity/acceleration
        distance: number;       // H1 distance window
        throughput: number;     // Communication throughput
        urllc: number;          // URLLC reliability
    };

    // ===== Two-Stage Sampling =====
    sampling: {
        dtOptimize: number;     // Coarse sampling for optimization
        dtValidate: number;     // Fine sampling for validation
    };

    // ===== Optimizer Settings =====
    optimizer: {
        maxIterations: number;
        gEpsilon: number;       // Gradient convergence threshold
        fdBaseStep: number;     // Central FD base step
        fdRelativeStep: number; // Central FD relative step
        enableCache: boolean;   // Enable gradient cache
        cacheRounding: number;  // Cache key rounding precision
    };
}

/**
 * Default Task1 configuration based on paper parameters
 */
export const DEFAULT_TASK1_CONFIG: Task1Config = {
    // Task Metadata
    taskName: 'isac-trajectory-task1',
    seed: 42,

    // Scenario
    startPos: [0, 30, 100],
    endPos: [150, 30, 100],
    totalTime: 30.0,
    numWaypoints: 12,
    numTargetPoints: 50,
    obstacles: [
        { center: [60, 35, 100], radius: 15 },
        { center: [100, 45, 100], radius: 10 }
    ],

    // Dynamic Constraints
    maxVelocity: 15.0,
    maxAcceleration: 10.0,
    safetyMargin: 2.0,

    // Communication Constraints
    minCommDistance: 10.0,
    maxCommDistance: 80.0,
    losConeAngle: Math.PI / 6,

    // RF Parameters  
    fcGHz: 2.9,
    bandwidthHz: 10e6,
    txPowerDbm: 20,
    noisePowerDbm: -97,

    // URLLC
    urllcLosThreshold: 0.8,

    // Weights (from paper)
    weights: {
        smooth: 1.0,
        obstacle: 10000.0,
        dynamics: 500.0,
        distance: 200.0,
        throughput: 0.1,
        urllc: 5000.0,
    },

    // Two-Stage Sampling
    sampling: {
        dtOptimize: 0.4,
        dtValidate: 0.1,
    },

    // Optimizer
    optimizer: {
        maxIterations: 80,
        gEpsilon: 1e-4,
        fdBaseStep: 1e-2,
        fdRelativeStep: 1e-4,
        enableCache: true,
        cacheRounding: 1e-3,
    },
};

/**
 * Deep merge user config with Task1 defaults
 */
export function mergeTask1Config(userCfg: Partial<Task1Config>): Task1Config {
    return {
        ...DEFAULT_TASK1_CONFIG,
        ...userCfg,
        weights: {
            ...DEFAULT_TASK1_CONFIG.weights,
            ...(userCfg.weights || {}),
        },
        sampling: {
            ...DEFAULT_TASK1_CONFIG.sampling,
            ...(userCfg.sampling || {}),
        },
        optimizer: {
            ...DEFAULT_TASK1_CONFIG.optimizer,
            ...(userCfg.optimizer || {}),
        },
        obstacles: userCfg.obstacles ?? DEFAULT_TASK1_CONFIG.obstacles,
    };
}

/**
 * Compute configuration hash for reproducibility
 */
export function computeTask1SchemaHash(cfg: Task1Config): string {
    const str = JSON.stringify({
        weights: cfg.weights,
        sampling: cfg.sampling,
        optimizer: cfg.optimizer,
    });
    let hash = 0;
    for (let i = 0; i < str.length; i++) {
        const char = str.charCodeAt(i);
        hash = ((hash << 5) - hash) + char;
        hash = hash & hash;
    }
    return Math.abs(hash).toString(16).slice(0, 8);
}
