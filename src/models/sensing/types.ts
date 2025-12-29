/**
 * @module sensing/types
 * @description Type definitions for the Sensing module
 * 
 * Re-exports shared types from drone/types for consistency
 */

// Re-export shared types from drone/types to avoid duplication
export type {
    Vector2,
    Pose,
    RobotShape,
    ShapeType,
    FOVConfig,
    Obstacle,
    WorldConfig,
    Robot,
    GridMap,
} from '../robotics/drone/types';

// Import for local use
import type { Vector2 } from '../robotics/drone/types';

/**
 * 3D Vector representation
 */
export interface Vector3 {
    x: number;
    y: number;
    z: number;
}

/**
 * LiDAR Sensor Configuration
 */
export interface LiDARConfig {
    /** Maximum scanning range (m) */
    maxRange: number;
    /** Minimum scanning range (m) */
    minRange?: number;
    /** Field of View (degrees) */
    fov: number;
    /** Angular resolution (degrees) */
    resolution: number;
    /** Distance measurement noise standard deviation */
    noise?: number;
    /** Angular noise standard deviation */
    angleNoise?: number;
    /** Offset relative to the robot center [x, y, theta] */
    offset?: [number, number, number];
    /** Rendering transparency */
    alpha?: number;
}

/**
 * LiDAR Scan Result
 */
export interface LiDARScan {
    /** Scanning angles (radians) */
    angles: number[];
    /** Measured distances (m) */
    distances: number[];
    /** Whether an object was detected at the corresponding angle */
    detected: boolean[];
    /** Coordinates of hit points */
    hitPoints?: Vector2[];
}
