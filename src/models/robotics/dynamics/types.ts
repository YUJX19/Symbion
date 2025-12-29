/**
 * Dynamics module type definitions
 */

/**
 * Pose (position + orientation)
 */
export interface Pose {
    x: number;
    y: number;
    theta: number;  // Heading angle (radians)
}

/**
 * Velocity
 */
export interface Velocity {
    vx: number;     // Linear velocity in x direction
    vy: number;     // Linear velocity in y direction
    omega: number;  // Angular velocity (rad/s)
}

/**
 * Kinematics type
 */
export type KinematicsType = 'differential' | 'omni' | 'ackermann' | 'quadrotor';

/**
 * Kinematics noise model (α model used for odometry error)
 */
export interface KinematicsNoise {
    enabled: boolean;
    alpha: [number, number, number, number];  // [α1, α2, α3, α4]
}

/**
 * Robot shape
 */
export interface RobotShape {
    type: 'circle' | 'rectangle' | 'polygon';
    radius?: number;      // Circle radius
    width?: number;       // Rectangle width
    height?: number;      // Rectangle height
    vertices?: { x: number; y: number }[];  // Polygon vertices
    wheelbase?: number;   // Ackermann steering wheelbase
}

/**
 * 3D position array [x, y, z]
 */
export type Vector3Array = [number, number, number];

/**
 * Quaternion representation [w, x, y, z]
 */
export type Quaternion = [number, number, number, number];

/**
 * Full 3D state of a quadrotor
 */
export interface QuadrotorState3D {
    /** Position in world frame [x, y, z] (m) */
    position: Vector3Array;
    /** Velocity in world frame [vx, vy, vz] (m/s) */
    velocity: Vector3Array;
    /** Orientation as quaternion [w, x, y, z] */
    quaternion: Quaternion;
    /** Angular velocity in body frame [ωx, ωy, ωz] (rad/s) */
    angularVelocity: Vector3Array;
}

/**
 * Physical parameters for quadrotor dynamics
 */
export interface QuadrotorParams {
    /** Mass (kg) */
    mass: number;
    /** Inertia matrix diagonal [Ixx, Iyy, Izz] (kg·m²) */
    inertia: Vector3Array;
    /** Drag coefficients [kx, ky, kz] */
    dragCoeff: Vector3Array;
    /** Gravitational acceleration (m/s²) */
    gravity: number;
    /** Minimum thrust (N) */
    thrustMin: number;
    /** Maximum thrust (N) */
    thrustMax: number;
    /** Maximum torque [τmax_x, τmax_y, τmax_z] (N·m) */
    torqueMax: Vector3Array;
}
