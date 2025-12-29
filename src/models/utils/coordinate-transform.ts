/**
 * @module utils/coordinate-transform
 * @description Coordinate system transformation utilities between Three.js and Physics/MINCO.
 * 
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                    COORDINATE SYSTEM CONVENTIONS                          ║
 * ╠═══════════════════════════════════════════════════════════════════════════╣
 * ║                                                                           ║
 * ║  Three.js (Y-up, right-handed, graphics convention):                     ║
 * ║    +X = Right (→)                                                         ║
 * ║    +Y = Up (↑)                                                            ║
 * ║    +Z = Out of screen (towards viewer)                                    ║
 * ║                                                                           ║
 * ║  Physics/MINCO (Z-up, right-handed, aerospace/robotics convention):      ║
 * ║    +X = Forward (body front direction)                                    ║
 * ║    +Y = Right (body right wing)                                          ║
 * ║    +Z = Up (opposite to gravity)                                         ║
 * ║                                                                           ║
 * ╠═══════════════════════════════════════════════════════════════════════════╣
 * ║                         TRANSFORMATION MAPPING                            ║
 * ╠═══════════════════════════════════════════════════════════════════════════╣
 * ║                                                                           ║
 * ║  Physics → Three.js:                                                      ║
 * ║    physics.x (forward) → three.z (into screen)                           ║
 * ║    physics.y (right)   → three.x (right)                                 ║
 * ║    physics.z (up)      → three.y (up)                                    ║
 * ║                                                                           ║
 * ║    physicsToThree([x, y, z]) = [y, z, x]                                 ║
 * ║                                                                           ║
 * ║  Three.js → Physics:                                                      ║
 * ║    three.x (right)     → physics.y (right)                               ║
 * ║    three.y (up)        → physics.z (up)                                  ║
 * ║    three.z (forward)   → physics.x (forward)                             ║
 * ║                                                                           ║
 * ║    threeToPhysics([x, y, z]) = [z, x, y]                                 ║
 * ║                                                                           ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 * 
 * IMPORTANT: All coordinates in physics/MINCO modules (controller, dynamics, 
 * trajectory optimization) use the Physics convention. Only convert to Three.js
 * when rendering visuals.
 */

import type { Vector3Array, TrajectoryState } from '../planning/trajectory/types';
import type { Quaternion } from '../robotics/control/types';
import type { QuadrotorState3D } from '../robotics/dynamics/types';

// ==================== Position Transforms ====================

/**
 * Convert a position from Physics/MINCO to Three.js coordinate system.
 * 
 * @param physPos - Position in Physics coordinates [x_forward, y_right, z_up]
 * @returns Position in Three.js coordinates [x_right, y_up, z_forward]
 * 
 * @example
 * const physPos = [100, 0, 15]; // 100m forward, 0m right, 15m altitude
 * const threePos = physicsToThree(physPos); // [0, 15, 100]
 */
export function physicsToThree(physPos: Vector3Array): Vector3Array {
    return [physPos[1], physPos[2], physPos[0]];
}

/**
 * Convert a position from Three.js to Physics/MINCO coordinate system.
 * 
 * @param threePos - Position in Three.js coordinates [x_right, y_up, z_forward]
 * @returns Position in Physics coordinates [x_forward, y_right, z_up]
 * 
 * @example
 * const threePos = [0, 15, 100]; // 0m right, 15m up, 100m forward
 * const physPos = threeToPhysics(threePos); // [100, 0, 15]
 */
export function threeToPhysics(threePos: Vector3Array): Vector3Array {
    return [threePos[2], threePos[0], threePos[1]];
}

// ==================== Aliases for Clarity ====================

/** Alias for physicsToThree - converts MINCO trajectory coordinates to Three.js */
export const mincoToThree = physicsToThree;

/** Alias for threeToPhysics - converts Three.js coordinates to MINCO */
export const threeToMinco = threeToPhysics;

// ==================== Velocity Transforms ====================

/**
 * Convert a velocity vector from Physics to Three.js coordinate system.
 * Same transformation as position since velocity is a vector.
 */
export function physicsToThreeVel(physVel: Vector3Array): Vector3Array {
    return physicsToThree(physVel);
}

/**
 * Convert a velocity vector from Three.js to Physics coordinate system.
 */
export function threeToPhysicsVel(threeVel: Vector3Array): Vector3Array {
    return threeToPhysics(threeVel);
}

// ==================== Batch Conversions ====================

/**
 * Convert an array of Physics/MINCO positions to Three.js coordinates.
 * Useful for converting trajectory waypoints for visualization.
 */
export function physicsPathToThree(path: Vector3Array[]): Vector3Array[] {
    return path.map(physicsToThree);
}

/**
 * Convert an array of Three.js positions to Physics/MINCO coordinates.
 * Useful for converting waypoints from UI to trajectory optimization.
 */
export function threePathToPhysics(path: Vector3Array[]): Vector3Array[] {
    return path.map(threeToPhysics);
}

// ==================== Quaternion Transforms ====================

/**
 * Convert a quaternion from Three.js to Physics coordinate system.
 * 
 * Three.js quaternion: {x, y, z, w} object format
 * Physics quaternion: [w, x, y, z] array format
 * 
 * The rotation axes are also transformed to match the coordinate system.
 */
export function threeToPhysicsQuat(threeQuat: { x: number; y: number; z: number; w: number }): Quaternion {
    // Transform the rotation axis from Three.js to Physics frame
    // Three.js rotation around X (right) → Physics rotation around Y (right)
    // Three.js rotation around Y (up) → Physics rotation around Z (up)
    // Three.js rotation around Z (forward) → Physics rotation around X (forward)
    return [threeQuat.w, threeQuat.z, threeQuat.x, threeQuat.y];
}

/**
 * Convert a quaternion from Physics to Three.js coordinate system.
 * 
 * Physics quaternion: [w, x, y, z] with Z-up
 * Three.js quaternion: {x, y, z, w} with Y-up
 */
export function physicsToThreeQuat(physQuat: Quaternion): { x: number; y: number; z: number; w: number } {
    // Inverse of the above transformation
    return {
        x: physQuat[2],  // physics Y → three X
        y: physQuat[3],  // physics Z → three Y
        z: physQuat[1],  // physics X → three Z
        w: physQuat[0]
    };
}

// ==================== Full State Transforms ====================

/**
 * Three.js representation of a drone state (for rendering)
 */
export interface ThreeJSDroneState {
    position: Vector3Array;  // [x, y, z] in Three.js coords
    velocity: Vector3Array;  // [vx, vy, vz] in Three.js coords
    quaternion: { x: number; y: number; z: number; w: number };
    angularVelocity: Vector3Array;  // [wx, wy, wz]
}

/**
 * Convert a full quadrotor state from Three.js to Physics coordinate system.
 * Use this BEFORE calling physics simulation functions.
 */
export function threeStateToPhysics(threeState: ThreeJSDroneState): QuadrotorState3D {
    return {
        position: threeToPhysics(threeState.position),
        velocity: threeToPhysicsVel(threeState.velocity),
        quaternion: threeToPhysicsQuat(threeState.quaternion),
        angularVelocity: threeToPhysicsVel(threeState.angularVelocity)
    };
}

/**
 * Convert a full quadrotor state from Physics to Three.js coordinate system.
 * Use this AFTER physics simulation to update the visual representation.
 */
export function physicsStateToThree(physState: QuadrotorState3D): ThreeJSDroneState {
    return {
        position: physicsToThree(physState.position),
        velocity: physicsToThreeVel(physState.velocity),
        quaternion: physicsToThreeQuat(physState.quaternion),
        angularVelocity: physicsToThreeVel(physState.angularVelocity)
    };
}

// ==================== Trajectory State Transforms ====================

/**
 * Convert a trajectory state from MINCO output (physics coords) to Three.js.
 * MINCO trajectory evaluator outputs positions in physics coordinate system.
 */
export function trajectoryStateToThree(physTrajState: TrajectoryState): TrajectoryState {
    return {
        position: physicsToThree(physTrajState.position),
        velocity: physicsToThreeVel(physTrajState.velocity),
        acceleration: physicsToThreeVel(physTrajState.acceleration),
        jerk: physTrajState.jerk ? physicsToThreeVel(physTrajState.jerk) : undefined
    };
}

/**
 * Convert a trajectory state from Three.js to Physics coordinates.
 * Use this when creating desired states for the controller.
 */
export function trajectoryStateToPhysics(threeTrajState: TrajectoryState): TrajectoryState {
    return {
        position: threeToPhysics(threeTrajState.position),
        velocity: threeToPhysicsVel(threeTrajState.velocity),
        acceleration: threeToPhysicsVel(threeTrajState.acceleration),
        jerk: threeTrajState.jerk ? threeToPhysicsVel(threeTrajState.jerk) : undefined
    };
}

// ==================== Helper: Create Initial States ====================

/**
 * Create an initial hovering state at the given Physics/MINCO position.
 * Use this when you already have coordinates in the Physics frame.
 * 
 * @param physPosition - Initial position in Physics coordinates [x_forward, y_right, z_up]
 * @returns QuadrotorState3D ready for physics simulation
 */
export function createHoverStateFromPhysics(physPosition: Vector3Array): QuadrotorState3D {
    return {
        position: [...physPosition] as Vector3Array,
        velocity: [0, 0, 0],
        quaternion: [1, 0, 0, 0] as Quaternion,  // Identity quaternion (level hover)
        angularVelocity: [0, 0, 0]
    };
}

/**
 * Create an initial hovering state at the given Three.js position.
 * Returns a physics-coordinate state suitable for simulation.
 * 
 * @param threePosition - Initial position in Three.js coordinates [x_right, y_up, z_forward]
 * @returns QuadrotorState3D ready for physics simulation
 */
export function createHoverStateFromThree(threePosition: Vector3Array): QuadrotorState3D {
    return {
        position: threeToPhysics(threePosition),
        velocity: [0, 0, 0],
        quaternion: [1, 0, 0, 0] as Quaternion,  // Identity quaternion (level hover)
        angularVelocity: [0, 0, 0]
    };
}

// ==================== Building/Obstacle Conversion ====================

/**
 * Building definition in Three.js coordinates (for visualization)
 */
export interface ThreeJSBuilding {
    position: { x: number; y: number; z: number };  // Center position
    width: number;   // Size along Three.js X axis
    height: number;  // Size along Three.js Y axis (vertical)
    depth: number;   // Size along Three.js Z axis
}

/**
 * Building definition in Physics coordinates (for path planning)
 */
export interface PhysicsBuilding {
    center: Vector3Array;  // [x_forward, y_right, z_up]
    sizeX: number;  // Size along forward axis
    sizeY: number;  // Size along right axis
    sizeZ: number;  // Size along up axis (height)
}

/**
 * Convert a Three.js building to Physics coordinates.
 * Useful for obstacle avoidance planning.
 */
export function threeBuildingToPhysics(building: ThreeJSBuilding): PhysicsBuilding {
    return {
        center: threeToPhysics([building.position.x, building.position.y, building.position.z]),
        sizeX: building.depth,   // Three.js depth (Z) → Physics forward (X)
        sizeY: building.width,   // Three.js width (X) → Physics right (Y)
        sizeZ: building.height   // Three.js height (Y) → Physics up (Z)
    };
}

/**
 * Convert an array of Three.js buildings to Physics coordinates.
 */
export function threeBuildingsToPhysics(buildings: ThreeJSBuilding[]): PhysicsBuilding[] {
    return buildings.map(threeBuildingToPhysics);
}

// ==================== Utility Functions ====================

/**
 * Check if two Vector3Arrays are approximately equal.
 */
export function vec3ApproxEqual(a: Vector3Array, b: Vector3Array, epsilon = 1e-6): boolean {
    return Math.abs(a[0] - b[0]) < epsilon &&
        Math.abs(a[1] - b[1]) < epsilon &&
        Math.abs(a[2] - b[2]) < epsilon;
}

/**
 * Verify that round-trip conversion preserves the original value.
 * Useful for debugging coordinate transformations.
 */
export function verifyRoundTrip(original: Vector3Array, direction: 'physics' | 'three'): boolean {
    if (direction === 'physics') {
        const converted = threeToPhysics(physicsToThree(original));
        return vec3ApproxEqual(original, converted);
    } else {
        const converted = physicsToThree(threeToPhysics(original));
        return vec3ApproxEqual(original, converted);
    }
}

/**
 * Get the gravity vector in Physics coordinates.
 * Gravity points in the -Z direction in the Physics frame.
 */
export function getGravityVector(g = 9.81): Vector3Array {
    return [0, 0, -g];
}

/**
 * Get the up vector in Physics coordinates.
 */
export function getPhysicsUpVector(): Vector3Array {
    return [0, 0, 1];
}

/**
 * Get the forward vector in Physics coordinates.
 */
export function getPhysicsForwardVector(): Vector3Array {
    return [1, 0, 0];
}
