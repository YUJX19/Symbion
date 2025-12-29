/**
 * @module planning/target-prediction
 * @description Target motion prediction using A* search with dynamics constraints.
 * 
 * Predicts future trajectory of a moving target (e.g., vehicle, drone) by searching
 * through the space of possible accelerations while respecting velocity limits
 * and avoiding obstacles.
 * 
 */

import type { Vector3Array } from '../trajectory/types';
import {
    isOccupied,
    type OccupancyGrid,
    type TargetState,
    type TargetPredictionOptions,
    type TargetPredictionResult,
} from './tracking-types';
import { MinHeap } from './utils';

// ==================== Prediction Node ====================

interface PredictionNode {
    position: Vector3Array;
    velocity: Vector3Array;
    acceleration: Vector3Array;
    time: number;
    score: number;       // Accumulated cost
    heuristic: number;   // Estimated cost to goal
    parentIndex: number; // Index in node pool
}

// ==================== Main Prediction Function ====================


/**
 * Predict target motion trajectory using A* search with dynamics constraints.
 * 
 * The algorithm searches through possible accelerations to find a feasible
 * trajectory from the current state to a predicted end position. It considers:
 * - Velocity constraints
 * - Obstacle avoidance
 * - Acceleration smoothness (penalized)
 * 
 * @param targetState Current target state (position and velocity)
 * @param map Occupancy grid for collision checking
 * @param duration Total prediction duration in seconds
 * @param options Prediction parameters
 * @returns Predicted trajectory with positions, velocities, and timestamps
 * 
 * @example
 * ```typescript
 * const result = predictTargetMotion(
 *   { position: [0, 0, 0], velocity: [1, 0, 0] },
 *   occupancyGrid,
 *   3.0, // 3 seconds prediction
 *   { maxVelocity: 5.0, maxAcceleration: 3.0 }
 * );
 * if (result.success) {
 *   console.log('Predicted path:', result.positions);
 * }
 * ```
 */
export function predictTargetMotion(
    targetState: TargetState,
    map: OccupancyGrid | null,
    duration: number,
    options: TargetPredictionOptions = {}
): TargetPredictionResult {
    // Parse options with defaults
    const dt = options.timeStep ?? 0.2;
    const maxAcc = options.maxAcceleration ?? 3.0;
    const maxVel = options.maxVelocity ?? 5.0;
    const rhoA = options.accelerationPenalty ?? 1.0;
    const maxTime = options.maxComputationTime ?? 0.1;
    const groundHeight = options.groundHeight;

    // Calculate expected end position (straight line prediction)
    const endPosition: Vector3Array = [
        targetState.position[0] + targetState.velocity[0] * duration,
        targetState.position[1] + targetState.velocity[1] * duration,
        groundHeight !== undefined ? groundHeight : targetState.position[2] + targetState.velocity[2] * duration
    ];

    // Check validity function
    const isValid = (pos: Vector3Array, vel: Vector3Array): boolean => {
        const speed = Math.sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]);
        if (speed > maxVel) return false;
        if (map && isOccupied(pos, map)) return false;
        return true;
    };

    // Heuristic function: distance to expected end position
    const heuristic = (pos: Vector3Array): number => {
        const dx = pos[0] - endPosition[0];
        const dy = pos[1] - endPosition[1];
        const dz = pos[2] - endPosition[2];
        return 0.001 * Math.sqrt(dx * dx + dy * dy + dz * dz);
    };

    // Score function: acceleration magnitude
    const scoreFn = (acc: Vector3Array): number => {
        return rhoA * Math.sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    };

    // Initialize node pool and open set
    const nodePool: PredictionNode[] = [];
    const openSet = new MinHeap();
    const startTime = performance.now();

    // Create start node
    const startNode: PredictionNode = {
        position: [...targetState.position] as Vector3Array,
        velocity: [...targetState.velocity] as Vector3Array,
        acceleration: [0, 0, 0],
        time: 0,
        score: 0,
        heuristic: heuristic(targetState.position),
        parentIndex: -1
    };
    nodePool.push(startNode);

    // Discrete acceleration inputs: {-maxAcc, 0, maxAcc} for x and y
    // For ground vehicles, z acceleration is typically 0
    const accInputs: number[] = [-maxAcc, 0, maxAcc];
    const dt2_2 = dt * dt / 2;

    let currentIndex = 0;
    let bestEndIndex = 0;

    // A* search loop
    while (nodePool[currentIndex].time < duration) {
        const current = nodePool[currentIndex];

        // Check computation time limit
        const elapsedTime = (performance.now() - startTime) / 1000;
        if (elapsedTime > maxTime) {
            return {
                positions: [],
                velocities: [],
                times: [],
                success: false,
                errorMessage: `Prediction timeout: exceeded ${maxTime}s`
            };
        }

        // Explore all acceleration combinations (9 for 2D, 27 for 3D)
        for (const ax of accInputs) {
            for (const ay of accInputs) {
                // For ground vehicles, az = 0; for flying targets, expand to 3D
                const azOptions = groundHeight !== undefined ? [0] : accInputs;

                for (const az of azOptions) {
                    const input: Vector3Array = [ax, ay, az];

                    // Compute new position and velocity using kinematics
                    const newPos: Vector3Array = [
                        current.position[0] + current.velocity[0] * dt + input[0] * dt2_2,
                        current.position[1] + current.velocity[1] * dt + input[1] * dt2_2,
                        groundHeight !== undefined ? groundHeight :
                            current.position[2] + current.velocity[2] * dt + input[2] * dt2_2
                    ];

                    const newVel: Vector3Array = [
                        current.velocity[0] + input[0] * dt,
                        current.velocity[1] + input[1] * dt,
                        groundHeight !== undefined ? 0 : current.velocity[2] + input[2] * dt
                    ];

                    // Check validity
                    if (!isValid(newPos, newVel)) {
                        continue;
                    }

                    // Check memory limit
                    if (nodePool.length >= 100000) {
                        return {
                            positions: [],
                            velocities: [],
                            times: [],
                            success: false,
                            errorMessage: 'Prediction out of memory'
                        };
                    }

                    // Create new node
                    const newNode: PredictionNode = {
                        position: newPos,
                        velocity: newVel,
                        acceleration: input,
                        time: current.time + dt,
                        score: current.score + scoreFn(input),
                        heuristic: heuristic(newPos),
                        parentIndex: currentIndex
                    };

                    const nodeIndex = nodePool.length;
                    nodePool.push(newNode);
                    openSet.push(String(nodeIndex), newNode.score + newNode.heuristic);
                }
            }
        }

        // Get next best node
        if (openSet.isEmpty()) {
            return {
                positions: [],
                velocities: [],
                times: [],
                success: false,
                errorMessage: 'No valid prediction path found'
            };
        }

        const next = openSet.pop()!;
        currentIndex = parseInt(next.key);
        bestEndIndex = currentIndex;
    }

    // Reconstruct trajectory
    const positions: Vector3Array[] = [];
    const velocities: Vector3Array[] = [];
    const times: number[] = [];

    let traceIndex = bestEndIndex;
    while (traceIndex >= 0) {
        const node = nodePool[traceIndex];
        positions.unshift([...node.position] as Vector3Array);
        velocities.unshift([...node.velocity] as Vector3Array);
        times.unshift(node.time);
        traceIndex = node.parentIndex;
    }

    return {
        positions,
        velocities,
        times,
        success: true
    };
}

// ==================== Simple Linear Prediction ====================

/**
 * Simple linear motion prediction (constant velocity model).
 * 
 * Useful as a fast fallback when A* prediction times out or for
 * short-term prediction where acceleration changes are unlikely.
 * 
 * @param targetState Current target state
 * @param duration Prediction duration in seconds
 * @param timeStep Time step between prediction points
 * @param groundHeight Optional: constrain z to ground height
 * @returns Predicted trajectory
 */
export function predictLinearMotion(
    targetState: TargetState,
    duration: number,
    timeStep: number = 0.1,
    groundHeight?: number
): TargetPredictionResult {
    const positions: Vector3Array[] = [];
    const velocities: Vector3Array[] = [];
    const times: number[] = [];

    const numSteps = Math.ceil(duration / timeStep);

    for (let i = 0; i <= numSteps; i++) {
        const t = Math.min(i * timeStep, duration);
        times.push(t);

        positions.push([
            targetState.position[0] + targetState.velocity[0] * t,
            targetState.position[1] + targetState.velocity[1] * t,
            groundHeight !== undefined ? groundHeight :
                targetState.position[2] + targetState.velocity[2] * t
        ]);

        velocities.push([...targetState.velocity] as Vector3Array);
    }

    return {
        positions,
        velocities,
        times,
        success: true
    };
}

// ==================== Kalman-style Prediction ====================

/**
 * Predict target motion using a simple constant acceleration model.
 * 
 * Assumes target continues with current velocity and gradually
 * decelerates to zero (friction-like behavior).
 * 
 * @param targetState Current target state
 * @param duration Prediction duration in seconds
 * @param timeStep Time step between prediction points
 * @param decelerationRate Rate of velocity decay (0-1, default 0.1)
 * @returns Predicted trajectory
 */
export function predictDeceleratingMotion(
    targetState: TargetState,
    duration: number,
    timeStep: number = 0.1,
    decelerationRate: number = 0.1
): TargetPredictionResult {
    const positions: Vector3Array[] = [];
    const velocities: Vector3Array[] = [];
    const times: number[] = [];

    let currentPos = [...targetState.position] as Vector3Array;
    let currentVel = [...targetState.velocity] as Vector3Array;
    let t = 0;

    while (t <= duration) {
        times.push(t);
        positions.push([...currentPos] as Vector3Array);
        velocities.push([...currentVel] as Vector3Array);

        // Update position
        currentPos = [
            currentPos[0] + currentVel[0] * timeStep,
            currentPos[1] + currentVel[1] * timeStep,
            currentPos[2] + currentVel[2] * timeStep
        ];

        // Apply deceleration
        const decay = Math.pow(1 - decelerationRate, timeStep);
        currentVel = [
            currentVel[0] * decay,
            currentVel[1] * decay,
            currentVel[2] * decay
        ];

        t += timeStep;
    }

    return {
        positions,
        velocities,
        times,
        success: true
    };
}
