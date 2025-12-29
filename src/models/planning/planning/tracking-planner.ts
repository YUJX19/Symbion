/**
 * @module planning/tracking-planner
 * @description Complete aerial tracking and landing planner.
 * 
 * Integrates target prediction, visibility-constrained path planning,
 * safe flight corridor generation, and trajectory optimization for
 * aerial tracking and landing scenarios.
 * 
 */

import type { Vector3Array, PiecewiseTrajectory } from '../trajectory/types';
import {
    DEFAULT_TRACKING_CONFIG,
    DEFAULT_LANDING_CONFIG,
    isOccupied,
    type OccupancyGrid,
    type TargetState,
    type TrackingPlannerConfig,
    type LandingConfig,
    type VisibilityRegion,
    type TargetPredictionResult,
    type VisibilityPathResult
} from './tracking-types';

import { predictTargetMotion, predictLinearMotion } from './target-prediction';
import { findVisibilityConstrainedPath, findShortestPath, smoothPath } from './visibility-path';
import { computeVisibilityRegions } from './visibility-region';
import { generateMinimumSnapTrajectory, evaluateTrajectory } from './minco';
import { generateOBBCorridor, inflateCorridorFIRI, DEFAULT_OBB_OPTIONS } from './corridor';
import { generateCorridorConstrainedTrajectory, type CorridorOptimizationConfig } from './minco-corridor-optimizer';
import type { HPolyhedron } from './types';

// Re-export types and defaults
export { DEFAULT_TRACKING_CONFIG, DEFAULT_LANDING_CONFIG } from './tracking-types';


// ==================== Planner State ====================

/**
 * State of the tracking planner
 */
export interface TrackingPlannerState {
    /** Current drone state */
    drone: {
        position: Vector3Array;
        velocity: Vector3Array;
        orientation: [number, number, number, number]; // quaternion
    };
    /** Latest target state */
    target: TargetState;
    /** Current trajectory being executed */
    currentTrajectory: PiecewiseTrajectory | null;
    /** Trajectory start time */
    trajectoryStartTime: number;
    /** Is landing mode active */
    landingMode: boolean;
    /** Planner mode: 'tracking' | 'landing' | 'hovering' */
    mode: 'tracking' | 'landing' | 'hovering';
}

/**
 * Planner output
 */
export interface TrackingPlannerOutput {
    /** Planned trajectory */
    trajectory: PiecewiseTrajectory | null;
    /** Path waypoints for visualization */
    path: Vector3Array[];
    /** Predicted target positions */
    targetPrediction: Vector3Array[];
    /** Visibility regions */
    visibilityRegions: VisibilityRegion[];
    /** Safe flight corridors (H-polyhedra) */
    corridors: HPolyhedron[];
    /** Desired yaw angle */
    yaw: number;
    /** Planning success */
    success: boolean;
    /** Status message */
    message: string;
    /** Corridor constraint violation after optimization */
    corridorViolations?: number;
    /** Total trajectory time */
    totalTime?: number;
}

// ==================== Main Tracking Planner ====================

/**
 * Aerial tracking planner.
 * 
 * 1. Predict target motion
 * 2. Find visibility-constrained path
 * 3. Generate visibility regions
 * 4. Generate safe flight corridor
 * 5. Optimize trajectory with tracking/visibility constraints
 * 
 * @example
 * ```typescript
 * const planner = new AerialTrackingPlanner(config, occupancyGrid);
 * 
 * // Update states each iteration
 * planner.updateDroneState(dronePos, droneVel, droneQuat);
 * planner.updateTargetState(targetPos, targetVel);
 * 
 * // Plan trajectory
 * const output = planner.plan();
 * if (output.success) {
 *   // Execute trajectory
 * }
 * ```
 */
export class AerialTrackingPlanner {
    private config: TrackingPlannerConfig;
    private landingConfig: LandingConfig;
    private map: OccupancyGrid | null;
    private state: TrackingPlannerState;

    // Corridor optimization configuration
    private corridorConfig: Partial<CorridorOptimizationConfig> = {
        timeWeight: 1.0,
        energyWeight: 1.0,
        corridorWeight: 100.0,
        maxIterations: 10,  // Reduced from 30 for faster execution
        verbose: false,
    };

    // Enable/disable corridor optimization
    // DISABLED by default: Current path planning only returns 2 points which causes
    // L-BFGS to struggle with many constraint violations. Enable when proper A* 
    // path planning with multiple waypoints is integrated.
    private useCorridorOptimization: boolean = false;

    constructor(
        config: Partial<TrackingPlannerConfig> = {},
        map: OccupancyGrid | null = null
    ) {
        this.config = { ...DEFAULT_TRACKING_CONFIG, ...config };
        this.landingConfig = { ...DEFAULT_LANDING_CONFIG };
        this.map = map;

        this.state = {
            drone: {
                position: [0, 0, 0],
                velocity: [0, 0, 0],
                orientation: [1, 0, 0, 0]
            },
            target: {
                position: [0, 0, 0],
                velocity: [0, 0, 0]
            },
            currentTrajectory: null,
            trajectoryStartTime: 0,
            landingMode: false,
            mode: 'hovering'
        };
    }

    // ==================== State Updates ====================

    /**
     * Update drone state
     */
    updateDroneState(
        position: Vector3Array,
        velocity: Vector3Array,
        orientation: [number, number, number, number] = [1, 0, 0, 0]
    ): void {
        this.state.drone.position = [...position] as Vector3Array;
        this.state.drone.velocity = [...velocity] as Vector3Array;
        this.state.drone.orientation = [...orientation] as [number, number, number, number];
    }

    /**
     * Update target state
     */
    updateTargetState(
        position: Vector3Array,
        velocity: Vector3Array,
        orientation?: [number, number, number, number]
    ): void {
        this.state.target = {
            position: [...position] as Vector3Array,
            velocity: [...velocity] as Vector3Array,
            orientation,
            timestamp: performance.now() / 1000
        };
    }

    /**
     * Update occupancy map
     */
    updateMap(map: OccupancyGrid): void {
        this.map = map;
    }

    /**
     * Trigger landing mode
     */
    triggerLanding(landingOffset?: Vector3Array): void {
        this.state.landingMode = true;
        this.state.mode = 'landing';
        if (landingOffset) {
            this.landingConfig.landingOffset = landingOffset;
        }
    }

    /**
     * Cancel landing and return to tracking
     */
    cancelLanding(): void {
        this.state.landingMode = false;
        this.state.mode = 'tracking';
    }

    // ==================== Main Planning Function ====================

    /**
     * Execute one planning iteration.
     * 
     * @returns Planned trajectory and visualization data
     */
    plan(): TrackingPlannerOutput {
        const dronePos = this.state.drone.position;
        const droneVel = this.state.drone.velocity;
        const targetPos = this.state.target.position;
        const targetVel = this.state.target.velocity;

        // Check if we should hover (target close and slow)
        if (this.shouldHover()) {
            return this.createHoverOutput('Target reached, hovering');
        }

        // Predict target motion
        const prediction = this.predictTarget();
        if (!prediction.success) {
            return this.createFailedOutput(`Prediction failed: ${prediction.errorMessage}`);
        }

        // Compute observation target point
        let observationTarget: Vector3Array[];
        if (this.state.landingMode) {
            // Landing mode: target is the landing point on vehicle
            observationTarget = this.computeLandingTargets(prediction.positions);
        } else {
            // Tracking mode: target is at tracking distance above target
            observationTarget = prediction.positions.map(p => [
                p[0], p[1], p[2] + 1.0  // 1m above target
            ] as Vector3Array);
        }

        // Find visibility-constrained path (or direct path for landing)
        let pathResult: VisibilityPathResult;
        if (this.state.landingMode) {
            // Direct A* to landing point
            const shortResult = findShortestPath(
                dronePos,
                observationTarget[observationTarget.length - 1],
                this.map!,
                0.2
            );
            pathResult = {
                success: shortResult.success,
                path: shortResult.path,
                waypoints: [observationTarget[observationTarget.length - 1]],
                errorMessage: shortResult.errorMessage
            };
        } else {
            // Visibility-constrained path for tracking
            if (this.map) {
                pathResult = findVisibilityConstrainedPath(
                    dronePos,
                    observationTarget,
                    this.map,
                    this.config.trackingDistance,
                    { toleranceDistance: this.config.distanceTolerance }
                );
            } else {
                // No map: direct line to observation point
                pathResult = {
                    success: true,
                    path: [dronePos, observationTarget[observationTarget.length - 1]],
                    waypoints: [observationTarget[observationTarget.length - 1]]
                };
            }
        }

        if (!pathResult.success) {
            return this.createFailedOutput(`Path planning failed: ${pathResult.errorMessage}`);
        }

        // Compute visibility regions (for tracking mode only)
        let visibilityRegions: VisibilityRegion[] = [];
        if (!this.state.landingMode && this.map && pathResult.waypoints.length > 0) {
            const targets = observationTarget.slice(0, pathResult.waypoints.length);
            visibilityRegions = computeVisibilityRegions(
                targets,
                pathResult.waypoints,
                this.map,
                this.config.trackingDistance
            );
        }

        // Smooth path
        const smoothedPath = this.map ? smoothPath(pathResult.path, this.map) : pathResult.path;

        // ============ CORRIDOR GENERATION (NEW) ============
        // Generate safe flight corridors for trajectory optimization
        let corridors: HPolyhedron[] = [];
        if (smoothedPath.length >= 2) {
            try {
                // Use OBB corridor generation (faster, deterministic)
                corridors = generateOBBCorridor(smoothedPath, {
                    halfWidth: this.config.corridorHalfWidth || 2.0,
                    halfHeight: this.config.corridorHalfHeight || 2.0,
                    waypointRadius: (this.config.corridorHalfWidth || 2.0) * 1.2,
                    radiusGrowthFactor: 0.8,
                    sphereFaces: 20,
                    segmentOverlap: 0.5
                });
            } catch (e) {
                console.warn('[TrackingPlanner] Corridor generation failed:', e);
            }
        }

        // ============ TRAJECTORY WITH CORRIDOR OPTIMIZATION (NEW) ============
        let trajectory: PiecewiseTrajectory | null = null;
        let corridorViolations = 0;
        let totalTime = 0;

        if (this.useCorridorOptimization && corridors.length > 0 && smoothedPath.length >= 2) {
            // Use L-BFGS optimized trajectory with corridor constraints
            try {
                // Estimate total time
                let totalDist = 0;
                for (let i = 1; i < smoothedPath.length; i++) {
                    const dx = smoothedPath[i][0] - smoothedPath[i - 1][0];
                    const dy = smoothedPath[i][1] - smoothedPath[i - 1][1];
                    const dz = smoothedPath[i][2] - smoothedPath[i - 1][2];
                    totalDist += Math.sqrt(dx * dx + dy * dy + dz * dz);
                }
                const avgSpeed = Math.min(this.config.maxVelocity * 0.7, 2.0);
                const timeHint = Math.max(totalDist / avgSpeed, 1.0);

                const result = generateCorridorConstrainedTrajectory(
                    smoothedPath,
                    timeHint,
                    corridors,
                    {
                        initialVelocity: this.state.drone.velocity,
                        finalVelocity: this.state.landingMode ? [0, 0, 0] : targetVel
                    },
                    {
                        ...this.corridorConfig,
                        robotId: 'tracker'
                    }
                );

                trajectory = result.trajectory;
                corridorViolations = result.optimizationResult.violatedPoints;
                totalTime = result.optimizationResult.totalTime;

            } catch (e) {
                console.warn('[TrackingPlanner] Corridor optimization failed, falling back:', e);
                trajectory = this.generateTrajectory(smoothedPath, targetVel);
            }
        } else {
            // Fallback: simple trajectory without corridor constraints
            trajectory = this.generateTrajectory(smoothedPath, targetVel);
        }

        // Calculate desired yaw (toward target)
        const dp: Vector3Array = [
            targetPos[0] - dronePos[0],
            targetPos[1] - dronePos[1],
            targetPos[2] - dronePos[2]
        ];
        let yaw = Math.atan2(dp[1], dp[0]);

        // For landing, align with vehicle orientation
        if (this.state.landingMode && this.state.target.orientation) {
            const [w, x, y, z] = this.state.target.orientation;
            yaw = 2 * Math.atan2(z, w);  // Extract yaw from quaternion
        }

        // Validate trajectory
        if (trajectory && this.map) {
            const isValid = this.validateTrajectory(trajectory);
            if (!isValid) {
                return this.createFailedOutput('Trajectory validation failed');
            }
        }

        // Update state
        if (trajectory) {
            this.state.currentTrajectory = trajectory;
            this.state.trajectoryStartTime = performance.now() / 1000;
            this.state.mode = this.state.landingMode ? 'landing' : 'tracking';
        }

        return {
            trajectory,
            path: smoothedPath,
            targetPrediction: prediction.positions,
            visibilityRegions,
            corridors,
            yaw,
            success: trajectory !== null,
            message: trajectory ? 'Planning successful' : 'Trajectory generation failed',
            corridorViolations,
            totalTime
        };
    }

    // ==================== Helper Methods ====================

    private shouldHover(): boolean {
        const dronePos = this.state.drone.position;
        const droneVel = this.state.drone.velocity;
        const targetPos = this.state.target.position;
        const targetVel = this.state.target.velocity;

        const droneSpeed = Math.sqrt(
            droneVel[0] ** 2 + droneVel[1] ** 2 + droneVel[2] ** 2
        );
        const targetSpeed = Math.sqrt(
            targetVel[0] ** 2 + targetVel[1] ** 2 + targetVel[2] ** 2
        );

        if (this.state.landingMode) {
            // Hover if very close to target and both slow
            const dist = Math.sqrt(
                (dronePos[0] - targetPos[0]) ** 2 +
                (dronePos[1] - targetPos[1]) ** 2 +
                (dronePos[2] - targetPos[2]) ** 2
            );
            return dist < 0.1 && droneSpeed < 0.1 && targetSpeed < 0.2;
        } else {
            // Hover if at tracking distance and facing target
            const horizDist = Math.sqrt(
                (dronePos[0] - targetPos[0]) ** 2 +
                (dronePos[1] - targetPos[1]) ** 2
            );
            const distError = Math.abs(horizDist - this.config.trackingDistance);
            return distError < this.config.distanceTolerance &&
                droneSpeed < 0.1 &&
                targetSpeed < 0.2;
        }
    }

    private predictTarget(): TargetPredictionResult {
        // Use A* prediction if map available, otherwise linear
        if (this.map) {
            return predictTargetMotion(
                this.state.target,
                this.map,
                this.config.predictionDuration,
                {
                    timeStep: this.config.predictionTimeStep,
                    maxVelocity: 5.0,
                    maxAcceleration: 3.0,
                    maxComputationTime: 0.05
                }
            );
        } else {
            return predictLinearMotion(
                this.state.target,
                this.config.predictionDuration,
                this.config.predictionTimeStep
            );
        }
    }

    private computeLandingTargets(predictedPositions: Vector3Array[]): Vector3Array[] {
        const offset = this.landingConfig.landingOffset;
        const targetQ = this.state.target.orientation || [1, 0, 0, 0];

        return predictedPositions.map(pos => {
            // Rotate offset by target orientation
            const rotatedOffset = this.rotateByQuaternion(offset, targetQ);
            return [
                pos[0] + rotatedOffset[0],
                pos[1] + rotatedOffset[1],
                pos[2] + rotatedOffset[2]
            ] as Vector3Array;
        });
    }

    private rotateByQuaternion(
        v: Vector3Array,
        q: [number, number, number, number]
    ): Vector3Array {
        const [w, x, y, z] = q;
        const [vx, vy, vz] = v;

        // Quaternion rotation: q * v * q^-1
        // Simplified for unit quaternion
        const t0 = 2 * (w * vx + y * vz - z * vy);
        const t1 = 2 * (w * vy + z * vx - x * vz);
        const t2 = 2 * (w * vz + x * vy - y * vx);

        return [
            vx + w * t0 + y * t2 - z * t1,
            vy + w * t1 + z * t0 - x * t2,
            vz + w * t2 + x * t1 - y * t0
        ];
    }

    private generateTrajectory(
        path: Vector3Array[],
        finalVelocity: Vector3Array
    ): PiecewiseTrajectory | null {
        if (path.length < 2) return null;

        try {
            // Estimate total time based on path length and max velocity
            let totalDist = 0;
            for (let i = 1; i < path.length; i++) {
                const dx = path[i][0] - path[i - 1][0];
                const dy = path[i][1] - path[i - 1][1];
                const dz = path[i][2] - path[i - 1][2];
                totalDist += Math.sqrt(dx * dx + dy * dy + dz * dz);
            }

            const avgSpeed = Math.min(this.config.maxVelocity * 0.7, 2.0);
            const totalTime = Math.max(totalDist / avgSpeed, 1.0);

            return generateMinimumSnapTrajectory(
                path,
                totalTime,
                {
                    initialVelocity: this.state.drone.velocity,
                    finalVelocity: this.state.landingMode ? [0, 0, 0] : finalVelocity
                }
            );
        } catch (e) {
            console.error('Trajectory generation failed:', e);
            return null;
        }
    }

    private validateTrajectory(trajectory: PiecewiseTrajectory): boolean {
        if (!this.map) return true;

        // Sample trajectory and check for collisions
        const totalDuration = trajectory.pieces.reduce((sum, p) => sum + p.duration, 0);
        const checkInterval = 0.1;

        for (let t = 0; t <= totalDuration; t += checkInterval) {
            const state = evaluateTrajectory(trajectory, t);
            const pos = state.position;
            // Simple collision check using occupancy grid
            if (isOccupied(pos, this.map)) {
                return false;
            }
        }

        return true;
    }


    private createHoverOutput(message: string): TrackingPlannerOutput {
        this.state.mode = 'hovering';
        return {
            trajectory: null,
            path: [this.state.drone.position],
            targetPrediction: [],
            visibilityRegions: [],
            corridors: [],
            yaw: Math.atan2(
                this.state.target.position[1] - this.state.drone.position[1],
                this.state.target.position[0] - this.state.drone.position[0]
            ),
            success: true,
            message
        };
    }

    private createFailedOutput(message: string): TrackingPlannerOutput {
        return {
            trajectory: null,
            path: [],
            targetPrediction: [],
            visibilityRegions: [],
            corridors: [],
            yaw: 0,
            success: false,
            message
        };
    }

    // ==================== Getters ====================

    getState(): TrackingPlannerState {
        return this.state;
    }

    getConfig(): TrackingPlannerConfig {
        return this.config;
    }

    getMode(): 'tracking' | 'landing' | 'hovering' {
        return this.state.mode;
    }

    /**
     * Get current trajectory position at given time
     */
    getTrajectoryPosition(time: number): Vector3Array | null {
        if (!this.state.currentTrajectory) return null;

        const relativeTime = time - this.state.trajectoryStartTime;
        if (relativeTime < 0) return null;

        const state = evaluateTrajectory(this.state.currentTrajectory, relativeTime);
        return state.position;
    }
}

// ==================== Convenience Functions ====================

/**
 * Create a simple tracking planner with default configuration.
 * 
 * @param trackingDistance Desired tracking distance
 * @param map Optional occupancy grid
 * @returns Configured planner instance
 */
export function createTrackingPlanner(
    trackingDistance: number = 5.0,
    map: OccupancyGrid | null = null
): AerialTrackingPlanner {
    return new AerialTrackingPlanner(
        { trackingDistance },
        map
    );
}

/**
 * One-shot planning function for simple use cases.
 * 
 * @param droneState Current drone state
 * @param targetState Current target state
 * @param trackingDistance Desired tracking distance
 * @param map Optional occupancy grid
 * @returns Planning result
 */
export function planTrackingTrajectory(
    droneState: { position: Vector3Array; velocity: Vector3Array },
    targetState: { position: Vector3Array; velocity: Vector3Array },
    trackingDistance: number = 5.0,
    map: OccupancyGrid | null = null
): TrackingPlannerOutput {
    const planner = createTrackingPlanner(trackingDistance, map);
    planner.updateDroneState(droneState.position, droneState.velocity);
    planner.updateTargetState(targetState.position, targetState.velocity);
    return planner.plan();
}
