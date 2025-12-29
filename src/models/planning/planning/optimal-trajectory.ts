import { generateSafeFlightCorridor, createBoundingBox } from './corridor';
import { generateMinimumSnapTrajectory, evaluateTrajectory } from './minco';
import { generateOptimizedTrajectory } from './minco-time-optimizer';
import type { Vector3Array, PiecewiseTrajectory, TrajectoryState as BaseTrajectoryState } from '../trajectory/types';
import type { HPolyhedron } from './types';
import type { Pose, Velocity, Obstacle, Robot, Vector2 } from '../../robotics/drone/types';

// Import flight controller and physics
import {
    CascadeController,
    DEFAULT_CONTROLLER_PARAMS,
    DEFAULT_QUADROTOR_PARAMS
} from '../../robotics/control/controller';
import {
    quadrotorPhysicsUpdate,
    createHoverState
} from '../../robotics/dynamics/physics';
import type { QuadrotorState3D, QuadrotorParams } from '../../robotics/dynamics/types';
import type { ControllerParams } from '../../robotics/control/types';

// Import optimization history
import {
    initOptimizationHistory,
    recordPlanningSummary,
    recordMINCOData,
    recordCorridorGeneration,
    getOptimizationHistory
} from './optimization-history';
export { getOptimizationHistory } from './optimization-history';
export type { OptimizationHistory, LBFGSIterationData, MINCOOptimizationData } from './optimization-history';


// ==================== Types ====================

export type { Vector2 };

export interface TrajectoryState {
    position: [number, number, number];
    velocity: [number, number, number];
    acceleration: [number, number, number];
}

export interface TrajectorySegment {
    coefficients: number[][];
    duration: number;
}

export interface OptimalTrajectory {
    segments: TrajectorySegment[];
    totalDuration: number;
    waypoints: Vector2[];
}

export interface PlannerConfig {
    maxVelocity: number;
    maxAcceleration: number;
    safetyMargin: number;
    gridResolution?: number;
    /** Enable L-BFGS time optimization (default: true) */
    enableTimeOptimization?: boolean;
    /** Enable verbose logging for optimization (default: false) */
    verboseOptimization?: boolean;
}

export interface PlannerState {
    trajectory: OptimalTrajectory | null;
    mincoTrajectory?: PiecewiseTrajectory; // Added MINCO trajectory
    corridors?: HPolyhedron[]; // Added corridors for visualization
    lastPlanTime: number;
    startTime: number;
    config: PlannerConfig;
    lastSafetyMargin: number;
    // 3D Flight Simulation State (optional)
    flightState?: QuadrotorState3D;
    controller?: CascadeController;
    quadParams?: QuadrotorParams;
}

// ==================== Constants ====================

const DEFAULT_CONFIG: PlannerConfig = {
    maxVelocity: 5.0,
    maxAcceleration: 10.0,
    safetyMargin: 0.5,
    gridResolution: 0.5,
    enableTimeOptimization: true,
    verboseOptimization: false,
};

// 3D simulation timestep (fixed for physics stability)
const PHYSICS_DT = 0.01; // 10ms = 100Hz

// Global planner cache (one instance per robot)
const plannerCache = new Map<string, PlannerState>();


// ==================== Path Visualization Cache ====================

export interface PathVisualization {
    aStarPath: Vector2[];
    optimizedPath: Vector2[];
    corridors?: {
        center: Vector2;
        vertices: Vector2[];  // 2D projection of corridor bounds
    }[];
    /** Detailed corridor constraint statistics */
    corridorStats?: {
        /** Total number of half-plane constraints across all segments */
        totalHalfPlanes: number;
        /** FIRI iterations used */
        firiIterations: number;
        /** Average corridor radius (if applicable) */
        avgCorridorRadius?: number;
        /** Per-segment half-plane counts */
        segmentHalfPlanes?: number[];
    };
    currentTrajectoryPoint?: Vector2;  // Where the drone should be on trajectory
    elapsedTime?: number;
    totalDuration?: number;
    segmentTimes?: number[];
    debugInfo?: {
        robotPosition?: { x: number; y: number; z?: number };
        trajPosition?: Vector2;
        positionError?: number;
        velocityCmd?: { x: number; y: number; z?: number };
        // MINCO optimization metrics
        lbfgsIterations?: number;
        corridorCost?: number;
        violatedPoints?: number;
        totalCost?: number;
        // Gate-specific metrics
        gateCount?: number;
        segmentCorridorCount?: number;
    };
    commMetrics?: {
        sinr: number;
        capacity: number;
        pathLoss: number;
        alignment: number;
        distance: number;
        isLoS: boolean;
    };
    timestamp: number;
}

// Use globalThis to share cache across module instances in Next.js
// This prevents HMR and module isolation from breaking cache sharing
interface GlobalWithCache {
    __pathVisualizationCache?: Map<string, PathVisualization>;
}

// Server-side fallback cache (for SSR)
let ssrFallbackCache: Map<string, PathVisualization> | null = null;

function getPathVisualizationCache(): Map<string, PathVisualization> {
    // Check if we're in browser environment
    if (typeof globalThis !== 'undefined' && typeof (globalThis as GlobalWithCache).__pathVisualizationCache !== 'undefined') {
        return (globalThis as GlobalWithCache).__pathVisualizationCache!;
    }

    if (typeof globalThis !== 'undefined') {
        (globalThis as GlobalWithCache).__pathVisualizationCache = new Map<string, PathVisualization>();
        return (globalThis as GlobalWithCache).__pathVisualizationCache!;
    }

    // Fallback for SSR (shouldn't be needed for browser-only code)
    if (!ssrFallbackCache) {
        ssrFallbackCache = new Map<string, PathVisualization>();
    }
    return ssrFallbackCache;
}


export function getPlannedPath(robotId: string): PathVisualization | null {
    const cache = getPathVisualizationCache();
    return cache.get(robotId) || null;
}


export function setPlannedPath(robotId: string, visualization: PathVisualization): void {
    const cache = getPathVisualizationCache();
    cache.set(robotId, visualization);
}

export function clearPathVisualization(robotId?: string): void {
    const cache = getPathVisualizationCache();
    if (robotId) {
        cache.delete(robotId);
    } else {
        cache.clear();
    }
}


export function clearTrajectoryCache(robotId?: string): void {
    const vizcache = getPathVisualizationCache();
    if (robotId) {
        plannerCache.delete(robotId);
        vizcache.delete(robotId);
    } else {
        plannerCache.clear();
        vizcache.clear();
    }
}

// ==================== Main Behavior ====================

export function optimalTrajectoryBehavior(
    robot: Robot,
    goal: Pose,
    obstacles: Obstacle[],
    neighbors: Robot[],
    currentTime: number = Date.now(),
    replanInterval: number = 1000,
    worldBounds: { minX: number; maxX: number; minY: number; maxY: number } = { minX: 0, maxX: 100, minY: 0, maxY: 100 },
    extraConfig: Partial<PlannerConfig> = {}
): Velocity {
    const robotId = robot.id || 'robot';
    const maxSpeed = robot.maxSpeed || 5.0;
    const robotRadius = robot.shape?.radius || 0.5;

    let state = plannerCache.get(robotId);
    const currentSafetyMargin = extraConfig.safetyMargin ?? (robotRadius + 0.3);

    if (!state) {
        state = {
            trajectory: null,
            lastPlanTime: 0,
            startTime: 0,
            config: { ...DEFAULT_CONFIG, ...extraConfig },
            lastSafetyMargin: currentSafetyMargin
        };
        plannerCache.set(robotId, state);
    }

    state.config.maxVelocity = maxSpeed;
    state.config.safetyMargin = currentSafetyMargin;
    if (extraConfig.gridResolution) {
        state.config.gridResolution = extraConfig.gridResolution;
    }

    const distToGoal = Math.sqrt(
        (goal.x - robot.pose.x) ** 2 + (goal.y - robot.pose.y) ** 2
    );

    let needReplan = !state.trajectory;

    // Check safety margin change
    if (!needReplan && Math.abs(state.lastSafetyMargin - currentSafetyMargin) > 0.01) {
        needReplan = true;
        state.lastSafetyMargin = currentSafetyMargin;
    }

    // Check if we need to replan based on trajectory tracking error
    // Only replan if we're significantly off-track (>3m) AND enough time has passed
    if (!needReplan && state.mincoTrajectory && state.startTime > 0) {
        const elapsed = (currentTime - state.startTime) / 1000.0;
        let totalDuration = 0;
        state.mincoTrajectory.pieces.forEach((p: { duration: number }) => totalDuration += p.duration);

        // Only check tracking error if the trajectory is still valid
        if (elapsed <= totalDuration) {
            const stateAtT = evaluateTrajectory(state.mincoTrajectory, elapsed);
            const errorX = stateAtT.position[0] - robot.pose.x;
            const errorY = stateAtT.position[1] - robot.pose.y;
            const trackingError = Math.sqrt(errorX * errorX + errorY * errorY);

            // Only replan if tracking error is large AND at least 2 seconds since last replan
            const minReplanInterval = 2000; // ms
            if (trackingError > 5.0 && (currentTime - state.lastPlanTime) > minReplanInterval) {
                console.log('[OptimalTrajectory] Large tracking error (' + trackingError.toFixed(2) + 'm), replanning...');
                needReplan = true;
            }
        } else {
            // Trajectory has finished, check if we're far from goal
            if (distToGoal > 2.0) {
                // Only replan if enough time has passed since last plan
                const minReplanInterval = 1000;
                if ((currentTime - state.lastPlanTime) > minReplanInterval) {
                    needReplan = true;
                }
            }
        }
    }

    if (needReplan) {
        try {
            const totalInflation = robotRadius + state.config.safetyMargin;
            const gridRes = state.config.gridResolution || 0.5;

            const waypoints = aStarPathPlanning(
                { x: robot.pose.x, y: robot.pose.y },
                { x: goal.x, y: goal.y },
                obstacles,
                worldBounds,
                gridRes,
                totalInflation
            );

            if (waypoints.length > 0) {
                waypoints[waypoints.length - 1] = { x: goal.x, y: goal.y };
            }

            if (waypoints.length >= 2) {
                // 1. Convert to 3D for MINCO/FIRI
                const waypoints3D: Vector3Array[] = waypoints.map(wp => [wp.x, wp.y, 2.0]);

                // 2. Obstacles to 3D Point Cloud
                const obstacleCloud: Vector3Array[] = [];
                obstacles.forEach(obs => {
                    if (obs.shape.type === 'polygon' && obs.shape.vertices) {
                        const verts = obs.shape.vertices;
                        const numVerts = verts.length;

                        // Iterate through edges and dense sample
                        for (let i = 0; i < numVerts; i++) {
                            const p1 = verts[i];
                            const p2 = verts[(i + 1) % numVerts];

                            // Add vertex
                            obstacleCloud.push([p1.x, p1.y, 2.0]);
                            obstacleCloud.push([p1.x, p1.y, 0.0]);
                            obstacleCloud.push([p1.x, p1.y, 4.0]);

                            // Sample edge
                            const dx = p2.x - p1.x;
                            const dy = p2.y - p1.y;
                            const len = Math.sqrt(dx * dx + dy * dy);
                            const sampleStep = 0.5; // Every 0.5 meters
                            const steps = Math.floor(len / sampleStep);

                            for (let s = 1; s < steps; s++) {
                                const t = s / (steps + 1); // Avoid duplicating endpoints
                                const px = p1.x + t * dx;
                                const py = p1.y + t * dy;
                                obstacleCloud.push([px, py, 2.0]);
                                obstacleCloud.push([px, py, 0.0]);
                                obstacleCloud.push([px, py, 4.0]);
                            }
                        }
                    } else if (obs.shape.type === 'rectangle' && obs.shape.width && obs.shape.height) {
                        // Convert rectangle to polygon vertices
                        const halfW = obs.shape.width / 2;
                        const halfH = obs.shape.height / 2;
                        const x = obs.position.x;
                        const y = obs.position.y;

                        const rectVerts = [
                            { x: x - halfW, y: y - halfH },
                            { x: x + halfW, y: y - halfH },
                            { x: x + halfW, y: y + halfH },
                            { x: x - halfW, y: y + halfH }
                        ];

                        // Reuse edge sampling logic
                        const numVerts = 4;
                        for (let i = 0; i < numVerts; i++) {
                            const p1 = rectVerts[i];
                            const p2 = rectVerts[(i + 1) % numVerts];

                            obstacleCloud.push([p1.x, p1.y, 2.0]);
                            obstacleCloud.push([p1.x, p1.y, 0.0]);
                            obstacleCloud.push([p1.x, p1.y, 4.0]);

                            const dx = p2.x - p1.x;
                            const dy = p2.y - p1.y;
                            const len = Math.sqrt(dx * dx + dy * dy);
                            const sampleStep = 0.5;
                            const steps = Math.floor(len / sampleStep);

                            for (let s = 1; s < steps; s++) {
                                const t = s / (steps + 1);
                                const px = p1.x + t * dx;
                                const py = p1.y + t * dy;
                                obstacleCloud.push([px, py, 2.0]);
                                obstacleCloud.push([px, py, 0.0]);
                                obstacleCloud.push([px, py, 4.0]);
                            }
                        }
                    } else if (obs.shape.type === 'circle' && obs.shape.radius) {
                        const outputPoints = 8;
                        for (let i = 0; i < outputPoints; i++) {
                            const theta = (i / outputPoints) * Math.PI * 2;
                            const ox = obs.position.x + Math.cos(theta) * obs.shape.radius;
                            const oy = obs.position.y + Math.sin(theta) * obs.shape.radius;
                            obstacleCloud.push([ox, oy, 2.0]);
                        }
                    }
                });

                // 3. Generate Safe Flight Corridor
                const bounds3D = createBoundingBox(
                    [worldBounds.minX, worldBounds.minY, 0],
                    [worldBounds.maxX, worldBounds.maxY, 10]
                );

                const corridors = generateSafeFlightCorridor(
                    waypoints3D,
                    obstacleCloud,
                    bounds3D,
                    { iterations: 2, epsilon: 0.1 }
                );

                // 4. Generate MINCO Trajectory (with optional L-BFGS time optimization)
                const totalDist = waypoints.reduce((acc, wp, i) => {
                    if (i === 0) return 0;
                    const dx = wp.x - waypoints[i - 1].x;
                    const dy = wp.y - waypoints[i - 1].y;
                    return acc + Math.sqrt(dx * dx + dy * dy);
                }, 0);

                const totalTimeHint = Math.max(1.0, totalDist / (state.config.maxVelocity * 0.9));

                let mincoTraj: PiecewiseTrajectory;
                let totalTime: number;

                if (state.config.enableTimeOptimization !== false && waypoints3D.length >= 2) {
                    // Use L-BFGS optimized time allocation
                    try {
                        const optResult = generateOptimizedTrajectory(
                            waypoints3D,
                            totalTimeHint,
                            {
                                initialVelocity: [robot.velocity.vx, robot.velocity.vy, 0],
                                initialAcceleration: [0, 0, 0],
                                finalVelocity: [0, 0, 0],
                                finalAcceleration: [0, 0, 0]
                            },
                            {
                                verbose: state.config.verboseOptimization ?? false,
                                maxIterations: 30,
                                robotId: robot.id,
                            }
                        );
                        mincoTraj = optResult.trajectory;
                        totalTime = optResult.optimizationResult.totalTime;

                        if (state.config.verboseOptimization) {
                            console.log(`[Planner] L-BFGS: ${optResult.optimizationResult.iterations} iters, ` +
                                `Cost ${optResult.optimizationResult.initialCost.toFixed(2)} -> ${optResult.optimizationResult.cost.toFixed(2)}`);
                        }
                    } catch (e) {
                        // Fallback to heuristic if optimization fails
                        console.warn('[Planner] L-BFGS failed, using heuristic:', e);
                        mincoTraj = generateMinimumSnapTrajectory(
                            waypoints3D,
                            totalTimeHint,
                            {
                                initialVelocity: [robot.velocity.vx, robot.velocity.vy, 0],
                                initialAcceleration: [0, 0, 0],
                                finalVelocity: [0, 0, 0],
                                finalAcceleration: [0, 0, 0]
                            }
                        );
                        totalTime = totalTimeHint;
                    }
                } else {
                    // Use heuristic time allocation (faster, less optimal)
                    mincoTraj = generateMinimumSnapTrajectory(
                        waypoints3D,
                        totalTimeHint,
                        {
                            initialVelocity: [robot.velocity.vx, robot.velocity.vy, 0],
                            initialAcceleration: [0, 0, 0],
                            finalVelocity: [0, 0, 0],
                            finalAcceleration: [0, 0, 0]
                        }
                    );
                    totalTime = totalTimeHint;
                }

                state.mincoTrajectory = mincoTraj;
                state.corridors = corridors;

                state.trajectory = {
                    segments: [],
                    totalDuration: totalTime,
                    waypoints
                };

                state.lastPlanTime = currentTime;
                state.startTime = currentTime;
                state.lastSafetyMargin = currentSafetyMargin;

                // Visualization: Sample the smooth trajectory
                const smoothPathPoints: Vector2[] = [];
                const sampleCount = 100;
                for (let k = 0; k <= sampleCount; k++) {
                    const t = (k / sampleCount) * totalTime;
                    const s = evaluateTrajectory(mincoTraj, t);
                    smoothPathPoints.push({ x: s.position[0], y: s.position[1] });
                }

                // Convert corridor to 2D visualization
                // Each corridor segment is visualized as a rotated rectangle around the path segment
                const corridorViz: { center: Vector2; vertices: Vector2[] }[] = [];
                const corridorWidth = totalInflation * 2 + 1.0; // Width based on safety margin

                for (let cIdx = 0; cIdx < waypoints.length - 1; cIdx++) {
                    const start = waypoints[cIdx];
                    const end = waypoints[cIdx + 1];

                    // Compute direction and perpendicular
                    const dx = end.x - start.x;
                    const dy = end.y - start.y;
                    const len = Math.sqrt(dx * dx + dy * dy);

                    if (len < 0.1) continue; // Skip very short segments

                    // Unit direction vector
                    const ux = dx / len;
                    const uy = dy / len;

                    // Perpendicular vector (rotated 90 degrees)
                    const px = -uy;
                    const py = ux;

                    // Half-widths
                    const halfLen = len / 2 + 0.5; // Extend slightly beyond segment
                    const halfWidth = corridorWidth / 2;

                    // Center of the corridor segment
                    const center: Vector2 = {
                        x: (start.x + end.x) / 2,
                        y: (start.y + end.y) / 2
                    };

                    // Create rotated rectangle vertices
                    const vertices: Vector2[] = [
                        { x: center.x - ux * halfLen - px * halfWidth, y: center.y - uy * halfLen - py * halfWidth },
                        { x: center.x + ux * halfLen - px * halfWidth, y: center.y + uy * halfLen - py * halfWidth },
                        { x: center.x + ux * halfLen + px * halfWidth, y: center.y + uy * halfLen + py * halfWidth },
                        { x: center.x - ux * halfLen + px * halfWidth, y: center.y - uy * halfLen + py * halfWidth }
                    ];

                    corridorViz.push({ center, vertices });
                }


                getPathVisualizationCache().set(robotId, {
                    aStarPath: waypoints,
                    optimizedPath: smoothPathPoints,
                    corridors: corridorViz,
                    timestamp: currentTime
                });

                // Record optimization history for visualization
                const planEndTime = Date.now();
                initOptimizationHistory(robotId);

                // Record planning summary
                recordPlanningSummary(
                    robotId,
                    planEndTime - currentTime,
                    0, // A* iterations not tracked here
                    totalDist,
                    waypoints.length
                );

                // Record MINCO data
                const durations: number[] = [];
                mincoTraj.pieces.forEach(p => durations.push(p.duration));
                recordMINCOData(
                    robotId,
                    mincoTraj.pieces.length,
                    durations,
                    totalTime,
                    { position: 0, velocity: 0, acceleration: 0, total: 0 } // Costs not computed in simple mode
                );

                // Record corridor generation (simplified)
                for (let cIdx = 0; cIdx < corridors.length; cIdx++) {
                    recordCorridorGeneration(
                        robotId,
                        cIdx,
                        4, // FIRI iterations (default)
                        0, // Ellipsoid volume (not computed)
                        corridors[cIdx].length // Number of half-space constraints
                    );
                }
            }
        } catch (error) {
            console.warn('[OptimalTrajectory] Planning failed:', error);
            return { vx: 0, vy: 0, omega: 0 };
        }
    }

    if (state.mincoTrajectory) {
        const elapsed = (currentTime - state.startTime) / 1000.0;

        let totalDuration = 0;
        state.mincoTrajectory.pieces.forEach(p => totalDuration += p.duration);

        // If trajectory finished, use simpleGoToGoal to continue navigation
        if (elapsed > totalDuration + 0.5) {
            // Check if we're still far from goal
            const distToGoal = Math.sqrt(
                (goal.x - robot.pose.x) ** 2 + (goal.y - robot.pose.y) ** 2
            );
            if (distToGoal > 0.5) {
                return simpleGoToGoal(robot, goal);
            }
            // Already at goal, stop
            return { vx: 0, vy: 0, omega: 0 };
        }

        // Evaluate MINCO trajectory
        const stateAtT = evaluateTrajectory(state.mincoTrajectory, elapsed);

        const posErrorX = stateAtT.position[0] - robot.pose.x;
        const posErrorY = stateAtT.position[1] - robot.pose.y;
        const posError = Math.sqrt(posErrorX * posErrorX + posErrorY * posErrorY);

        const kp = 2.5;
        const kv = 1.0;

        // World-frame velocity commands
        const cmdVxWorld = kv * stateAtT.velocity[0] + kp * posErrorX;
        const cmdVyWorld = kv * stateAtT.velocity[1] + kp * posErrorY;

        // Update visualization cache with current trajectory state for debugging
        const vizcache = getPathVisualizationCache();
        const existingViz = vizcache.get(robotId);
        if (existingViz) {
            existingViz.currentTrajectoryPoint = {
                x: stateAtT.position[0],
                y: stateAtT.position[1]
            };
            existingViz.elapsedTime = elapsed;
            existingViz.totalDuration = totalDuration;
            existingViz.debugInfo = {
                robotPosition: { x: robot.pose.x, y: robot.pose.y },
                trajPosition: { x: stateAtT.position[0], y: stateAtT.position[1] },
                positionError: posError,
                velocityCmd: { x: cmdVxWorld, y: cmdVyWorld }
            };
            vizcache.set(robotId, existingViz);
        }

        // Debug log (uncomment to diagnose coordinate issues)
        // if (Math.random() < 0.05) {
        //     console.log('[MINCO Debug] t=' + elapsed.toFixed(2) + 's/' + totalDuration.toFixed(2) + 's | ' +
        //         'Robot: (' + robot.pose.x.toFixed(2) + ', ' + robot.pose.y.toFixed(2) + ') | ' +
        //         'Traj: (' + stateAtT.position[0].toFixed(2) + ', ' + stateAtT.position[1].toFixed(2) + ') | ' +
        //         'Error: ' + posError.toFixed(2) + 'm | ' +
        //         'VelCmd: (' + cmdVxWorld.toFixed(2) + ', ' + cmdVyWorld.toFixed(2) + ')');
        // }

        const speedSq = cmdVxWorld * cmdVxWorld + cmdVyWorld * cmdVyWorld;
        let desiredHeading = robot.pose.theta;
        if (speedSq > 0.1) {
            desiredHeading = Math.atan2(cmdVyWorld, cmdVxWorld);
        }

        let headingError = desiredHeading - robot.pose.theta;
        while (headingError > Math.PI) headingError -= 2 * Math.PI;
        while (headingError < -Math.PI) headingError += 2 * Math.PI;

        // Convert world-frame velocity to body-frame for quadrotorUpdate
        // Body-frame: vx = forward, vy = lateral
        // World-to-body rotation: R^T = [cos, sin; -sin, cos]
        const cosTheta = Math.cos(robot.pose.theta);
        const sinTheta = Math.sin(robot.pose.theta);
        const cmdVxBody = cmdVxWorld * cosTheta + cmdVyWorld * sinTheta;
        const cmdVyBody = -cmdVxWorld * sinTheta + cmdVyWorld * cosTheta;

        return {
            vx: Math.max(-maxSpeed, Math.min(maxSpeed, cmdVxBody)),
            vy: Math.max(-maxSpeed, Math.min(maxSpeed, cmdVyBody)),
            omega: headingError * 3.5
        };
    }

    return simpleGoToGoal(robot, goal);
}

function simpleGoToGoal(robot: Robot, goal: Pose): Velocity {
    const dx = goal.x - robot.pose.x;
    const dy = goal.y - robot.pose.y;
    const dist = Math.sqrt(dx * dx + dy * dy);

    if (dist < 0.3) {
        return { vx: 0, vy: 0, omega: 0 };
    }

    const maxSpeed = robot.maxSpeed || 5.0;
    const speed = Math.min(maxSpeed, dist * 1.5);
    const vxWorld = (dx / dist) * speed;
    const vyWorld = (dy / dist) * speed;

    const desiredHeading = Math.atan2(dy, dx);
    let headingError = desiredHeading - robot.pose.theta;
    while (headingError > Math.PI) headingError -= 2 * Math.PI;
    while (headingError < -Math.PI) headingError += 2 * Math.PI;

    return {
        vx: Math.cos(robot.pose.theta) * vxWorld + Math.sin(robot.pose.theta) * vyWorld,
        vy: -Math.sin(robot.pose.theta) * vxWorld + Math.cos(robot.pose.theta) * vyWorld,
        omega: headingError * 2.0
    };
}

// ==================== Algorithms ====================

interface AStarNode {
    x: number;
    y: number;
    g: number;
    h: number;
    f: number;
    parent: AStarNode | null;
}

export function aStarPathPlanning(
    start: Vector2,
    goal: Vector2,
    obstacles: Obstacle[],
    bounds: { minX: number; maxX: number; minY: number; maxY: number },
    gridResolution: number = 1.0,
    safetyMargin: number = 0.5
): Vector2[] {
    const gridWidth = Math.ceil((bounds.maxX - bounds.minX) / gridResolution);
    const gridHeight = Math.ceil((bounds.maxY - bounds.minY) / gridResolution);

    // Debug: Log planning info
    console.log(`[A *] Grid: ${gridWidth}x${gridHeight}, Resolution: ${gridResolution}, Margin: ${safetyMargin} `);
    console.log(`[A *] Obstacles: ${obstacles.length} `, obstacles.map(o => o.shape.type));

    const obstacleGrid: boolean[][] = [];
    let blockedCount = 0;
    for (let i = 0; i < gridWidth; i++) {
        obstacleGrid[i] = [];
        for (let j = 0; j < gridHeight; j++) {
            const worldX = bounds.minX + i * gridResolution + gridResolution / 2;
            const worldY = bounds.minY + j * gridResolution + gridResolution / 2;
            obstacleGrid[i][j] = isPointInObstacle(worldX, worldY, obstacles, safetyMargin);
            if (obstacleGrid[i][j]) blockedCount++;
        }
    }
    console.log(`[A *] Blocked cells: ${blockedCount} / ${gridWidth * gridHeight} (${(blockedCount / (gridWidth * gridHeight) * 100).toFixed(1)
        }%)`);

    const worldToGrid = (p: Vector2): { gx: number; gy: number } => ({
        gx: Math.floor((p.x - bounds.minX) / gridResolution),
        gy: Math.floor((p.y - bounds.minY) / gridResolution)
    });

    const gridToWorld = (gx: number, gy: number): Vector2 => ({
        x: bounds.minX + gx * gridResolution + gridResolution / 2,
        y: bounds.minY + gy * gridResolution + gridResolution / 2
    });

    const heuristic = (a: { x: number; y: number }, b: { x: number; y: number }): number => {
        return Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2);
    };

    const nodeKey = (x: number, y: number): string => `${x},${y} `;

    const startGrid = worldToGrid(start);
    const goalGrid = worldToGrid(goal);

    if (startGrid.gx < 0 || startGrid.gx >= gridWidth ||
        startGrid.gy < 0 || startGrid.gy >= gridHeight ||
        goalGrid.gx < 0 || goalGrid.gx >= gridWidth ||
        goalGrid.gy < 0 || goalGrid.gy >= gridHeight) {
        return [start, goal];
    }

    const startNode: AStarNode = {
        x: startGrid.gx,
        y: startGrid.gy,
        g: 0,
        h: heuristic({ x: startGrid.gx, y: startGrid.gy }, { x: goalGrid.gx, y: goalGrid.gy }),
        f: 0,
        parent: null
    };
    startNode.f = startNode.g + startNode.h;

    const openSet: AStarNode[] = [startNode];
    const closedSet = new Set<string>();
    const openMap = new Map<string, AStarNode>();
    openMap.set(nodeKey(startNode.x, startNode.y), startNode);

    const directions = [
        { dx: 1, dy: 0, cost: 1 },
        { dx: -1, dy: 0, cost: 1 },
        { dx: 0, dy: 1, cost: 1 },
        { dx: 0, dy: -1, cost: 1 },
        { dx: 1, dy: 1, cost: Math.SQRT2 },
        { dx: 1, dy: -1, cost: Math.SQRT2 },
        { dx: -1, dy: 1, cost: Math.SQRT2 },
        { dx: -1, dy: -1, cost: Math.SQRT2 }
    ];

    let iterations = 0;
    const maxIterations = 100000; // Increased for large grids

    // Simple binary heap helpers for performance
    const heapPush = (heap: AStarNode[], node: AStarNode) => {
        heap.push(node);
        let i = heap.length - 1;
        while (i > 0) {
            const parent = Math.floor((i - 1) / 2);
            if (heap[parent].f <= heap[i].f) break;
            [heap[parent], heap[i]] = [heap[i], heap[parent]];
            i = parent;
        }
    };

    const heapPop = (heap: AStarNode[]): AStarNode | undefined => {
        if (heap.length === 0) return undefined;
        const top = heap[0];
        const last = heap.pop()!;
        if (heap.length > 0) {
            heap[0] = last;
            let i = 0;
            while (true) {
                const left = 2 * i + 1;
                const right = 2 * i + 2;
                let smallest = i;
                if (left < heap.length && heap[left].f < heap[smallest].f) smallest = left;
                if (right < heap.length && heap[right].f < heap[smallest].f) smallest = right;
                if (smallest === i) break;
                [heap[i], heap[smallest]] = [heap[smallest], heap[i]];
                i = smallest;
            }
        }
        return top;
    };

    // Weighted A* heuristic multiplier (1.5 makes it more greedy/faster)
    const heuristicWeight = 1.5;

    while (openSet.length > 0 && iterations < maxIterations) {
        iterations++;
        const current = heapPop(openSet)!;
        const currentKey = nodeKey(current.x, current.y);
        openMap.delete(currentKey);

        if (current.x === goalGrid.gx && current.y === goalGrid.gy) {
            console.log(`[A *] SUCCESS! Found path in ${iterations} iterations`);
            const path: Vector2[] = [];
            let node: AStarNode | null = current;
            while (node) {
                path.unshift(gridToWorld(node.x, node.y));
                node = node.parent;
            }
            return simplifyPath(path, obstacles, safetyMargin);
        }

        closedSet.add(currentKey);

        for (const dir of directions) {
            const nx = current.x + dir.dx;
            const ny = current.y + dir.dy;

            if (nx < 0 || nx >= gridWidth || ny < 0 || ny >= gridHeight) continue;
            if (closedSet.has(nodeKey(nx, ny))) continue;
            if (obstacleGrid[nx][ny]) continue;

            const tentativeG = current.g + dir.cost * gridResolution;
            const neighborKey = nodeKey(nx, ny);
            const existing = openMap.get(neighborKey);

            if (existing) {
                if (tentativeG < existing.g) {
                    existing.g = tentativeG;
                    existing.f = existing.g + existing.h;
                    existing.parent = current;
                }
            } else {
                const neighbor: AStarNode = {
                    x: nx,
                    y: ny,
                    g: tentativeG,
                    h: heuristic({ x: nx, y: ny }, { x: goalGrid.gx, y: goalGrid.gy }) * gridResolution * heuristicWeight,
                    f: 0,
                    parent: current
                };
                neighbor.f = neighbor.g + neighbor.h;
                heapPush(openSet, neighbor);
                openMap.set(neighborKey, neighbor);
            }
        }
    }

    console.log(`[A *] FAILED to find path after ${iterations} iterations(max: ${maxIterations})`);
    return [start, goal];
}

function simplifyPath(path: Vector2[], obstacles: Obstacle[], margin: number): Vector2[] {
    if (path.length <= 2) return path;
    const simplified: Vector2[] = [path[0]];
    let current = 0;
    while (current < path.length - 1) {
        let furthest = current + 1;
        for (let i = path.length - 1; i > current + 1; i--) {
            if (isLineOfSightClear(path[current], path[i], obstacles, margin)) {
                furthest = i;
                break;
            }
        }
        simplified.push(path[furthest]);
        current = furthest;
    }
    return simplified;
}

function isLineOfSightClear(start: Vector2, end: Vector2, obstacles: Obstacle[], margin: number): boolean {
    const dx = end.x - start.x;
    const dy = end.y - start.y;
    const len = Math.sqrt(dx * dx + dy * dy);
    const steps = Math.ceil(len / 0.5);
    for (let i = 0; i <= steps; i++) {
        const t = i / steps;
        const x = start.x + t * dx;
        const y = start.y + t * dy;
        if (isPointInObstacle(x, y, obstacles, margin)) return false;
    }
    return true;
}

function isPathClear(start: Vector2, end: Vector2, obstacles: Obstacle[], margin: number): boolean {
    return isLineOfSightClear(start, end, obstacles, margin);
}

function isPointInObstacle(x: number, y: number, obstacles: Obstacle[], margin: number): boolean {
    for (const obs of obstacles) {
        if (obs.shape.type === 'circle') {
            const dist = Math.sqrt((x - obs.position.x) ** 2 + (y - obs.position.y) ** 2);
            if (dist < (obs.shape.radius || 1) + margin) return true;
        } else if (obs.shape.type === 'polygon' && obs.shape.vertices) {
            const vertices = obs.shape.vertices;
            if (isPointInPolygon(x, y, vertices)) return true;
            if (distanceToPolygonEdges(x, y, vertices) < margin) return true;
        } else if (obs.shape.type === 'rectangle' && obs.shape.width && obs.shape.height) {
            const halfW = obs.shape.width / 2;
            const halfH = obs.shape.height / 2;
            // Check if point is inside rectangle or near edges
            // Assuming axis-aligned rectangle for simple visualization
            if (x > obs.position.x - halfW - margin && x < obs.position.x + halfW + margin &&
                y > obs.position.y - halfH - margin && y < obs.position.y + halfH + margin) {
                return true;
            }
        }
    }
    return false;
}

function isPointInPolygon(x: number, y: number, vertices: Vector2[]): boolean {
    let inside = false;
    const n = vertices.length;
    for (let i = 0, j = n - 1; i < n; j = i++) {
        const xi = vertices[i].x, yi = vertices[i].y;
        const xj = vertices[j].x, yj = vertices[j].y;
        if (((yi > y) !== (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

function distanceToPolygonEdges(x: number, y: number, vertices: Vector2[]): number {
    let minDist = Infinity;
    const n = vertices.length;
    for (let i = 0; i < n; i++) {
        const j = (i + 1) % n;
        const dist = distanceToSegment(x, y, vertices[i].x, vertices[i].y, vertices[j].x, vertices[j].y);
        if (dist < minDist) minDist = dist;
    }
    return minDist;
}

function distanceToSegment(px: number, py: number, x1: number, y1: number, x2: number, y2: number): number {
    const dx = x2 - x1;
    const dy = y2 - y1;
    const lenSq = dx * dx + dy * dy;
    if (lenSq === 0) return Math.sqrt((px - x1) ** 2 + (py - y1) ** 2);
    let t = ((px - x1) * dx + (py - y1) * dy) / lenSq;
    t = Math.max(0, Math.min(1, t));
    const projX = x1 + t * dx;
    const projY = y1 + t * dy;
    return Math.sqrt((px - projX) ** 2 + (py - projY) ** 2);
}

// ==================== 3D Full Physics Flight Control ====================

/**
 * Full 3D flight simulation with cascade PID controller and physics engine
 * 
 * This function provides multiple simulation modes:
 * - 2D Mode (use3DPhysics=false): Uses MINCO trajectory with proportional control
 * - 3D Mode (use3DPhysics=true): Full cascade PID + rigid body physics
 * 
 * @param robot - Current robot state
 * @param goal - Goal pose
 * @param obstacles - Obstacles for planning
 * @param neighbors - Neighboring robots
 * @param currentTime - Current timestamp in ms
 * @param simDt - Simulation timestep in seconds (external)
 * @param worldBounds - World boundaries
 * @param extraConfig - Additional planner config
 * @param controllerParams - Optional controller parameters (3D mode only)
 * @param quadParams - Optional quadrotor parameters (3D mode only)
 * @param use3DPhysics - Enable full 3D physics simulation (default: false for 2D mode)
 * @returns Updated 3D state and 2D velocity command for compatibility
 */
export function fullPhysicsTrajectoryBehavior(
    robot: Robot,
    goal: Pose,
    obstacles: Obstacle[],
    neighbors: Robot[],
    currentTime: number = Date.now(),
    simDt: number = 0.02, // External simulation dt (50Hz)
    worldBounds: { minX: number; maxX: number; minY: number; maxY: number } = { minX: 0, maxX: 100, minY: 0, maxY: 100 },
    extraConfig: Partial<PlannerConfig> = {},
    controllerParams?: Partial<ControllerParams>,
    quadParams?: Partial<QuadrotorParams>,
    use3DPhysics: boolean = false // Default to 2D mode for stability
): { velocity: Velocity; flightState: QuadrotorState3D } {
    const robotId = robot.id || 'robot';
    const maxSpeed = robot.maxSpeed || 5.0;
    const robotRadius = robot.shape?.radius || 0.5;

    let state = plannerCache.get(robotId);
    const currentSafetyMargin = extraConfig.safetyMargin ?? (robotRadius + 0.3);

    // Initialize state if not exists
    if (!state) {
        state = {
            trajectory: null,
            lastPlanTime: 0,
            startTime: 0,
            config: { ...DEFAULT_CONFIG, ...extraConfig },
            lastSafetyMargin: currentSafetyMargin,
            // Initialize 3D flight state from robot pose
            flightState: createHoverState([robot.pose.x, robot.pose.y, 2.0]),
            controller: use3DPhysics ? new CascadeController(
                { ...DEFAULT_CONTROLLER_PARAMS, ...controllerParams },
                { ...DEFAULT_QUADROTOR_PARAMS, ...quadParams }
            ) : undefined,
            quadParams: use3DPhysics ? { ...DEFAULT_QUADROTOR_PARAMS, ...quadParams } : undefined
        };
        plannerCache.set(robotId, state);
    }

    // Update config
    state.config.maxVelocity = maxSpeed;
    state.config.safetyMargin = currentSafetyMargin;
    if (extraConfig.gridResolution) {
        state.config.gridResolution = extraConfig.gridResolution;
    }

    // ===== 2D Mode: Use optimalTrajectoryBehavior directly =====
    if (!use3DPhysics) {
        // Get 2D velocity from the planning/control logic
        const velocity2D = optimalTrajectoryBehavior(
            robot, goal, obstacles, neighbors, currentTime,
            1000, worldBounds, extraConfig
        );

        // Update 3D flight state to track 2D position (for consistency)
        if (!state.flightState) {
            state.flightState = createHoverState([robot.pose.x, robot.pose.y, 2.0]);
        }
        // Sync position from robot's 2D pose
        state.flightState.position = [robot.pose.x, robot.pose.y, 2.0];
        state.flightState.velocity = [velocity2D.vx, velocity2D.vy, 0];

        // Update quaternion from yaw
        const halfYaw = robot.pose.theta / 2;
        state.flightState.quaternion = [Math.cos(halfYaw), 0, 0, Math.sin(halfYaw)];

        return {
            velocity: velocity2D,
            flightState: state.flightState
        };
    }

    // ===== 3D Mode: Full Physics Simulation =====

    // Ensure 3D components exist
    if (!state.flightState) {
        state.flightState = createHoverState([robot.pose.x, robot.pose.y, 2.0]);
    }
    if (!state.controller) {
        state.controller = new CascadeController(
            { ...DEFAULT_CONTROLLER_PARAMS, ...controllerParams },
            { ...DEFAULT_QUADROTOR_PARAMS, ...quadParams }
        );
    }
    if (!state.quadParams) {
        state.quadParams = { ...DEFAULT_QUADROTOR_PARAMS, ...quadParams };
    }

    // Save startTime before calling optimalTrajectoryBehavior to prevent reset
    const previousStartTime = state.startTime;
    const hadTrajectory = !!state.mincoTrajectory;

    // Call 2D planning logic - it will update state.mincoTrajectory if needed
    const velocity2D = optimalTrajectoryBehavior(
        robot, goal, obstacles, neighbors, currentTime,
        1000, worldBounds, extraConfig
    );

    // Restore startTime if trajectory wasn't actually replanned
    if (hadTrajectory && state.startTime !== previousStartTime && previousStartTime > 0) {
        // A replan happened - keep the new startTime
    } else if (hadTrajectory && previousStartTime > 0 && state.startTime === 0) {
        // Restore if incorrectly reset
        state.startTime = previousStartTime;
    }

    // 3D Control and Physics Update
    if (state.mincoTrajectory && state.controller && state.quadParams) {
        // Calculate elapsed time from trajectory start
        const elapsed = (currentTime - state.startTime) / 1000.0;

        // Get total duration
        let totalDuration = 0;
        state.mincoTrajectory.pieces.forEach(p => totalDuration += p.duration);

        // Clamp elapsed time
        const t = Math.min(Math.max(0, elapsed), totalDuration);

        // Evaluate MINCO trajectory at current time
        const refState = evaluateTrajectory(state.mincoTrajectory, t);

        // Build desired state for controller
        const desiredState: BaseTrajectoryState = {
            position: refState.position,
            velocity: refState.velocity,
            acceleration: refState.acceleration,
            jerk: [0, 0, 0],
        };

        // Compute desired yaw from velocity direction
        const vel3D = refState.velocity;
        let desiredYaw = robot.pose.theta;
        if (vel3D[0] * vel3D[0] + vel3D[1] * vel3D[1] > 0.01) {
            desiredYaw = Math.atan2(vel3D[1], vel3D[0]);
        }

        // Run physics at fixed timestep
        const numSubSteps = Math.max(1, Math.round(simDt / PHYSICS_DT));
        const actualPhysDt = simDt / numSubSteps;

        for (let i = 0; i < numSubSteps; i++) {
            const control = state.controller.update(
                state.flightState,
                desiredState,
                desiredYaw,
                0,
                actualPhysDt
            );

            state.flightState = quadrotorPhysicsUpdate(
                state.flightState,
                control,
                state.quadParams,
                actualPhysDt
            );
        }

        const newVelocity: Velocity = {
            vx: state.flightState.velocity[0],
            vy: state.flightState.velocity[1],
            omega: state.flightState.angularVelocity[2]
        };

        return {
            velocity: newVelocity,
            flightState: state.flightState
        };
    }

    // Fallback: use 2D velocity without physics
    return {
        velocity: velocity2D,
        flightState: state.flightState || createHoverState([robot.pose.x, robot.pose.y, 2.0])
    };
}

/**
 * Get the current 3D flight state for a robot
 */
export function getFlightState(robotId: string): QuadrotorState3D | null {
    const state = plannerCache.get(robotId);
    return state?.flightState || null;
}

/**
 * Reset the 3D flight state for a robot
 */
export function resetFlightState(robotId: string, position?: Vector3Array): void {
    const state = plannerCache.get(robotId);
    if (state) {
        state.flightState = createHoverState(position || [0, 0, 2.0]);
        state.controller?.reset();
    }
}

/**
 * Sync 2D robot pose to 3D flight state
 * Call this when robot pose is modified externally
 */
export function syncFlightStateFromPose(robotId: string, pose: Pose, velocity?: Velocity): void {
    const state = plannerCache.get(robotId);
    if (state?.flightState) {
        state.flightState.position = [pose.x, pose.y, state.flightState.position[2]];
        if (velocity) {
            state.flightState.velocity = [velocity.vx, velocity.vy, 0];
        }
        // Update quaternion from yaw
        const halfYaw = pose.theta / 2;
        state.flightState.quaternion = [Math.cos(halfYaw), 0, 0, Math.sin(halfYaw)];
    }
}
