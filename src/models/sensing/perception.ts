/**
 * @module sensing/perception
 * @description Core perception and sensor simulation algorithms
 */

import { Vector2, Pose, Obstacle, LiDARConfig, LiDARScan, Robot, FOVConfig } from './types';
import { gaussianRandom, normalizeAngle } from './utils';

// ==================== LiDAR Simulation ====================

/**
 * Ray-circle intersection
 * Returns distance to intersection or Infinity if no hit
 */
function rayCircleIntersection(
    rayOrigin: Vector2,
    rayDir: Vector2,
    circleCenter: Vector2,
    radius: number,
    maxRange: number
): number {
    const dx = rayOrigin.x - circleCenter.x;
    const dy = rayOrigin.y - circleCenter.y;

    const a = rayDir.x * rayDir.x + rayDir.y * rayDir.y;
    const b = 2 * (dx * rayDir.x + dy * rayDir.y);
    const c = dx * dx + dy * dy - radius * radius;

    const discriminant = b * b - 4 * a * c;

    if (discriminant < 0) return Infinity;

    const sqrtD = Math.sqrt(discriminant);
    const t1 = (-b - sqrtD) / (2 * a);
    const t2 = (-b + sqrtD) / (2 * a);

    // Return closest positive intersection within range
    if (t1 > 0 && t1 < maxRange) return t1;
    if (t2 > 0 && t2 < maxRange) return t2;
    return Infinity;
}

/**
 * Simulate 2D LiDAR sensor measurement
 * 
 * @param robotPose - Current pose of the robot
 * @param obstacles - List of obstacles in the environment
 * @param config - LiDAR configuration parameters
 * @returns LiDAR scan result containing angles and distances
 */
export function simulateLiDAR(
    robotPose: Pose,
    obstacles: Obstacle[],
    config: LiDARConfig
): LiDARScan {
    const numRays = Math.floor(config.fov / config.resolution);
    const startAngle = robotPose.theta - (config.fov * Math.PI / 180) / 2;

    const angles: number[] = [];
    const distances: number[] = [];
    const detected: boolean[] = [];

    for (let i = 0; i < numRays; i++) {
        const angle = startAngle + (i * config.resolution * Math.PI / 180);
        angles.push(angle);

        const rayDir: Vector2 = {
            x: Math.cos(angle),
            y: Math.sin(angle)
        };

        let minDist = config.maxRange;
        let hit = false;

        for (const obs of obstacles) {
            let dist: number | null = null;

            if (obs.shape.type === 'circle') {
                dist = rayCircleIntersection(
                    { x: robotPose.x, y: robotPose.y },
                    rayDir,
                    obs.position,
                    obs.shape.radius || 0.5,
                    config.maxRange
                );
            } else if (obs.shape.type === 'rectangle') {
                // Convert rectangle to polygon vertices
                const w = obs.shape.width || 1;
                const h = obs.shape.height || 1;
                const rotation = obs.rotation || 0;
                const center = obs.position;

                const corners = [
                    { x: -w / 2, y: -h / 2 },
                    { x: w / 2, y: -h / 2 },
                    { x: w / 2, y: h / 2 },
                    { x: -w / 2, y: h / 2 }
                ].map(p => {
                    const rx = p.x * Math.cos(rotation) - p.y * Math.sin(rotation);
                    const ry = p.x * Math.sin(rotation) + p.y * Math.cos(rotation);
                    return { x: rx + center.x, y: ry + center.y };
                });

                dist = rayPolygonIntersection({ x: robotPose.x, y: robotPose.y }, rayDir, corners, config.maxRange);

            } else if (obs.shape.type === 'polygon' && obs.shape.vertices) {
                // Transform relative vertices to world coordinates
                const worldVertices = obs.shape.vertices.map(v => ({
                    x: v.x + obs.position.x,
                    y: v.y + obs.position.y
                }));
                dist = rayPolygonIntersection({ x: robotPose.x, y: robotPose.y }, rayDir, worldVertices, config.maxRange);
            }

            if (dist !== null && dist < minDist) {
                minDist = dist;
                hit = true;
            }
        }

        // Add Gaussian noise if configured
        if (config.noise && config.noise > 0 && hit) {
            minDist += gaussianRandom() * config.noise;
            minDist = Math.max(0, minDist);
        }

        distances.push(minDist);
        detected.push(hit);
    }

    return { angles, distances, detected };
}

/**
 * Ray-Polygon Intersection Helper
 * Standard ray-segment intersection using parametric form
 * 
 * @param origin - Ray origin
 * @param direction - Ray direction unit vector
 * @param vertices - Polygon vertices in order
 * @param maxRange - Maximum detectable range
 * @returns Distance to intersection, or null if no hit
 */
export function rayPolygonIntersection(
    origin: Vector2,
    direction: Vector2,
    vertices: Vector2[],
    maxRange: number
): number | null {
    let closestDist = Infinity;
    let hit = false;

    const n = vertices.length;
    for (let i = 0; i < n; i++) {
        const p1 = vertices[i];
        const p2 = vertices[(i + 1) % n];

        // Segment vector: d = p2 - p1
        const dx = p2.x - p1.x;
        const dy = p2.y - p1.y;

        // Vector from segment start to ray origin: f = origin - p1
        const fx = origin.x - p1.x;
        const fy = origin.y - p1.y;

        // Determinant (cross product): det = dy * Dx - dx * Dy
        const det = dy * direction.x - dx * direction.y;

        // If det is near zero, ray and segment are parallel
        if (Math.abs(det) < 1e-9) continue;

        // t = distance along ray to intersection point
        const t = (dx * fy - dy * fx) / det;

        // s = parameter along segment (0 to 1 means the intersection occurs on the segment)
        const s = (fy * direction.x - fx * direction.y) / det;

        // Valid intersection: t >= 0 (forward along ray), 0 <= s <= 1 (on segment)
        if (t >= 0 && s >= 0 && s <= 1) {
            if (t < closestDist) {
                closestDist = t;
                hit = true;
            }
        }
    }

    return (hit && closestDist <= maxRange) ? closestDist : null;
}

// ==================== FOV Sensor Detection ====================

/**
 * Check if a point is within a robot's field of view (FOV)
 * 
 * @param robotPose - Observer pose
 * @param targetPos - Target position to check
 * @param fovAngle - Angular width of FOV (radians)
 * @param fovRange - Radial reach of FOV (meters)
 * @returns True if the target is within the FOV
 */
export function isInFOV(
    robotPose: Pose,
    targetPos: Vector2,
    fovAngle: number,
    fovRange: number
): boolean {
    const dx = targetPos.x - robotPose.x;
    const dy = targetPos.y - robotPose.y;
    const dist = Math.sqrt(dx * dx + dy * dy);

    // Range check
    if (dist > fovRange) return false;

    // Angle check
    const angleToTarget = Math.atan2(dy, dx);
    const angleDiff = normalizeAngle(angleToTarget - robotPose.theta);

    return Math.abs(angleDiff) <= fovAngle / 2;
}

/**
 * Detect all robots and obstacles within the current FOV
 * 
 * @param robot - The observing robot
 * @param allRobots - List of all robots in simulation
 * @param obstacles - List of obstacles in simulation
 * @param fovConfig - Configuration for the FOV sensor
 * @returns List of detected robot and obstacle IDs
 */
export function detectInFOV(
    robot: Robot,
    allRobots: Robot[],
    obstacles: Obstacle[],
    fovConfig: FOVConfig
): { detectedRobots: string[], detectedObstacles: string[] } {
    const detectedRobots: string[] = [];
    const detectedObstacles: string[] = [];

    if (!fovConfig.enabled) {
        return { detectedRobots, detectedObstacles };
    }

    // Check for other robots
    for (const other of allRobots) {
        if (other.id === robot.id) continue;
        if (isInFOV(robot.pose, { x: other.pose.x, y: other.pose.y }, fovConfig.angle, fovConfig.range)) {
            detectedRobots.push(other.id);
        }
    }

    // Check for obstacles
    for (const obs of obstacles) {
        if (isInFOV(robot.pose, obs.position, fovConfig.angle, fovConfig.range)) {
            detectedObstacles.push(obs.id);
        }
    }

    return { detectedRobots, detectedObstacles };
}

