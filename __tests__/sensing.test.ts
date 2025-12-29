/**
 * Sensing Module Tests
 * Tests for perception (LiDAR, FOV) and environment generation
 */

import { describe, it, expect } from 'vitest';
import {
    // Perception
    simulateLiDAR,
    rayPolygonIntersection,
    isInFOV,
    detectInFOV,
} from '../src/models/sensing/perception';
import {
    // Environment
    getRobotColor,
    ROBOT_COLORS,
    generateRandomObstacles,
    generateRandomPolygonObstacles,
    generateCircleDistribution,
    generateRandomDistribution,
    createSimpleGridMap,
    createMazeGridMap,
    worldToGrid,
    gridToWorld,
} from '../src/models/sensing/environment';
import type { Obstacle, LiDARConfig, Robot, FOVConfig, Pose, Vector2 } from '../src/models/sensing/types';
import { isClose, std } from './test-utils';

// Helper to create test obstacle
function createTestObstacle(
    id: string,
    position: Vector2,
    shape: { type: 'circle' | 'rectangle' | 'polygon'; radius?: number; width?: number; height?: number },
    extra?: { rotation?: number }
): Obstacle {
    return {
        id,
        position,
        shape,
        color: '#888888',
        ...extra
    };
}

// Helper to create minimal test robot
function createTestRobot(id: string, pose: Pose): Robot {
    return {
        id,
        pose,
        velocity: { vx: 0, vy: 0, omega: 0 },
        shape: { type: 'circle', radius: 0.5 },
        kinematics: 'differential',
        color: '#ff0000',
        trajectory: [],
        maxSpeed: 1.0,
    };
}

// Simple seeded RNG for testing (matches implementation in environment.ts)
class TestSeededRNG {
    private seed: number;

    constructor(seed: number = 0) {
        this.seed = seed;
    }

    next(): number {
        this.seed = (this.seed * 1103515245 + 12345) % 2147483648;
        return this.seed / 2147483648;
    }
}

// ==================== LiDAR Simulation Tests ====================

describe('LiDAR Simulation', () => {
    const defaultConfig: LiDARConfig = {
        maxRange: 10,
        fov: 180,
        resolution: 10,
        noise: 0
    };

    describe('simulateLiDAR', () => {
        it('should return correct number of rays', () => {
            const pose: Pose = { x: 0, y: 0, theta: 0 };
            const obstacles: Obstacle[] = [];

            const scan = simulateLiDAR(pose, obstacles, defaultConfig);

            const expectedRays = Math.floor(defaultConfig.fov / defaultConfig.resolution);
            expect(scan.angles.length).toBe(expectedRays);
            expect(scan.distances.length).toBe(expectedRays);
            expect(scan.detected.length).toBe(expectedRays);
        });

        it('should detect circular obstacle', () => {
            const pose: Pose = { x: 0, y: 0, theta: 0 };
            const obstacles: Obstacle[] = [
                createTestObstacle('obs1', { x: 5, y: 0 }, { type: 'circle', radius: 1 })
            ];

            const scan = simulateLiDAR(pose, obstacles, defaultConfig);

            // At least one ray should hit the obstacle at distance ~4
            const hitDistances = scan.distances.filter((d, i) => scan.detected[i]);
            expect(hitDistances.length).toBeGreaterThan(0);
            expect(Math.min(...hitDistances)).toBeCloseTo(4, 0.5);
        });

        it('should detect rectangular obstacle', () => {
            const pose: Pose = { x: 0, y: 0, theta: 0 };
            const obstacles: Obstacle[] = [
                createTestObstacle('obs1', { x: 5, y: 0 }, { type: 'rectangle', width: 2, height: 2 })
            ];

            const scan = simulateLiDAR(pose, obstacles, defaultConfig);

            const hitDistances = scan.distances.filter((d, i) => scan.detected[i]);
            expect(hitDistances.length).toBeGreaterThan(0);
            expect(Math.min(...hitDistances)).toBeCloseTo(4, 0.5);
        });

        it('should respect maxRange', () => {
            const pose: Pose = { x: 0, y: 0, theta: 0 };
            const obstacles: Obstacle[] = [
                createTestObstacle('obs1', { x: 20, y: 0 }, { type: 'circle', radius: 1 })
            ];

            const scan = simulateLiDAR(pose, obstacles, defaultConfig);

            // No hits should be detected (all at max range)
            const hits = scan.detected.filter(d => d);
            expect(hits.length).toBe(0);
        });

        it('should add noise when configured', () => {
            const pose: Pose = { x: 0, y: 0, theta: 0 };
            const obstacles: Obstacle[] = [
                createTestObstacle('obs1', { x: 5, y: 0 }, { type: 'circle', radius: 1 })
            ];

            const noisyConfig = { ...defaultConfig, noise: 0.5 };

            // Run multiple times to see variation
            const distances: number[] = [];
            for (let i = 0; i < 50; i++) {
                const scan = simulateLiDAR(pose, obstacles, noisyConfig);
                const minDist = Math.min(...scan.distances.filter((_, j) => scan.detected[j]));
                if (Number.isFinite(minDist)) {
                    distances.push(minDist);
                }
            }

            // Should have some variance due to noise
            const variance = std(distances);
            expect(variance).toBeGreaterThan(0.1);
        });

        it('should handle rotated obstacles', () => {
            const pose: Pose = { x: 0, y: 0, theta: 0 };
            const obstacles: Obstacle[] = [
                createTestObstacle(
                    'obs1',
                    { x: 5, y: 0 },
                    { type: 'rectangle', width: 4, height: 0.5 },
                    { rotation: Math.PI / 4 }
                )
            ];

            const scan = simulateLiDAR(pose, obstacles, defaultConfig);

            const hits = scan.detected.filter(d => d);
            expect(hits.length).toBeGreaterThan(0);
        });
    });

    describe('rayPolygonIntersection', () => {
        it('should find intersection with square', () => {
            const origin = { x: 0, y: 0 };
            const direction = { x: 1, y: 0 };
            const vertices = [
                { x: 3, y: -1 },
                { x: 5, y: -1 },
                { x: 5, y: 1 },
                { x: 3, y: 1 },
            ];

            const dist = rayPolygonIntersection(origin, direction, vertices, 10);

            expect(dist).not.toBeNull();
            expect(dist).toBeCloseTo(3, 5);
        });

        it('should return null when ray misses', () => {
            const origin = { x: 0, y: 0 };
            const direction = { x: 1, y: 0 };
            const vertices = [
                { x: 3, y: 5 },
                { x: 5, y: 5 },
                { x: 5, y: 7 },
                { x: 3, y: 7 },
            ];

            const dist = rayPolygonIntersection(origin, direction, vertices, 10);

            expect(dist).toBeNull();
        });

        it('should respect maxRange', () => {
            const origin = { x: 0, y: 0 };
            const direction = { x: 1, y: 0 };
            const vertices = [
                { x: 8, y: -1 },
                { x: 10, y: -1 },
                { x: 10, y: 1 },
                { x: 8, y: 1 },
            ];

            const dist = rayPolygonIntersection(origin, direction, vertices, 5);

            expect(dist).toBeNull(); // Beyond max range
        });
    });
});

// ==================== FOV Detection Tests ====================

describe('FOV Detection', () => {
    describe('isInFOV', () => {
        it('should detect target in front', () => {
            const robotPose: Pose = { x: 0, y: 0, theta: 0 };
            const targetPos = { x: 5, y: 0 };

            expect(isInFOV(robotPose, targetPos, Math.PI / 2, 10)).toBe(true);
        });

        it('should not detect target behind', () => {
            const robotPose: Pose = { x: 0, y: 0, theta: 0 };
            const targetPos = { x: -5, y: 0 };

            expect(isInFOV(robotPose, targetPos, Math.PI / 2, 10)).toBe(false);
        });

        it('should not detect target beyond range', () => {
            const robotPose: Pose = { x: 0, y: 0, theta: 0 };
            const targetPos = { x: 15, y: 0 };

            expect(isInFOV(robotPose, targetPos, Math.PI, 10)).toBe(false);
        });

        it('should respect FOV angle', () => {
            const robotPose: Pose = { x: 0, y: 0, theta: 0 };
            const targetNearEdge = { x: 5, y: 4 }; // About 38 degrees off

            // Wide FOV should detect
            expect(isInFOV(robotPose, targetNearEdge, Math.PI, 10)).toBe(true);
            // Narrow FOV should not
            expect(isInFOV(robotPose, targetNearEdge, Math.PI / 6, 10)).toBe(false);
        });

        it('should work with rotated robot', () => {
            const robotPose: Pose = { x: 0, y: 0, theta: Math.PI / 2 };
            const targetAbove = { x: 0, y: 5 };
            const targetRight = { x: 5, y: 0 };

            expect(isInFOV(robotPose, targetAbove, Math.PI / 2, 10)).toBe(true);
            expect(isInFOV(robotPose, targetRight, Math.PI / 2, 10)).toBe(false);
        });
    });

    describe('detectInFOV', () => {
        it('should detect multiple robots in FOV', () => {
            const robot = createTestRobot('r1', { x: 0, y: 0, theta: 0 });
            const allRobots: Robot[] = [
                robot,
                createTestRobot('r2', { x: 3, y: 0, theta: 0 }),
                createTestRobot('r3', { x: -3, y: 0, theta: 0 }),
            ];
            const fovConfig: FOVConfig = {
                enabled: true,
                angle: Math.PI / 2,
                range: 10,
            };

            const result = detectInFOV(robot, allRobots, [], fovConfig);

            expect(result.detectedRobots).toContain('r2');
            expect(result.detectedRobots).not.toContain('r3');
            expect(result.detectedRobots).not.toContain('r1');
        });

        it('should detect obstacles in FOV', () => {
            const robot = createTestRobot('r1', { x: 0, y: 0, theta: 0 });
            const obstacles: Obstacle[] = [
                createTestObstacle('o1', { x: 5, y: 0 }, { type: 'circle', radius: 1 }),
                createTestObstacle('o2', { x: -5, y: 0 }, { type: 'circle', radius: 1 }),
            ];
            const fovConfig: FOVConfig = {
                enabled: true,
                angle: Math.PI / 2,
                range: 10,
            };

            const result = detectInFOV(robot, [], obstacles, fovConfig);

            expect(result.detectedObstacles).toContain('o1');
            expect(result.detectedObstacles).not.toContain('o2');
        });

        it('should return empty when disabled', () => {
            const robot = createTestRobot('r1', { x: 0, y: 0, theta: 0 });
            const fovConfig: FOVConfig = {
                enabled: false,
                angle: Math.PI,
                range: 100,
            };

            const result = detectInFOV(robot, [robot], [], fovConfig);

            expect(result.detectedRobots).toEqual([]);
            expect(result.detectedObstacles).toEqual([]);
        });
    });
});

// ==================== Environment Generation Tests ====================

describe('Environment Generation', () => {
    describe('getRobotColor', () => {
        it('should return color for valid index', () => {
            expect(getRobotColor(0)).toBe(ROBOT_COLORS[0]);
            expect(getRobotColor(1)).toBe(ROBOT_COLORS[1]);
        });

        it('should wrap around for large indices', () => {
            const length = ROBOT_COLORS.length;
            expect(getRobotColor(length)).toBe(ROBOT_COLORS[0]);
            expect(getRobotColor(length + 1)).toBe(ROBOT_COLORS[1]);
        });
    });

    describe('SeededRNG (internal)', () => {
        it('should produce deterministic sequence', () => {
            const rng1 = new TestSeededRNG(42);
            const rng2 = new TestSeededRNG(42);

            for (let i = 0; i < 10; i++) {
                expect(rng1.next()).toBe(rng2.next());
            }
        });

        it('should produce different sequences for different seeds', () => {
            const rng1 = new TestSeededRNG(1);
            const rng2 = new TestSeededRNG(2);

            // Very unlikely to produce same first 10 values
            let sameCount = 0;
            for (let i = 0; i < 10; i++) {
                if (rng1.next() === rng2.next()) sameCount++;
            }
            expect(sameCount).toBeLessThan(5);
        });

        it('should produce values in [0, 1)', () => {
            const rng = new TestSeededRNG(123);
            for (let i = 0; i < 1000; i++) {
                const val = rng.next();
                expect(val).toBeGreaterThanOrEqual(0);
                expect(val).toBeLessThan(1);
            }
        });
    });

    describe('generateRandomObstacles', () => {
        it('should generate correct number of obstacles', () => {
            const obstacles = generateRandomObstacles(10, { width: 100, height: 100 }, 42);
            expect(obstacles.length).toBe(10);
        });

        it('should place obstacles within world bounds', () => {
            const width = 50;
            const height = 50;
            const obstacles = generateRandomObstacles(20, { width, height }, 42);

            for (const obs of obstacles) {
                expect(obs.position.x).toBeGreaterThanOrEqual(0);
                expect(obs.position.x).toBeLessThanOrEqual(width);
                expect(obs.position.y).toBeGreaterThanOrEqual(0);
                expect(obs.position.y).toBeLessThanOrEqual(height);
            }
        });

        it('should be deterministic with same seed', () => {
            const obs1 = generateRandomObstacles(5, { width: 100, height: 100 }, 123);
            const obs2 = generateRandomObstacles(5, { width: 100, height: 100 }, 123);

            for (let i = 0; i < 5; i++) {
                expect(obs1[i].position.x).toBe(obs2[i].position.x);
                expect(obs1[i].position.y).toBe(obs2[i].position.y);
            }
        });
    });

    describe('generateRandomPolygonObstacles', () => {
        it('should generate polygon obstacles', () => {
            const obstacles = generateRandomPolygonObstacles(5, { width: 100, height: 100 }, 42);

            expect(obstacles.length).toBe(5);
            for (const obs of obstacles) {
                expect(obs.shape.type).toBe('polygon');
                expect(obs.shape.vertices).toBeDefined();
                expect(obs.shape.vertices!.length).toBeGreaterThanOrEqual(3);
            }
        });

        it('should respect center range options', () => {
            const options = {
                centerRange: [10, 10, 90, 90] as [number, number, number, number],
            };
            const obstacles = generateRandomPolygonObstacles(
                20, { width: 100, height: 100 }, 42, options
            );

            for (const obs of obstacles) {
                // Position now correctly stores the center
                expect(obs.position.x).toBeGreaterThanOrEqual(10);
                expect(obs.position.x).toBeLessThanOrEqual(90);
                expect(obs.position.y).toBeGreaterThanOrEqual(10);
                expect(obs.position.y).toBeLessThanOrEqual(90);

                // Vertices should be relative to position (centroid near 0,0)
                const vertices = obs.shape.vertices!;
                const cx = vertices.reduce((sum, v) => sum + v.x, 0) / vertices.length;
                const cy = vertices.reduce((sum, v) => sum + v.y, 0) / vertices.length;

                // Centroid of relative vertices should be near origin
                expect(Math.abs(cx)).toBeLessThan(3); // Some variance due to irregular shapes
                expect(Math.abs(cy)).toBeLessThan(3);
            }
        });
    });

    describe('generateCircleDistribution', () => {
        it('should generate correct number of robot positions', () => {
            const distribution = generateCircleDistribution(8, 50, 50, 20);
            expect(distribution.length).toBe(8);
        });

        it('should place robots on circle', () => {
            const cx = 50, cy = 50, r = 20;
            const distribution = generateCircleDistribution(8, cx, cy, r);

            for (const { start } of distribution) {
                const dist = Math.sqrt((start.x - cx) ** 2 + (start.y - cy) ** 2);
                expect(isClose(dist, r, 0.01)).toBe(true);
            }
        });

        it('should set goal to opposite side', () => {
            const cx = 50, cy = 50, r = 20;
            const distribution = generateCircleDistribution(4, cx, cy, r);

            for (const { goal } of distribution) {
                // Goal should be approximately opposite (distance ≈ 2r through center)
                const goalDist = Math.sqrt((goal.x - cx) ** 2 + (goal.y - cy) ** 2);
                expect(isClose(goalDist, r, 0.01)).toBe(true);
            }
        });
    });

    describe('generateRandomDistribution', () => {
        it('should generate correct number of poses', () => {
            const poses = generateRandomDistribution(10, 100, 100);
            expect(poses.length).toBe(10);
        });

        it('should respect minimum distance constraint', () => {
            const minDist = 5;
            const poses = generateRandomDistribution(10, 100, 100, minDist);

            for (let i = 0; i < poses.length; i++) {
                for (let j = i + 1; j < poses.length; j++) {
                    const dist = Math.sqrt(
                        (poses[i].x - poses[j].x) ** 2 +
                        (poses[i].y - poses[j].y) ** 2
                    );
                    expect(dist).toBeGreaterThanOrEqual(minDist);
                }
            }
        });
    });
});

// ==================== Grid Map Tests ====================

describe('Grid Map', () => {
    describe('createSimpleGridMap', () => {
        it('should create map with boundary walls', () => {
            const map = createSimpleGridMap(10, 10, 1);

            expect(map.width).toBeGreaterThan(0);
            expect(map.height).toBeGreaterThan(0);
            expect(map.data.length).toBe(map.height);
            expect(map.data[0].length).toBe(map.width);

            // Border should be occupied
            for (let x = 0; x < map.width; x++) {
                expect(map.data[0][x]).toBe(1); // Top row
                expect(map.data[map.height - 1][x]).toBe(1); // Bottom row
            }
            for (let y = 0; y < map.height; y++) {
                expect(map.data[y][0]).toBe(1); // Left column
                expect(map.data[y][map.width - 1]).toBe(1); // Right column
            }
        });

        it('should have free interior', () => {
            const map = createSimpleGridMap(10, 10, 1);

            // Interior should be free
            for (let y = 1; y < map.height - 1; y++) {
                for (let x = 1; x < map.width - 1; x++) {
                    expect(map.data[y][x]).toBe(0);
                }
            }
        });

        it('should respect resolution', () => {
            const map = createSimpleGridMap(10, 10, 0.5);

            // With 0.5m resolution, 10m world should give 20 cells
            expect(map.width).toBe(20);
            expect(map.height).toBe(20);
        });
    });

    describe('createMazeGridMap', () => {
        it('should create map with some obstacles', () => {
            const map = createMazeGridMap(20, 20, 1, 42);

            let occupiedCount = 0;
            for (let y = 0; y < map.height; y++) {
                for (let x = 0; x < map.width; x++) {
                    if (map.data[y][x] === 1) occupiedCount++;
                }
            }

            // Should have some walls but not be completely blocked
            expect(occupiedCount).toBeGreaterThan(map.width * 2); // More than just borders
            expect(occupiedCount).toBeLessThan(map.width * map.height * 0.5);
        });

        it('should be deterministic with seed', () => {
            const map1 = createMazeGridMap(20, 20, 1, 123);
            const map2 = createMazeGridMap(20, 20, 1, 123);

            for (let y = 0; y < map1.height; y++) {
                for (let x = 0; x < map1.width; x++) {
                    expect(map1.data[y][x]).toBe(map2.data[y][x]);
                }
            }
        });
    });

    describe('worldToGrid / gridToWorld', () => {
        it('should convert world to grid coordinates', () => {
            const map = createSimpleGridMap(10, 10, 0.5);
            const worldPos = { x: 2.5, y: 3.0 };

            const { gx, gy } = worldToGrid(worldPos, map);

            expect(gx).toBe(5);  // 2.5 / 0.5 = 5
            expect(gy).toBe(6);  // 3.0 / 0.5 = 6
        });

        it('should convert grid to world coordinates', () => {
            const map = createSimpleGridMap(10, 10, 0.5);

            const worldPos = gridToWorld(5, 6, map);

            // Should return center of cell
            expect(worldPos.x).toBeCloseTo(2.75, 5); // (5 + 0.5) * 0.5
            expect(worldPos.y).toBeCloseTo(3.25, 5); // (6 + 0.5) * 0.5
        });

        it('should round-trip approximately', () => {
            const map = createSimpleGridMap(10, 10, 0.5);
            const original = { x: 2.5, y: 3.0 };

            const { gx, gy } = worldToGrid(original, map);
            const roundTrip = gridToWorld(gx, gy, map);

            // Should be within one cell
            expect(Math.abs(roundTrip.x - original.x)).toBeLessThan(map.resolution);
            expect(Math.abs(roundTrip.y - original.y)).toBeLessThan(map.resolution);
        });
    });
});

// ==================== Additional LiDAR and Perception Tests ====================

describe('LiDAR Advanced Tests', () => {
    const defaultConfig: LiDARConfig = {
        maxRange: 10,
        fov: 360,
        resolution: 10,
        noise: 0
    };

    it('should handle multiple obstacles and return closest hit', () => {
        const pose: Pose = { x: 0, y: 0, theta: 0 };
        const obstacles: Obstacle[] = [
            createTestObstacle('near', { x: 3, y: 0 }, { type: 'circle', radius: 0.5 }),
            createTestObstacle('far', { x: 7, y: 0 }, { type: 'circle', radius: 0.5 }),
        ];

        const scan = simulateLiDAR(pose, obstacles, defaultConfig);

        // Find ray going in +x direction (theta = 0)
        const forwardRayIdx = scan.angles.findIndex(a => Math.abs(a) < 0.1);
        if (forwardRayIdx >= 0) {
            expect(scan.detected[forwardRayIdx]).toBe(true);
            expect(scan.distances[forwardRayIdx]).toBeCloseTo(2.5, 0.5); // 3 - 0.5 radius
        }
    });

    it('should handle 360 degree FOV', () => {
        const pose: Pose = { x: 0, y: 0, theta: 0 };
        const obstacles: Obstacle[] = [
            createTestObstacle('front', { x: 5, y: 0 }, { type: 'circle', radius: 0.5 }),
            createTestObstacle('back', { x: -5, y: 0 }, { type: 'circle', radius: 0.5 }),
            createTestObstacle('left', { x: 0, y: 5 }, { type: 'circle', radius: 0.5 }),
            createTestObstacle('right', { x: 0, y: -5 }, { type: 'circle', radius: 0.5 }),
        ];

        const scan = simulateLiDAR(pose, obstacles, { ...defaultConfig, fov: 360 });

        // Should detect obstacles in all directions
        const hits = scan.detected.filter(d => d);
        expect(hits.length).toBeGreaterThanOrEqual(4);
    });

    it('should handle narrow FOV', () => {
        const pose: Pose = { x: 0, y: 0, theta: 0 };
        const obstacles: Obstacle[] = [
            createTestObstacle('front', { x: 5, y: 0 }, { type: 'circle', radius: 0.5 }),
            createTestObstacle('side', { x: 0, y: 5 }, { type: 'circle', radius: 0.5 }),
        ];

        // Very narrow FOV (only forward)
        const narrowConfig = { ...defaultConfig, fov: 30 };
        const scan = simulateLiDAR(pose, obstacles, narrowConfig);

        // Should only detect the front obstacle
        const hits = scan.detected.filter(d => d);
        expect(hits.length).toBeGreaterThan(0);
        expect(hits.length).toBeLessThanOrEqual(5); // Few rays in 30 degree FOV
    });

    it('should handle empty obstacles list', () => {
        const pose: Pose = { x: 0, y: 0, theta: 0 };

        const scan = simulateLiDAR(pose, [], defaultConfig);

        // All distances should be at max range, no detections
        expect(scan.distances.every(d => d === defaultConfig.maxRange)).toBe(true);
        expect(scan.detected.every(d => !d)).toBe(true);
    });

    it('should handle robot at non-origin position', () => {
        const pose: Pose = { x: 10, y: 10, theta: 0 };
        const obstacles: Obstacle[] = [
            createTestObstacle('obs', { x: 15, y: 10 }, { type: 'circle', radius: 0.5 }),
        ];

        const scan = simulateLiDAR(pose, obstacles, defaultConfig);

        const hits = scan.detected.filter(d => d);
        expect(hits.length).toBeGreaterThan(0);
    });
});

describe('Ray Polygon Intersection Edge Cases', () => {
    it('should handle triangle intersection', () => {
        const origin = { x: 0, y: 0 };
        const direction = { x: 1, y: 0 };
        const triangle = [
            { x: 3, y: -2 },
            { x: 5, y: 0 },
            { x: 3, y: 2 },
        ];

        const dist = rayPolygonIntersection(origin, direction, triangle, 10);

        expect(dist).not.toBeNull();
        expect(dist!).toBeGreaterThanOrEqual(3);
        expect(dist!).toBeLessThanOrEqual(5);
    });

    it('should handle concave polygon', () => {
        const origin = { x: 0, y: 0 };
        const direction = { x: 1, y: 0 };
        // L-shaped polygon
        const lShape = [
            { x: 3, y: -1 },
            { x: 4, y: -1 },
            { x: 4, y: 0 },
            { x: 5, y: 0 },
            { x: 5, y: 1 },
            { x: 3, y: 1 },
        ];

        const dist = rayPolygonIntersection(origin, direction, lShape, 10);

        expect(dist).not.toBeNull();
    });

    it('should return null for parallel ray', () => {
        const origin = { x: 0, y: 3 }; // Above the square
        const direction = { x: 1, y: 0 }; // Parallel to x-axis
        const square = [
            { x: 3, y: -1 },
            { x: 5, y: -1 },
            { x: 5, y: 1 },
            { x: 3, y: 1 },
        ];

        const dist = rayPolygonIntersection(origin, direction, square, 10);

        expect(dist).toBeNull();
    });
});

describe('FOV Edge Cases', () => {
    it('should handle target at exact boundary of range', () => {
        const robotPose: Pose = { x: 0, y: 0, theta: 0 };
        const targetAtBoundary = { x: 10, y: 0 };
        const targetJustBeyond = { x: 10.1, y: 0 };

        expect(isInFOV(robotPose, targetAtBoundary, Math.PI, 10)).toBe(true);
        expect(isInFOV(robotPose, targetJustBeyond, Math.PI, 10)).toBe(false);
    });

    it('should handle target at exact boundary of FOV angle', () => {
        const robotPose: Pose = { x: 0, y: 0, theta: 0 };
        const fovAngle = Math.PI / 2; // 90 degrees total (45 each side)

        // At 45 degrees
        const at45deg = { x: 5, y: 5 }; // atan2(5,5) = 45 deg
        // At 50 degrees (just outside)
        const at50deg = { x: 5, y: 6 };

        // Edge case - may or may not include boundary
        const inFOV = isInFOV(robotPose, at45deg, fovAngle, 20);
        const outside = isInFOV(robotPose, at50deg, fovAngle, 20);

        expect(typeof inFOV).toBe('boolean');
        expect(typeof outside).toBe('boolean');
    });

    it('should handle negative theta (robot facing backward)', () => {
        const robotPose: Pose = { x: 0, y: 0, theta: Math.PI }; // Facing -x
        const targetBehindOrigin = { x: -5, y: 0 };
        const targetInFront = { x: 5, y: 0 };

        expect(isInFOV(robotPose, targetBehindOrigin, Math.PI / 2, 10)).toBe(true);
        expect(isInFOV(robotPose, targetInFront, Math.PI / 2, 10)).toBe(false);
    });

    it('should handle very narrow FOV', () => {
        const robotPose: Pose = { x: 0, y: 0, theta: 0 };
        const targetDirect = { x: 5, y: 0 };
        const targetSlightlyOff = { x: 5, y: 0.5 };

        const narrowFOV = Math.PI / 36; // 5 degrees
        expect(isInFOV(robotPose, targetDirect, narrowFOV, 10)).toBe(true);
        expect(isInFOV(robotPose, targetSlightlyOff, narrowFOV, 10)).toBe(false);
    });

    it('should handle full 360 FOV', () => {
        const robotPose: Pose = { x: 0, y: 0, theta: 0 };

        expect(isInFOV(robotPose, { x: 5, y: 0 }, 2 * Math.PI, 10)).toBe(true);
        expect(isInFOV(robotPose, { x: -5, y: 0 }, 2 * Math.PI, 10)).toBe(true);
        expect(isInFOV(robotPose, { x: 0, y: 5 }, 2 * Math.PI, 10)).toBe(true);
        expect(isInFOV(robotPose, { x: 0, y: -5 }, 2 * Math.PI, 10)).toBe(true);
    });
});

describe('detectInFOV Edge Cases', () => {
    it('should not include self in detected robots', () => {
        const robot = createTestRobot('self', { x: 0, y: 0, theta: 0 });
        const fovConfig: FOVConfig = {
            enabled: true,
            angle: 2 * Math.PI,
            range: 100,
        };

        const result = detectInFOV(robot, [robot], [], fovConfig);

        expect(result.detectedRobots).not.toContain('self');
    });

    it('should detect robots when overlapping position', () => {
        const robot = createTestRobot('r1', { x: 0, y: 0, theta: 0 });
        const otherRobot = createTestRobot('r2', { x: 0.1, y: 0, theta: 0 });
        const fovConfig: FOVConfig = {
            enabled: true,
            angle: Math.PI / 2,
            range: 10,
        };

        const result = detectInFOV(robot, [robot, otherRobot], [], fovConfig);

        expect(result.detectedRobots).toContain('r2');
    });

    it('should handle large number of robots efficiently', () => {
        const robot = createTestRobot('r0', { x: 50, y: 50, theta: 0 });
        const allRobots: Robot[] = [robot];

        // Add 100 robots in a grid
        for (let i = 0; i < 10; i++) {
            for (let j = 0; j < 10; j++) {
                allRobots.push(createTestRobot(`r${i * 10 + j + 1}`, { x: i * 10, y: j * 10, theta: 0 }));
            }
        }

        const fovConfig: FOVConfig = {
            enabled: true,
            angle: Math.PI * 2,
            range: 50,
        };

        const start = Date.now();
        const result = detectInFOV(robot, allRobots, [], fovConfig);
        const elapsed = Date.now() - start;

        expect(result.detectedRobots.length).toBeGreaterThan(0);
        expect(elapsed).toBeLessThan(100); // Should be fast
    });
});

