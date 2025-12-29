/**
 * Planning Module Tests
 * Comprehensive tests for planning algorithms:
 * - Polynomial root finding
 * - Trajectory tracking and planning
 * - Visibility-constrained navigation
 * - Full physics integration
 */

import { describe, test, it, expect, beforeEach } from 'vitest';

// ==================== Root Finder Imports ====================
import {
    polyEval,
    polyDerivative,
    polyConv,
    polySqr,
    countRoots,
    solveCubic,
    solveQuartic,
    safeNewton,
    isolateRealRoots,
    findPolyMaxMagnitude,
    polyExceedsThreshold,
} from '../src/models/planning/planning/root-finder';

// ==================== Tracking Planning Imports ====================
import {
    // Occupancy Grid
    createOccupancyGrid,
    setOccupancy,
    positionToIndex,
    indexToPosition,
    isOccupied,
    isValidIndex,
    DEFAULT_TRACKING_CONFIG,
    DEFAULT_LANDING_CONFIG,

    // Target Prediction
    predictTargetMotion,
    predictLinearMotion,
    predictDeceleratingMotion,

    // Visibility Path
    findVisibilityConstrainedPath,
    findShortestPath,
    checkRayValid,
    smoothPath,

    // Visibility Region
    computeVisibilityRegions,
    computeVisibilityPair,
    isInVisibilityRegion,
    generateVisibilityArc,

    // MINCO Constraints
    computeTrackingCost,
    computeVisibilityCost,
    computeVelocityCost,
    computeAccelerationCost,

    // Tracking Planner
    AerialTrackingPlanner,
    createTrackingPlanner,
    planTrackingTrajectory,

    // Types
    type OccupancyGrid,
    type VisibilityRegion,
} from '../src/models/planning/planning';

// ==================== Physics Integration Imports ====================
import {
    fullPhysicsTrajectoryBehavior,
    getFlightState,
    resetFlightState
} from '../src/models/planning/planning/optimal-trajectory';
import { quadrotorUpdate } from '../src/models/robotics/dynamics/kinematics';
import type { Vector3Array } from '../src/models/planning/trajectory/types';
import type { Robot, Pose, Obstacle } from '../src/models/robotics/drone/types';

// ==================== Test Utilities ====================
import { isClose, arraysClose } from './test-utils';

/**
 * Create a test occupancy grid with optional obstacles
 */
function createTestGrid(
    size: number = 50,
    resolution: number = 0.2,
    obstacles: [number, number, number, number, number, number][] = []
): OccupancyGrid {
    const extent = size * resolution;
    const grid = createOccupancyGrid(
        [size, size, size],
        resolution,
        [-extent / 2, -extent / 2, 0]
    );

    for (const [xMin, xMax, yMin, yMax, zMin, zMax] of obstacles) {
        for (let x = xMin; x <= xMax; x += resolution) {
            for (let y = yMin; y <= yMax; y += resolution) {
                for (let z = zMin; z <= zMax; z += resolution) {
                    const idx = positionToIndex([x, y, z], grid);
                    if (isValidIndex(idx, grid)) {
                        setOccupancy(idx, 1, grid);
                    }
                }
            }
        }
    }

    return grid;
}

function vectorsApproxEqual(a: Vector3Array, b: Vector3Array, tol: number = 0.1): boolean {
    return Math.abs(a[0] - b[0]) < tol &&
        Math.abs(a[1] - b[1]) < tol &&
        Math.abs(a[2] - b[2]) < tol;
}

function createTestRobot(id: string, x: number, y: number, theta: number = 0): Robot {
    return {
        id,
        pose: { x, y, theta },
        velocity: { vx: 0, vy: 0, omega: 0 },
        shape: { type: 'circle', radius: 0.5 },
        kinematics: 'quadrotor',
        color: '#00ff00',
        trajectory: [],
        maxSpeed: 5.0
    };
}

function distance(a: { x: number; y: number }, b: { x: number; y: number }): number {
    return Math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2);
}

// ============================================================================
// Part 1: Polynomial Root Finding
// ============================================================================

describe('Root Finder: Polynomial Operations', () => {
    describe('polyEval', () => {
        it('should evaluate constant polynomial', () => {
            expect(polyEval([5], 0)).toBe(5);
            expect(polyEval([5], 10)).toBe(5);
        });

        it('should evaluate linear polynomial', () => {
            expect(polyEval([2, 3], 0)).toBe(3);
            expect(polyEval([2, 3], 1)).toBe(5);
            expect(polyEval([2, 3], 2)).toBe(7);
        });

        it('should evaluate quadratic polynomial', () => {
            expect(isClose(polyEval([1, -3, 2], 0), 2)).toBe(true);
            expect(isClose(polyEval([1, -3, 2], 1), 0)).toBe(true);
            expect(isClose(polyEval([1, -3, 2], 2), 0)).toBe(true);
        });

        it('should evaluate higher order polynomial', () => {
            expect(isClose(polyEval([1, -6, 11, -6], 1), 0)).toBe(true);
            expect(isClose(polyEval([1, -6, 11, -6], 2), 0)).toBe(true);
            expect(isClose(polyEval([1, -6, 11, -6], 3), 0)).toBe(true);
        });
    });

    describe('polyDerivative', () => {
        it('should handle constant polynomial derivative', () => {
            const d = polyDerivative([5]);
            expect(d.length).toBeGreaterThanOrEqual(0);
        });

        it('should compute derivative of linear polynomial', () => {
            expect(polyDerivative([2, 3])).toEqual([2]);
        });

        it('should compute derivative of quadratic', () => {
            expect(polyDerivative([1, -3, 2])).toEqual([2, -3]);
        });

        it('should compute derivative of cubic', () => {
            expect(polyDerivative([1, 2, -1, 5])).toEqual([3, 4, -1]);
        });
    });

    describe('polyConv', () => {
        it('should multiply constant polynomials', () => {
            expect(polyConv([2], [3])).toEqual([6]);
        });

        it('should multiply linear polynomials', () => {
            expect(polyConv([1, 1], [1, -1])).toEqual([1, 0, -1]);
        });

        it('should multiply (x-1)(x-2) correctly', () => {
            expect(polyConv([1, -1], [1, -2])).toEqual([1, -3, 2]);
        });
    });

    describe('polySqr', () => {
        it('should square linear polynomial', () => {
            expect(polySqr([1, 1])).toEqual([1, 2, 1]);
        });

        it('should square x-1', () => {
            expect(polySqr([1, -1])).toEqual([1, -2, 1]);
        });

        it('should square constant', () => {
            expect(polySqr([3])).toEqual([9]);
        });
    });
});

describe('Root Finder: Cubic Solver', () => {
    it('should find roots of (x-1)(x-2)(x-3)', () => {
        const roots = solveCubic(1, -6, 11, -6);
        expect(roots.length).toBe(3);
        roots.sort((a, b) => a - b);
        expect(isClose(roots[0], 1, 1e-6)).toBe(true);
        expect(isClose(roots[1], 2, 1e-6)).toBe(true);
        expect(isClose(roots[2], 3, 1e-6)).toBe(true);
    });

    it('should handle triple root', () => {
        const roots = solveCubic(1, -6, 12, -8);
        expect(roots.length).toBeGreaterThanOrEqual(1);
        for (const root of roots) {
            expect(isClose(root, 2, 1e-4)).toBe(true);
        }
    });

    it('should handle one real root, two complex', () => {
        const roots = solveCubic(1, 0, 1, 0);
        const hasZero = roots.some(r => isClose(r, 0, 1e-6));
        expect(hasZero).toBe(true);
    });

    it('should reduce to quadratic when a=0', () => {
        const roots = solveCubic(0, 1, -3, 2);
        roots.sort((a, b) => a - b);
        expect(isClose(roots[0], 1, 1e-6)).toBe(true);
        expect(isClose(roots[1], 2, 1e-6)).toBe(true);
    });
});

describe('Root Finder: Quartic Solver', () => {
    it('should find roots of (x-1)(x-2)(x-3)(x-4)', () => {
        const roots = solveQuartic(1, -10, 35, -50, 24);
        expect(roots.length).toBe(4);
        roots.sort((a, b) => a - b);
        expect(isClose(roots[0], 1, 1e-4)).toBe(true);
        expect(isClose(roots[1], 2, 1e-4)).toBe(true);
        expect(isClose(roots[2], 3, 1e-4)).toBe(true);
        expect(isClose(roots[3], 4, 1e-4)).toBe(true);
    });

    it('should handle x⁴ - 1 = 0', () => {
        const roots = solveQuartic(1, 0, 0, 0, -1);
        expect(roots.length).toBe(2);
        roots.sort((a, b) => a - b);
        expect(isClose(roots[0], -1, 1e-6)).toBe(true);
        expect(isClose(roots[1], 1, 1e-6)).toBe(true);
    });

    it('should reduce to cubic when a=0', () => {
        const roots = solveQuartic(0, 1, -6, 11, -6);
        expect(roots.length).toBe(3);
        roots.sort((a, b) => a - b);
        expect(isClose(roots[0], 1, 1e-4)).toBe(true);
        expect(isClose(roots[1], 2, 1e-4)).toBe(true);
        expect(isClose(roots[2], 3, 1e-4)).toBe(true);
    });
});

describe('Root Finder: Safe Newton-Raphson', () => {
    it('should find root of f(x) = x² - 2 (√2)', () => {
        const root = safeNewton(
            (x) => x * x - 2,
            (x) => 2 * x,
            1, 2,
            1e-12, 100
        );
        expect(isClose(root, Math.sqrt(2), 1e-10)).toBe(true);
    });

    it('should find root of f(x) = x - 1 (trivial)', () => {
        const root = safeNewton(
            (x) => x - 1,
            () => 1,
            0, 2,
            1e-12, 100
        );
        expect(isClose(root, 1, 1e-10)).toBe(true);
    });

    it('should find root of f(x) = sin(x) - 0.5', () => {
        const root = safeNewton(
            (x) => Math.sin(x) - 0.5,
            (x) => Math.cos(x),
            0, 1,
            1e-10, 100
        );
        expect(isClose(root, Math.asin(0.5), 1e-8)).toBe(true);
    });

    it('should handle zero at boundary', () => {
        const root = safeNewton(
            (x) => x,
            () => 1,
            0, 1,
            1e-12, 100
        );
        expect(isClose(root, 0, 1e-10)).toBe(true);
    });
});

describe('Root Finder: countRoots & isolateRealRoots', () => {
    it('should count roots of (x-1)(x-2)(x-3) in [0, 4]', () => {
        expect(countRoots([1, -6, 11, -6], 0, 4)).toBe(3);
    });

    it('should count roots of (x-1)(x-2)(x-3) in [1.5, 4]', () => {
        expect(countRoots([1, -6, 11, -6], 1.5, 4)).toBe(2);
    });

    it('should count roots of x² + 1 in [-10, 10]', () => {
        expect(countRoots([1, 0, 1], -10, 10)).toBe(0);
    });

    it('should find all roots of (x-1)(x-2)(x-3)(x-4)(x-5)', () => {
        const coeffs = [1, -15, 85, -225, 274, -120];
        const roots = isolateRealRoots(coeffs, 0, 10, 1e-8);
        expect(roots.length).toBe(5);
        roots.sort((a, b) => a - b);
        for (let i = 0; i < 5; i++) {
            expect(isClose(roots[i], i + 1, 1e-6)).toBe(true);
        }
    });
});

describe('Root Finder: Polynomial Extrema', () => {
    it('should find maximum of constant polynomial', () => {
        const { maxVal } = findPolyMaxMagnitude([5], 0, 10);
        expect(isClose(maxVal, 5)).toBe(true);
    });

    it('should find maximum of linear polynomial', () => {
        const { maxVal } = findPolyMaxMagnitude([1, 0], 0, 5);
        expect(isClose(maxVal, 5)).toBe(true);
    });

    it('should find maximum of parabola', () => {
        const { maxVal } = findPolyMaxMagnitude([1, -2, 1], 0, 3);
        expect(isClose(maxVal, 4)).toBe(true);
    });

    it('should detect when polynomial exceeds threshold', () => {
        expect(polyExceedsThreshold([1, 0], 5, 0, 10)).toBe(true);
        expect(polyExceedsThreshold([1, 0], 5, 0, 3)).toBe(false);
        expect(polyExceedsThreshold([-10], 5, 0, 10)).toBe(true);
    });
});

// ============================================================================
// Part 2: Target Motion Prediction
// ============================================================================

describe('Tracking: Target Motion Prediction', () => {
    test('should predict linear motion trajectory', () => {
        const result = predictLinearMotion(
            { position: [0, 0, 0], velocity: [1, 0, 0] },
            5.0,
            0.5
        );

        expect(result.success).toBe(true);
        expect(result.positions.length).toBeGreaterThan(0);
        expect(vectorsApproxEqual(result.positions[0], [0, 0, 0])).toBe(true);

        const lastPos = result.positions[result.positions.length - 1];
        expect(vectorsApproxEqual(lastPos, [5, 0, 0])).toBe(true);
    });

    test('should predict decelerating motion', () => {
        const result = predictDeceleratingMotion(
            { position: [0, 0, 0], velocity: [2, 0, 0] },
            3.0,
            0.1,
            0.5
        );

        expect(result.success).toBe(true);
        const firstVel = result.velocities[0];
        const lastVel = result.velocities[result.velocities.length - 1];
        expect(Math.abs(lastVel[0])).toBeLessThan(Math.abs(firstVel[0]));
    });

    test('should predict motion with occupancy grid (A* prediction)', () => {
        const grid = createTestGrid(20, 0.5);
        const result = predictTargetMotion(
            { position: [0, 0, 1], velocity: [1, 0, 0] },
            grid,
            2.0,
            { timeStep: 0.2, maxVelocity: 3.0, maxAcceleration: 2.0 }
        );

        expect(result.success).toBe(true);
        expect(result.positions.length).toBeGreaterThan(1);
    });

    test('should handle zero velocity', () => {
        const result = predictLinearMotion(
            { position: [5, 5, 2], velocity: [0, 0, 0] },
            2.0,
            0.5
        );

        expect(result.success).toBe(true);
        result.positions.forEach(pos => {
            expect(vectorsApproxEqual(pos, [5, 5, 2])).toBe(true);
        });
    });
});

// ============================================================================
// Part 3: Ray Casting & Visibility
// ============================================================================

describe('Tracking: Ray Casting', () => {
    test('should detect clear ray in empty space', () => {
        const grid = createTestGrid(40, 0.5);
        const isClear = checkRayValid([0, 0, 1], [4, 0, 1], grid);
        expect(isClear).toBe(true);
    });

    test('should detect blocked ray', () => {
        const grid = createTestGrid(20, 0.5, [[2, 3, -0.5, 0.5, 0.5, 1.5]]);
        const isClear = checkRayValid([0, 0, 1], [5, 0, 1], grid);
        expect(isClear).toBe(false);
    });

    test('should handle diagonal rays', () => {
        const grid = createTestGrid(20, 0.5);
        const isClear = checkRayValid([0, 0, 1], [3, 3, 2], grid);
        expect(isClear).toBe(true);
    });
});

describe('Tracking: Visibility-Constrained Path Planning', () => {
    test('should find visible path in empty space', () => {
        const grid = createTestGrid(30, 0.5);
        const result = findVisibilityConstrainedPath(
            [0, 0, 2],
            [[5, 0, 0]],
            grid,
            3.0,
            { toleranceDistance: 0.5 }
        );

        expect(result.success).toBe(true);
        expect(result.path.length).toBeGreaterThan(0);
        expect(result.waypoints.length).toBe(1);
    });

    test('should find shortest path', () => {
        const grid = createTestGrid(20, 0.5);
        const result = findShortestPath([0, 0, 1], [3, 0, 1], grid, 0.5);

        expect(result.success).toBe(true);
        expect(result.path.length).toBeGreaterThan(0);
        expect(vectorsApproxEqual(result.path[0], [0, 0, 1], 0.5)).toBe(true);
    });

    test('should smooth path', () => {
        const grid = createTestGrid(20, 0.5);
        const path: Vector3Array[] = [
            [0, 0, 1], [1, 0, 1], [2, 0.5, 1], [3, 0, 1], [4, 0, 1]
        ];

        const smoothed = smoothPath(path, grid);
        expect(smoothed.length).toBeLessThanOrEqual(path.length);
        expect(vectorsApproxEqual(smoothed[0], path[0])).toBe(true);
        expect(vectorsApproxEqual(smoothed[smoothed.length - 1], path[path.length - 1])).toBe(true);
    });
});

describe('Tracking: Visibility Region Computation', () => {
    test('should compute visibility region in empty space', () => {
        const grid = createTestGrid(60, 0.5);
        const result = computeVisibilityPair([0, 0, 0], [0, 4, 2], 4.0, grid);

        expect(result.halfAngle).toBeGreaterThan(0);
        const dist = Math.sqrt(result.visiblePoint[0] ** 2 + result.visiblePoint[1] ** 2);
        expect(dist).toBeCloseTo(4.0, 0);
    });

    test('should compute multiple visibility regions', () => {
        const grid = createTestGrid(80, 0.5);
        const targets: Vector3Array[] = [[0, 0, 0], [3, 0, 0], [6, 0, 0]];
        const seeds: Vector3Array[] = [[0, 4, 2], [3, 4, 2], [6, 4, 2]];

        const regions = computeVisibilityRegions(targets, seeds, grid, 4.0);
        expect(regions.length).toBe(3);
        regions.forEach(region => {
            expect(region.halfAngle).toBeGreaterThanOrEqual(0);
            expect(region.distance).toBe(4.0);
        });
    });

    test('should check if point is in visibility region', () => {
        const region: VisibilityRegion = {
            center: [5, 0, 0],
            visiblePoint: [5, 5, 2],
            halfAngle: Math.PI / 4,
            distance: 5.0
        };

        expect(isInVisibilityRegion([5, 3, 2], region)).toBe(true);
        expect(isInVisibilityRegion([5, -3, 2], region)).toBe(false);
    });

    test('should generate visibility arc for visualization', () => {
        const region: VisibilityRegion = {
            center: [0, 0, 0],
            visiblePoint: [5, 0, 1],
            halfAngle: Math.PI / 3,
            distance: 5.0
        };

        const arc = generateVisibilityArc(region, 10);
        expect(arc.length).toBe(11);
        arc.forEach(point => {
            const dist = Math.sqrt(point[0] ** 2 + point[1] ** 2);
            expect(dist).toBeCloseTo(5.0, 0);
        });
    });
});

// ============================================================================
// Part 4: MINCO Constraints
// ============================================================================

describe('Tracking: MINCO Constraint Functions', () => {
    test('should compute tracking cost at desired distance', () => {
        const result = computeTrackingCost([5, 0, 2], [0, 0, 0], 5.385, 1.0);
        expect(result.cost).toBeLessThan(0.1);
    });

    test('should compute tracking cost gradient', () => {
        const result = computeTrackingCost([10, 0, 2], [0, 0, 0], 5.0, 1.0);
        expect(result.gradient[0]).toBeGreaterThan(0);
    });

    test('should compute visibility cost inside sector', () => {
        const region: VisibilityRegion = {
            center: [0, 0, 0],
            visiblePoint: [5, 0, 2],
            halfAngle: Math.PI / 4,
            distance: 5.0
        };

        const result = computeVisibilityCost([4, 0, 2], region, 1.0);
        expect(result.cost).toBe(0);
        expect(result.gradient).toEqual([0, 0, 0]);
    });

    test('should compute visibility cost outside sector', () => {
        const region: VisibilityRegion = {
            center: [0, 0, 0],
            visiblePoint: [5, 0, 2],
            halfAngle: Math.PI / 6,
            distance: 5.0
        };

        const result = computeVisibilityCost([0, 5, 2], region, 1.0);
        expect(result.cost).toBeGreaterThan(0);
    });

    test('should compute velocity constraint cost', () => {
        expect(computeVelocityCost([1, 1, 0], 3.0, 1.0).cost).toBe(0);
        expect(computeVelocityCost([3, 3, 0], 3.0, 1.0).cost).toBeGreaterThan(0);
    });

    test('should compute acceleration constraint cost', () => {
        expect(computeAccelerationCost([1, 0, 0], 2.0, 1.0).cost).toBe(0);
        expect(computeAccelerationCost([2, 2, 0], 2.0, 1.0).cost).toBeGreaterThan(0);
    });
});

// ============================================================================
// Part 5: Aerial Tracking Planner
// ============================================================================

describe('Tracking: Aerial Tracking Planner', () => {
    test('should create planner with default config', () => {
        const planner = createTrackingPlanner(5.0);
        expect(planner.getConfig().trackingDistance).toBe(5.0);
        expect(planner.getMode()).toBe('hovering');
    });

    test('should update drone and target states', () => {
        const planner = new AerialTrackingPlanner();
        planner.updateDroneState([0, 0, 2], [0, 0, 0]);
        planner.updateTargetState([5, 0, 0], [1, 0, 0]);

        const state = planner.getState();
        expect(state.drone.position).toEqual([0, 0, 2]);
        expect(state.target.velocity).toEqual([1, 0, 0]);
    });

    test('should plan tracking trajectory without map', () => {
        const planner = createTrackingPlanner(5.0);
        planner.updateDroneState([0, 0, 2], [0, 0, 0]);
        planner.updateTargetState([10, 0, 0], [0.5, 0, 0]);

        const output = planner.plan();
        expect(output.success).toBe(true);
        expect(output.path.length).toBeGreaterThan(0);
        expect(output.targetPrediction.length).toBeGreaterThan(0);
    });

    test('should plan with occupancy grid', () => {
        const grid = createTestGrid(60, 0.5);
        const planner = new AerialTrackingPlanner({ trackingDistance: 4.0 }, grid);

        planner.updateDroneState([0, 0, 2], [0, 0, 0]);
        planner.updateTargetState([6, 0, 0], [0.5, 0, 0]);

        const output = planner.plan();
        expect(typeof output.success).toBe('boolean');
        expect(Array.isArray(output.path)).toBe(true);
    });

    test('should switch to landing mode', () => {
        const planner = new AerialTrackingPlanner();
        planner.updateDroneState([5, 0, 2], [0, 0, 0]);
        planner.updateTargetState([5, 0, 0], [0.1, 0, 0]);

        planner.triggerLanding([0, 0, 0.5]);
        expect(planner.getMode()).toBe('landing');

        planner.cancelLanding();
        expect(planner.getMode()).toBe('tracking');
    });

    test('should hover when target is reached', () => {
        const planner = createTrackingPlanner(3.0);
        planner.updateDroneState([3, 0, 1], [0, 0, 0]);
        planner.updateTargetState([0, 0, 0], [0, 0, 0]);

        const output = planner.plan();
        expect(output.message).toContain('hover');
    });

    test('one-shot planning function should work', () => {
        const output = planTrackingTrajectory(
            { position: [0, 0, 2], velocity: [0, 0, 0] },
            { position: [10, 0, 0], velocity: [1, 0, 0] },
            5.0
        );

        expect(output.success).toBe(true);
        expect(output.trajectory).not.toBeNull();
    });
});

// ============================================================================
// Part 6: Default Configurations
// ============================================================================

describe('Tracking: Default Configurations', () => {
    test('should have valid default tracking config', () => {
        expect(DEFAULT_TRACKING_CONFIG.trackingDistance).toBeGreaterThan(0);
        expect(DEFAULT_TRACKING_CONFIG.maxVelocity).toBeGreaterThan(0);
        expect(DEFAULT_TRACKING_CONFIG.planningFrequency).toBeGreaterThan(0);
    });

    test('should have valid default landing config', () => {
        expect(DEFAULT_LANDING_CONFIG.approachSpeed).toBeGreaterThan(0);
        expect(DEFAULT_LANDING_CONFIG.descentSpeed).toBeGreaterThan(0);
        expect(Array.isArray(DEFAULT_LANDING_CONFIG.landingOffset)).toBe(true);
    });
});

// ============================================================================
// Part 7: Full Physics Integration Tests
// ============================================================================

describe('Planning: Full Physics Integration', () => {
    it('should navigate point-to-point without obstacles', () => {
        const robot = createTestRobot('test-robot-1', 5, 5, 0);
        const goal: Pose = { x: 15, y: 15, theta: 0 };
        const obstacles: Obstacle[] = [];
        const worldBounds = { minX: 0, maxX: 20, minY: 0, maxY: 20 };

        resetFlightState(robot.id, [robot.pose.x, robot.pose.y, 2.0]);

        const dt = 0.02;
        const maxSteps = 500;
        let currentTime = Date.now();

        for (let step = 0; step < maxSteps; step++) {
            const result = fullPhysicsTrajectoryBehavior(
                robot, goal, obstacles, [],
                currentTime, dt, worldBounds,
                { gridResolution: 0.5 },
                undefined, undefined, false
            );

            const newPose = quadrotorUpdate(robot.pose, result.velocity.vx, result.velocity.vy, result.velocity.omega, dt);
            robot.pose = newPose;
            robot.velocity = result.velocity;
            currentTime += dt * 1000;

            if (distance(robot.pose, goal) < 1.0) break;
        }

        expect(distance(robot.pose, goal)).toBeLessThan(2.0);
    });

    it('should navigate with obstacles', () => {
        const robot = createTestRobot('test-robot-2', 2, 10, 0);
        const goal: Pose = { x: 18, y: 10, theta: 0 };
        const obstacles: Obstacle[] = [{
            id: 'wall-1',
            position: { x: 10, y: 10 },
            shape: { type: 'rectangle', width: 1, height: 10 },
            color: '#ff0000'
        }];
        const worldBounds = { minX: 0, maxX: 20, minY: 0, maxY: 20 };

        resetFlightState(robot.id, [robot.pose.x, robot.pose.y, 2.0]);

        const dt = 0.02;
        const maxSteps = 750;
        let currentTime = Date.now();

        for (let step = 0; step < maxSteps; step++) {
            const result = fullPhysicsTrajectoryBehavior(
                robot, goal, obstacles, [],
                currentTime, dt, worldBounds,
                { gridResolution: 0.5, safetyMargin: 1.0 },
                undefined, undefined, false
            );

            const newPose = quadrotorUpdate(robot.pose, result.velocity.vx, result.velocity.vy, result.velocity.omega, dt);
            robot.pose = newPose;
            robot.velocity = result.velocity;
            currentTime += dt * 1000;

            if (distance(robot.pose, goal) < 1.0) break;
        }

        expect(distance(robot.pose, goal)).toBeLessThan(3.0);
    });

    it('should maintain 3D state consistency', () => {
        const robot = createTestRobot('test-robot-3', 5, 5, 0);
        const goal: Pose = { x: 10, y: 10, theta: 0 };
        const worldBounds = { minX: 0, maxX: 20, minY: 0, maxY: 20 };

        resetFlightState(robot.id, [robot.pose.x, robot.pose.y, 2.0]);

        const dt = 0.02;
        let currentTime = Date.now();

        for (let i = 0; i < 50; i++) {
            const result = fullPhysicsTrajectoryBehavior(
                robot, goal, [], [],
                currentTime, dt, worldBounds,
                {}, undefined, undefined, false
            );

            const newPose = quadrotorUpdate(robot.pose, result.velocity.vx, result.velocity.vy, result.velocity.omega, dt);
            robot.pose = newPose;
            currentTime += dt * 1000;
        }

        const flightState = getFlightState(robot.id);
        expect(flightState).not.toBeNull();

        if (flightState) {
            const qNorm = Math.sqrt(
                flightState.quaternion[0] ** 2 +
                flightState.quaternion[1] ** 2 +
                flightState.quaternion[2] ** 2 +
                flightState.quaternion[3] ** 2
            );
            expect(Math.abs(qNorm - 1.0)).toBeLessThan(0.01);
        }
    });

    it('should maintain physics stability over 500 steps', () => {
        const robot = createTestRobot('test-robot-4', 0, 0, 0);
        const goal: Pose = { x: 20, y: 20, theta: 0 };
        const worldBounds = { minX: -10, maxX: 30, minY: -10, maxY: 30 };

        resetFlightState(robot.id, [robot.pose.x, robot.pose.y, 2.0]);

        const dt = 0.02;
        let currentTime = Date.now();
        let maxVelocity = 0;

        for (let i = 0; i < 500; i++) {
            const result = fullPhysicsTrajectoryBehavior(
                robot, goal, [], [],
                currentTime, dt, worldBounds,
                { gridResolution: 0.5 },
                undefined, undefined, false
            );

            const newPose = quadrotorUpdate(robot.pose, result.velocity.vx, result.velocity.vy, result.velocity.omega, dt);
            robot.pose = newPose;

            if (result.flightState) {
                const vel = Math.sqrt(
                    result.flightState.velocity[0] ** 2 +
                    result.flightState.velocity[1] ** 2 +
                    result.flightState.velocity[2] ** 2
                );
                maxVelocity = Math.max(maxVelocity, vel);
            }

            currentTime += dt * 1000;
        }

        const flightState = getFlightState(robot.id);
        const hasNaN = isNaN(robot.pose.x) || isNaN(robot.pose.y) ||
            (flightState && flightState.velocity.some(v => isNaN(v)));

        expect(hasNaN).toBe(false);
        expect(maxVelocity).toBeLessThan(100);
    });
});

describe('Tracking: Complete Scenario Integration', () => {
    test('should track moving target through environment', () => {
        const grid = createTestGrid(40, 0.5);
        const planner = new AerialTrackingPlanner({
            trackingDistance: 5.0,
            predictionDuration: 2.0
        }, grid);

        const iterations = 5;
        let dronePos: Vector3Array = [0, 0, 2];
        let targetPos: Vector3Array = [8, 0, 0];

        for (let i = 0; i < iterations; i++) {
            planner.updateDroneState(dronePos, [0, 0, 0]);
            planner.updateTargetState(targetPos, [0.5, 0, 0]);

            const output = planner.plan();

            if (output.success && output.trajectory) {
                const nextPos = planner.getTrajectoryPosition(
                    planner.getState().trajectoryStartTime + 0.5
                );
                if (nextPos) {
                    dronePos = nextPos;
                }
            }

            targetPos = [targetPos[0] + 0.5, targetPos[1], targetPos[2]];
        }

        const horizDist = Math.sqrt(
            (dronePos[0] - targetPos[0]) ** 2 +
            (dronePos[1] - targetPos[1]) ** 2
        );

        expect(horizDist).toBeLessThan(15);
    });
});

// ============================================================================
// Part 8: MINCO Trajectory Optimization Tests
// ============================================================================

import {
    MinimumJerkTrajectory,
    MinimumSnapTrajectory,
    BandedSystem
} from '../src/models/planning/planning/minco';

describe('MINCO: BandedSystem', () => {
    it('should create banded system with given dimensions', () => {
        const bs = new BandedSystem();
        bs.create(5, 2, 2);
        expect(() => bs.reset()).not.toThrow();
    });

    it('should set and get elements', () => {
        const bs = new BandedSystem();
        bs.create(5, 2, 2);
        bs.set(2, 2, 5.0);
        expect(bs.get(2, 2)).toBe(5.0);
    });

    it('should factorize and solve simple system', () => {
        const bs = new BandedSystem();
        bs.create(3, 1, 1);

        // Set up tridiagonal system
        bs.set(0, 0, 2);
        bs.set(0, 1, -1);
        bs.set(1, 0, -1);
        bs.set(1, 1, 2);
        bs.set(1, 2, -1);
        bs.set(2, 1, -1);
        bs.set(2, 2, 2);

        const b = [[1, 0, 0], [0, 0, 0], [1, 0, 0]];

        bs.factorizeLU();
        bs.solve(b);

        // Check solution is reasonable
        expect(b[0][0]).toBeDefined();
        expect(b[1][0]).toBeDefined();
        expect(b[2][0]).toBeDefined();
    });
});

describe('MINCO: MinimumJerkTrajectory', () => {
    it('should create and set conditions', () => {
        const traj = new MinimumJerkTrajectory();

        traj.setConditions(
            { position: [0, 0, 0], velocity: [0, 0, 0] },
            { position: [10, 0, 2], velocity: [0, 0, 0] },
            3
        );

        expect(traj).toBeDefined();
    });

    it('should compute trajectory through waypoints', () => {
        const traj = new MinimumJerkTrajectory();

        traj.setConditions(
            { position: [0, 0, 0], velocity: [0, 0, 0] },
            { position: [9, 0, 0], velocity: [0, 0, 0] },
            3
        );

        const waypoints: Vector3Array[] = [[3, 0, 0], [6, 0, 0]];
        const durations = [1.0, 1.0, 1.0];

        traj.setParameters(waypoints, durations);

        const trajectory = traj.getTrajectory();
        expect(trajectory.pieces.length).toBe(3);
        expect(trajectory.pieces[0].duration).toBe(1.0);
    });

    it('should compute energy', () => {
        const traj = new MinimumJerkTrajectory();

        traj.setConditions(
            { position: [0, 0, 0], velocity: [0, 0, 0] },
            { position: [10, 0, 0], velocity: [0, 0, 0] },
            2
        );

        traj.setParameters([[5, 0, 0]], [1.0, 1.0]);

        const energy = traj.getEnergy();
        expect(energy).toBeGreaterThanOrEqual(0);
    });

    it('should get raw coefficients', () => {
        const traj = new MinimumJerkTrajectory();

        traj.setConditions(
            { position: [0, 0, 0], velocity: [1, 0, 0] },
            { position: [5, 0, 0], velocity: [1, 0, 0] },
            2
        );

        traj.setParameters([[2.5, 0, 0]], [1.0, 1.0]);

        const coeffs = traj.getCoeffs();
        expect(coeffs.length).toBe(8); // 4 * 2
    });

    it('should compute energy gradients', () => {
        const traj = new MinimumJerkTrajectory();

        traj.setConditions(
            { position: [0, 0, 0], velocity: [0, 0, 0] },
            { position: [6, 0, 0], velocity: [0, 0, 0] },
            2
        );

        traj.setParameters([[3, 0, 0]], [1.0, 1.0]);

        const gradByCoeffs = traj.getEnergyPartialGradByCoeffs();
        const gradByTimes = traj.getEnergyPartialGradByTimes();

        expect(gradByCoeffs.length).toBe(8);
        expect(gradByTimes.length).toBe(2);
    });

    it('should propagate gradients correctly', () => {
        const traj = new MinimumJerkTrajectory();

        traj.setConditions(
            { position: [0, 0, 0], velocity: [0, 0, 0] },
            { position: [6, 0, 0], velocity: [0, 0, 0] },
            2
        );

        traj.setParameters([[3, 0, 0]], [1.0, 1.0]);

        const gradByCoeffs = traj.getEnergyPartialGradByCoeffs();
        const gradByTimes = traj.getEnergyPartialGradByTimes();

        const { gradByPoints, gradByTimes: propGradTimes } = traj.propagateGrad(gradByCoeffs, gradByTimes);

        expect(gradByPoints.length).toBe(1); // N-1 = 1
        expect(propGradTimes.length).toBe(2);
    });
});

describe('MINCO: MinimumSnapTrajectory', () => {
    it('should create and set conditions with acceleration', () => {
        const traj = new MinimumSnapTrajectory();

        traj.setConditions(
            { position: [0, 0, 0], velocity: [0, 0, 0], acceleration: [0, 0, 0] },
            { position: [10, 0, 2], velocity: [0, 0, 0], acceleration: [0, 0, 0] },
            3
        );

        expect(traj).toBeDefined();
    });

    it('should compute trajectory through waypoints', () => {
        const traj = new MinimumSnapTrajectory();

        traj.setConditions(
            { position: [0, 0, 0], velocity: [0, 0, 0], acceleration: [0, 0, 0] },
            { position: [9, 0, 0], velocity: [0, 0, 0], acceleration: [0, 0, 0] },
            3
        );

        const waypoints: Vector3Array[] = [[3, 0, 0], [6, 0, 0]];
        const durations = [1.0, 1.0, 1.0];

        traj.setParameters(waypoints, durations);

        const trajectory = traj.getTrajectory();
        expect(trajectory.pieces.length).toBe(3);
        expect(trajectory.pieces[0].coeffs.length).toBe(3); // x, y, z
        expect(trajectory.pieces[0].coeffs[0].length).toBe(6); // quintic = 6 coeffs
    });

    it('should compute snap energy', () => {
        const traj = new MinimumSnapTrajectory();

        traj.setConditions(
            { position: [0, 0, 0], velocity: [0, 0, 0], acceleration: [0, 0, 0] },
            { position: [10, 0, 0], velocity: [0, 0, 0], acceleration: [0, 0, 0] },
            2
        );

        traj.setParameters([[5, 0, 0]], [1.5, 1.5]);

        const energy = traj.getEnergy();
        expect(energy).toBeGreaterThanOrEqual(0);
    });

    it('should generate smooth 3D trajectory', () => {
        const traj = new MinimumSnapTrajectory();

        traj.setConditions(
            { position: [0, 0, 1], velocity: [1, 0, 0], acceleration: [0, 0, 0] },
            { position: [10, 5, 3], velocity: [0, 1, 0], acceleration: [0, 0, 0] },
            3
        );

        const waypoints: Vector3Array[] = [[3, 1, 1.5], [7, 3, 2.5]];
        const durations = [2.0, 2.0, 2.0];

        traj.setParameters(waypoints, durations);

        const trajectory = traj.getTrajectory();

        // Verify all pieces have valid coefficients
        trajectory.pieces.forEach(piece => {
            expect(piece.duration).toBeGreaterThan(0);
            piece.coeffs.forEach(dimCoeffs => {
                expect(dimCoeffs.length).toBe(6);
                expect(dimCoeffs.every(c => !isNaN(c))).toBe(true);
            });
        });
    });

    it('should compute energy gradients for optimization', () => {
        const traj = new MinimumSnapTrajectory();

        traj.setConditions(
            { position: [0, 0, 0], velocity: [0, 0, 0], acceleration: [0, 0, 0] },
            { position: [8, 0, 0], velocity: [0, 0, 0], acceleration: [0, 0, 0] },
            2
        );

        traj.setParameters([[4, 0, 0]], [2.0, 2.0]);

        const gradByCoeffs = traj.getEnergyPartialGradByCoeffs();
        const gradByTimes = traj.getEnergyPartialGradByTimes();

        expect(gradByCoeffs.length).toBe(12); // 6 * 2
        expect(gradByTimes.length).toBe(2);
    });

    it('should propagate gradients for L-BFGS optimization', () => {
        const traj = new MinimumSnapTrajectory();

        traj.setConditions(
            { position: [0, 0, 0], velocity: [0, 0, 0], acceleration: [0, 0, 0] },
            { position: [8, 0, 0], velocity: [0, 0, 0], acceleration: [0, 0, 0] },
            2
        );

        traj.setParameters([[4, 0, 0]], [2.0, 2.0]);

        const gradByCoeffs = traj.getEnergyPartialGradByCoeffs();
        const gradByTimes = traj.getEnergyPartialGradByTimes();

        const { gradByPoints, gradByTimes: propGradTimes } = traj.propagateGrad(gradByCoeffs, gradByTimes);

        expect(gradByPoints.length).toBe(1); // N-1
        expect(propGradTimes.length).toBe(2);
    });
});

// ============================================================================
// Part 9: Navigation Behaviors Tests
// ============================================================================

import {
    dashBehavior,
    patrolBehavior,
    followBehavior,
    wanderBehavior,
    rvoBehavior,
    orcaBehavior
} from '../src/models/planning/planning/navigation';

describe('Navigation: Basic Behaviors', () => {
    it('should compute dash behavior toward goal', () => {
        const currentPose: Pose = { x: 0, y: 0, theta: 0 };
        const goal: Pose = { x: 10, y: 0, theta: 0 };

        const velocity = dashBehavior(currentPose, goal, 5.0);

        expect(velocity.vx).toBeGreaterThan(0);
        expect(Math.abs(velocity.vy)).toBeLessThan(0.1);
    });

    it('should handle goal at diagonal angle', () => {
        // Robot facing goal direction (theta = 45deg = PI/4 rad)
        const currentPose: Pose = { x: 0, y: 0, theta: Math.PI / 4 };
        const goal: Pose = { x: 10, y: 10, theta: 0 };

        const velocity = dashBehavior(currentPose, goal, 5.0);

        // When facing the goal, moves forward (vx > 0)
        expect(velocity.vx).toBeGreaterThan(0);
    });

    it('should compute patrol behavior cycling waypoints', () => {
        const robot = createTestRobot('patrol-bot', 0, 0);
        const patrolPoints: Pose[] = [
            { x: 5, y: 0, theta: 0 },
            { x: 5, y: 5, theta: 0 },
            { x: 0, y: 5, theta: 0 },
        ];

        const result = patrolBehavior(robot, patrolPoints, 3.0);

        expect(result.velocity).toBeDefined();
        expect(result.nextGoalIndex).toBeGreaterThanOrEqual(0);
    });

    it('should compute follow behavior maintaining distance', () => {
        const follower = createTestRobot('follower', 0, 0);
        const leader = createTestRobot('leader', 10, 0);

        const velocity = followBehavior(follower, leader, 5.0, 3.0);

        expect(velocity.vx).toBeGreaterThan(0); // Moving toward leader
    });

    it('should compute wander behavior within bounds', () => {
        const robot = createTestRobot('wanderer', 5, 5);
        const rangeLow: [number, number, number] = [0, 0, 0];
        const rangeHigh: [number, number, number] = [10, 10, 10];

        const result = wanderBehavior(robot, rangeLow, rangeHigh, 2.0);

        expect(result.velocity).toBeDefined();
    });
});

describe('Navigation: Collision Avoidance', () => {
    it('should compute RVO behavior avoiding neighbors', () => {
        const agent = createTestRobot('agent', 0, 0);
        const neighbors = [
            createTestRobot('neighbor1', 5, 0),
            createTestRobot('neighbor2', 0, 5),
        ];
        const goal: Pose = { x: 10, y: 10, theta: 0 };

        const velocity = rvoBehavior(agent, neighbors, goal, 3.0, 1.0);

        expect(velocity).toBeDefined();
        expect(typeof velocity.vx).toBe('number');
        expect(typeof velocity.vy).toBe('number');
    });

    it('should compute RVO with no neighbors', () => {
        const agent = createTestRobot('agent', 0, 0);
        const goal: Pose = { x: 10, y: 0, theta: 0 };

        const velocity = rvoBehavior(agent, [], goal, 5.0, 1.0);

        expect(velocity.vx).toBeGreaterThan(0);
    });

    it('should compute ORCA behavior with config', () => {
        const agent = createTestRobot('orca-agent', 0, 0);
        const neighbors = [createTestRobot('n1', 3, 3)];
        const goal: Pose = { x: 10, y: 10, theta: 0 };

        const config = {
            neighborDist: 10.0,
            maxNeighbors: 10,
            timeHorizon: 5.0,
            timeHorizonObst: 5.0,
            safeRadius: 0.5,
            maxSpeed: 3.0,
        };

        const velocity = orcaBehavior(agent, neighbors, goal, config);

        expect(velocity).toBeDefined();
    });

    it('should compute ORCA with multiple neighbors', () => {
        const agent = createTestRobot('orca-agent', 5, 5);
        const neighbors = [
            createTestRobot('n1', 6, 5),
            createTestRobot('n2', 5, 6),
            createTestRobot('n3', 4, 5),
        ];
        const goal: Pose = { x: 10, y: 10, theta: 0 };

        const config = {
            neighborDist: 5.0,
            maxNeighbors: 5,
            timeHorizon: 3.0,
            timeHorizonObst: 3.0,
            safeRadius: 0.5,
            maxSpeed: 2.0,
        };

        const velocity = orcaBehavior(agent, neighbors, goal, config);

        // Should still produce valid velocity
        expect(!isNaN(velocity.vx)).toBe(true);
        expect(!isNaN(velocity.vy)).toBe(true);
    });
});

