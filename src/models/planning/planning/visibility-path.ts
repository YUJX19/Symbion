/**
 * @module planning/visibility-path
 * @description Visibility-constrained path planning for aerial tracking.
 * 
 * Implements modified A* search algorithm that finds paths maintaining:
 * 1. A desired observation distance from the target
 * 2. Clear line-of-sight visibility to the target
 * 
 */

import type { Vector3Array } from '../trajectory/types';
import {
    positionToIndex,
    indexToPosition,
    getOccupancy,
    isValidIndex,
    indexToKey,
    type OccupancyGrid,
    type VisibilityPathOptions,
    type VisibilityPathResult,
    type SearchNode,
} from './tracking-types';
import { MinHeap } from './utils';

// ==================== Ray Casting ====================


/**
 * Check if a ray from p0 to p1 is clear of obstacles.
 * Uses 3D DDA (Digital Differential Analyzer) algorithm.
 * 
 * @param p0 Ray origin
 * @param p1 Ray destination
 * @param grid Occupancy grid
 * @returns true if the ray is clear, false if blocked
 */
export function checkRayValid(
    p0: Vector3Array,
    p1: Vector3Array,
    grid: OccupancyGrid
): boolean {
    const idx0 = positionToIndex(p0, grid);
    const idx1 = positionToIndex(p1, grid);

    // Already at destination
    if (idx0[0] === idx1[0] && idx0[1] === idx1[1] && idx0[2] === idx1[2]) {
        return !isOccupiedIndex(idx0, grid);
    }

    const dp: Vector3Array = [
        p1[0] - p0[0],
        p1[1] - p0[1],
        p1[2] - p0[2]
    ];

    const step: [number, number, number] = [
        dp[0] >= 0 ? 1 : -1,
        dp[1] >= 0 ? 1 : -1,
        dp[2] >= 0 ? 1 : -1
    ];

    // t increments per cell in each dimension
    const deltaT: Vector3Array = [
        dp[0] === 0 ? Infinity : 1.0 / Math.abs(dp[0]),
        dp[1] === 0 ? Infinity : 1.0 / Math.abs(dp[1]),
        dp[2] === 0 ? Infinity : 1.0 / Math.abs(dp[2])
    ];

    // Initial t values to reach next cell boundary
    const tMax: Vector3Array = [
        deltaT[0] * (step[0] > 0 ? 1 - ((p0[0] / grid.resolution) % 1) : (p0[0] / grid.resolution) % 1),
        deltaT[1] * (step[1] > 0 ? 1 - ((p0[1] / grid.resolution) % 1) : (p0[1] / grid.resolution) % 1),
        deltaT[2] * (step[2] > 0 ? 1 - ((p0[2] / grid.resolution) % 1) : (p0[2] / grid.resolution) % 1)
    ];

    const rayIdx: [number, number, number] = [...idx0];

    // Traverse grid cells along ray
    const maxSteps = Math.abs(idx1[0] - idx0[0]) + Math.abs(idx1[1] - idx0[1]) + Math.abs(idx1[2] - idx0[2]) + 1;

    for (let i = 0; i < maxSteps; i++) {
        if (isOccupiedIndex(rayIdx, grid)) {
            return false;
        }

        // Check if reached destination
        if (rayIdx[0] === idx1[0] && rayIdx[1] === idx1[1] && rayIdx[2] === idx1[2]) {
            break;
        }

        // Find smallest tMax and advance in that dimension
        if (tMax[0] < tMax[1]) {
            if (tMax[0] < tMax[2]) {
                rayIdx[0] += step[0];
                tMax[0] += deltaT[0];
            } else {
                rayIdx[2] += step[2];
                tMax[2] += deltaT[2];
            }
        } else {
            if (tMax[1] < tMax[2]) {
                rayIdx[1] += step[1];
                tMax[1] += deltaT[1];
            } else {
                rayIdx[2] += step[2];
                tMax[2] += deltaT[2];
            }
        }
    }

    return true;
}

/**
 * Check if grid index is occupied
 */
function isOccupiedIndex(idx: [number, number, number], grid: OccupancyGrid): boolean {
    return getOccupancy(idx, grid) > 0;
}

// ==================== Visibility-Constrained A* Search ====================

/**
 * Find a path from start position to observation positions around targets.
 * 
 * The algorithm maintains a desired distance from each target while ensuring
 * clear line-of-sight visibility. It uses a modified A* heuristic that
 * guides the search toward points on a circle of radius `desiredDistance`
 * around each target.
 * 
 * @param startPos Starting position of the drone
 * @param targetPositions Sequence of target positions to track
 * @param map Occupancy grid for collision and visibility checking
 * @param desiredDistance Desired observation distance from target
 * @param options Search options
 * @returns Path and waypoints if successful
 * 
 * @example
 * ```typescript
 * const result = findVisibilityConstrainedPath(
 *   [0, 0, 2],           // Drone start position
 *   [[5, 5, 0], [10, 5, 0]],  // Target positions
 *   occupancyGrid,
 *   5.0,                  // 5m tracking distance
 *   { toleranceDistance: 0.5 }
 * );
 * ```
 */
export function findVisibilityConstrainedPath(
    startPos: Vector3Array,
    targetPositions: Vector3Array[],
    map: OccupancyGrid,
    desiredDistance: number,
    options: VisibilityPathOptions = {}
): VisibilityPathResult {
    const tolerance = options.toleranceDistance ?? 0.5;
    const maxTime = options.maxSearchTime ?? 0.2;

    if (targetPositions.length === 0) {
        return {
            path: [startPos],
            waypoints: [],
            success: true
        };
    }

    const path: Vector3Array[] = [startPos];
    const waypoints: Vector3Array[] = [];
    let currentStart = positionToIndex(startPos, map);

    const startTime = performance.now();

    // Find visible path to each target in sequence
    for (const target of targetPositions) {
        const targetIdx = positionToIndex(target, map);

        // Check time limit
        const elapsed = (performance.now() - startTime) / 1000;
        if (elapsed > maxTime) {
            return {
                path: [],
                waypoints: [],
                success: false,
                errorMessage: `Search timeout after ${elapsed.toFixed(2)}s`
            };
        }

        // Search for observation point
        const result = searchVisiblePath(
            currentStart,
            targetIdx,
            target,
            map,
            desiredDistance,
            tolerance,
            maxTime - elapsed
        );

        if (!result.success) {
            return {
                path: [],
                waypoints: [],
                success: false,
                errorMessage: result.errorMessage
            };
        }

        // Add path segment (excluding duplicate start)
        for (let i = 1; i < result.path.length; i++) {
            path.push(indexToPosition(result.path[i], map));
        }

        // Record waypoint
        const endIdx = result.path[result.path.length - 1];
        const waypointPos = indexToPosition(endIdx, map);
        waypoints.push(waypointPos);
        currentStart = endIdx;
    }

    return {
        path,
        waypoints,
        success: true
    };
}

/**
 * Internal A* search for single target
 */
function searchVisiblePath(
    startIdx: [number, number, number],
    targetIdx: [number, number, number],
    targetPos: Vector3Array,
    map: OccupancyGrid,
    desiredDist: number,
    tolerance: number,
    maxTime: number
): { success: boolean; path: [number, number, number][]; errorMessage?: string } {

    const stopDist = desiredDist / map.resolution;
    const toleranceGrid = tolerance / map.resolution;

    // Stop condition: close to desired distance and visible
    const stopCondition = (idx: [number, number, number], h: number): boolean => {
        if (h > toleranceGrid) return false;
        const pos = indexToPosition(idx, map);
        return checkRayValid(pos, targetPos, map);
    };

    // Heuristic: distance to the circle of radius desiredDist around target
    const calculateHeuristic = (idx: [number, number, number]): number => {
        const dx = targetIdx[0] - idx[0];
        const dy = targetIdx[1] - idx[1];
        const dz = targetIdx[2] - idx[2];
        const dr = Math.sqrt(dx * dx + dy * dy); // 2D distance (horizontal)

        // Lambda: how much to interpolate toward the target circle
        const lambda = Math.max(0, 1 - stopDist / Math.max(dr, 0.1));

        const hx = Math.abs(lambda * dx);
        const hy = Math.abs(lambda * dy);
        const hz = Math.abs(dz);

        // Add cross-product term for tie-breaking
        const dx0 = startIdx[0] - targetIdx[0];
        const dy0 = startIdx[1] - targetIdx[1];
        const cross = Math.abs(dx * dy0 - dy * dx0);

        return hx + hy + hz + 0.001 * cross;
    };

    // Initialize data structures
    const visited = new Map<string, SearchNode>();
    const openSet = new MinHeap();
    const startTime = performance.now();

    // 6-connected neighbors
    const neighbors: [number, number, number][] = [
        [1, 0, 0], [-1, 0, 0],
        [0, 1, 0], [0, -1, 0],
        [0, 0, 1], [0, 0, -1]
    ];

    // Initialize start node
    const startKey = indexToKey(startIdx);
    const startH = calculateHeuristic(startIdx);
    visited.set(startKey, {
        index: startIdx,
        g: 0,
        h: startH,
        parentKey: undefined,
        state: 2 // closed
    });

    if (stopCondition(startIdx, startH)) {
        return { success: true, path: [startIdx] };
    }

    // Add neighbors of start to open set
    for (const [dx, dy, dz] of neighbors) {
        const neighborIdx: [number, number, number] = [
            startIdx[0] + dx,
            startIdx[1] + dy,
            startIdx[2] + dz
        ];

        if (!isValidIndex(neighborIdx, map) || isOccupiedIndex(neighborIdx, map)) {
            continue;
        }

        const neighborKey = indexToKey(neighborIdx);
        const h = calculateHeuristic(neighborIdx);
        visited.set(neighborKey, {
            index: neighborIdx,
            g: 1,
            h,
            parentKey: startKey,
            state: 1 // open
        });
        openSet.push(neighborKey, 1 + h);
    }

    // A* main loop
    while (!openSet.isEmpty()) {
        // Check time limit
        if ((performance.now() - startTime) / 1000 > maxTime) {
            return { success: false, path: [], errorMessage: 'Search timeout' };
        }

        const current = openSet.pop()!;
        const currentNode = visited.get(current.key)!;
        currentNode.state = 2; // mark as closed

        // Check stop condition
        if (stopCondition(currentNode.index, currentNode.h)) {
            // Reconstruct path
            const path: [number, number, number][] = [];
            let key: string | undefined = current.key;
            while (key !== undefined) {
                const foundNode: SearchNode = visited.get(key)!;
                path.unshift(foundNode.index);
                key = foundNode.parentKey;
            }
            return { success: true, path };
        }

        // Expand neighbors
        for (const [dx, dy, dz] of neighbors) {
            const neighborIdx: [number, number, number] = [
                currentNode.index[0] + dx,
                currentNode.index[1] + dy,
                currentNode.index[2] + dz
            ];

            if (!isValidIndex(neighborIdx, map) || isOccupiedIndex(neighborIdx, map)) {
                continue;
            }

            const neighborKey = indexToKey(neighborIdx);
            const existingNode = visited.get(neighborKey);

            if (existingNode) {
                if (existingNode.state === 2) continue; // already closed

                // Check if this path is better
                const newG = currentNode.g + 1;
                if (newG < existingNode.g) {
                    existingNode.g = newG;
                    existingNode.parentKey = current.key;
                    // Note: we don't update priority in heap (lazy evaluation)
                }
            } else {
                // New node
                const h = calculateHeuristic(neighborIdx);
                const g = currentNode.g + 1;
                visited.set(neighborKey, {
                    index: neighborIdx,
                    g,
                    h,
                    parentKey: current.key,
                    state: 1 // open
                });
                openSet.push(neighborKey, g + h);
            }
        }

        // Memory check
        if (visited.size > 100000) {
            return { success: false, path: [], errorMessage: 'Search out of memory' };
        }
    }

    return { success: false, path: [], errorMessage: 'No valid path found' };
}

// ==================== Simple A* Path Finding ====================

/**
 * Standard A* path finding to a target position.
 * 
 * Unlike visibility-constrained search, this finds the shortest path
 * directly to the target position without visibility requirements.
 * 
 * @param startPos Starting position
 * @param goalPos Goal position
 * @param map Occupancy grid
 * @param maxTime Maximum search time in seconds
 * @returns Path if found
 */
export function findShortestPath(
    startPos: Vector3Array,
    goalPos: Vector3Array,
    map: OccupancyGrid,
    maxTime: number = 0.2
): { success: boolean; path: Vector3Array[]; errorMessage?: string } {

    const startIdx = positionToIndex(startPos, map);
    const goalIdx = positionToIndex(goalPos, map);

    // Check if start and goal are valid
    if (isOccupiedIndex(startIdx, map)) {
        return { success: false, path: [], errorMessage: 'Start position is occupied' };
    }

    // Heuristic: Manhattan distance
    const heuristic = (idx: [number, number, number]): number => {
        return Math.abs(goalIdx[0] - idx[0]) +
            Math.abs(goalIdx[1] - idx[1]) +
            Math.abs(goalIdx[2] - idx[2]);
    };

    const visited = new Map<string, SearchNode>();
    const openSet = new MinHeap();
    const startTime = performance.now();

    const neighbors: [number, number, number][] = [
        [1, 0, 0], [-1, 0, 0],
        [0, 1, 0], [0, -1, 0],
        [0, 0, 1], [0, 0, -1]
    ];

    const startKey = indexToKey(startIdx);
    visited.set(startKey, {
        index: startIdx,
        g: 0,
        h: heuristic(startIdx),
        parentKey: undefined,
        state: 1
    });
    openSet.push(startKey, heuristic(startIdx));

    while (!openSet.isEmpty()) {
        if ((performance.now() - startTime) / 1000 > maxTime) {
            return { success: false, path: [], errorMessage: 'Search timeout' };
        }

        const current = openSet.pop()!;
        const currentNode = visited.get(current.key)!;

        if (currentNode.state === 2) continue;
        currentNode.state = 2;

        // Check goal
        if (currentNode.index[0] === goalIdx[0] &&
            currentNode.index[1] === goalIdx[1] &&
            currentNode.index[2] === goalIdx[2]) {

            const path: Vector3Array[] = [];
            let key: string | undefined = current.key;
            while (key !== undefined) {
                const foundNode: SearchNode = visited.get(key)!;
                path.unshift(indexToPosition(foundNode.index, map));
                key = foundNode.parentKey;
            }
            return { success: true, path };
        }

        // Expand neighbors
        for (const [dx, dy, dz] of neighbors) {
            const neighborIdx: [number, number, number] = [
                currentNode.index[0] + dx,
                currentNode.index[1] + dy,
                currentNode.index[2] + dz
            ];

            if (!isValidIndex(neighborIdx, map) || isOccupiedIndex(neighborIdx, map)) {
                continue;
            }

            const neighborKey = indexToKey(neighborIdx);
            const existingNode = visited.get(neighborKey);
            const newG = currentNode.g + 1;

            if (!existingNode || newG < existingNode.g) {
                const h = heuristic(neighborIdx);
                visited.set(neighborKey, {
                    index: neighborIdx,
                    g: newG,
                    h,
                    parentKey: current.key,
                    state: 1
                });
                openSet.push(neighborKey, newG + h);
            }
        }

        if (visited.size > 100000) {
            return { success: false, path: [], errorMessage: 'Search out of memory' };
        }
    }

    return { success: false, path: [], errorMessage: 'No path found' };
}

// ==================== Path Smoothing ====================

/**
 * Smooth a path by removing unnecessary waypoints.
 * 
 * Uses line-of-sight checks to skip intermediate points
 * when direct paths are available.
 * 
 * @param path Original path
 * @param map Occupancy grid for collision checking
 * @returns Smoothed path with fewer waypoints
 */
export function smoothPath(
    path: Vector3Array[],
    map: OccupancyGrid
): Vector3Array[] {
    if (path.length <= 2) return path;

    const smoothed: Vector3Array[] = [path[0]];
    let currentIdx = 0;

    while (currentIdx < path.length - 1) {
        // Find the farthest point we can reach directly
        let farthest = currentIdx + 1;

        for (let i = currentIdx + 2; i < path.length; i++) {
            if (checkRayValid(path[currentIdx], path[i], map)) {
                farthest = i;
            }
        }

        smoothed.push(path[farthest]);
        currentIdx = farthest;
    }

    return smoothed;
}

/**
 * Convert path indices to world positions
 */
export function pathToPositions(
    path: [number, number, number][],
    map: OccupancyGrid
): Vector3Array[] {
    return path.map(idx => indexToPosition(idx, map));
}
