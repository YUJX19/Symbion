/**
 * @module utils/occupancy-grid-builder
 * @description Utilities for building OccupancyGrid from Three.js scene elements.
 * 
 * Handles coordinate transformation between Three.js (Y-up) and Physics/MINCO (Z-up).
 */

import type { Vector3Array } from '../planning/trajectory/types';
import {
    OccupancyGrid,
    createOccupancyGrid,
    setOccupancy,
    positionToIndex,
    isValidIndex
} from '../planning/planning/tracking-types';

/**
 * Building representation in Three.js coordinate system
 * (matches the typical Three.js mesh representation)
 */
export interface ThreeJSBuilding {
    /** Center position of the building in Three.js coords */
    position: { x: number; y: number; z: number };
    /** Width (size along Three.js X-axis) */
    width: number;
    /** Height (size along Three.js Y-axis, vertical) */
    height: number;
    /** Depth (size along Three.js Z-axis) */
    depth: number;
}

/**
 * Configuration for occupancy grid generation
 */
export interface OccupancyGridConfig {
    /** Grid resolution in meters (default: 1.0) */
    resolution?: number;
    /** Safety margin around obstacles in meters (default: 0.5) */
    safetyMargin?: number;
    /** Maximum height to include in grid (default: 100.0) */
    maxHeight?: number;
    /** Custom origin in Physics coordinates (default: computed from bounds) */
    origin?: Vector3Array;
    /** Custom dimensions in cells (default: computed from bounds) */
    dimensions?: [number, number, number];
}

/**
 * Convert Three.js position to Physics/MINCO coordinates
 * Three.js: [x=right, y=up, z=forward]
 * Physics:  [x=forward, y=right, z=up]
 */
function threeToPhysics(three: { x: number; y: number; z: number }): Vector3Array {
    return [three.z, three.x, three.y];
}

/**
 * Build an OccupancyGrid from an array of Three.js buildings.
 * 
 * The resulting grid uses Physics/MINCO coordinates (Z-up).
 * 
 * @param buildings - Array of buildings in Three.js coordinates
 * @param config - Grid configuration options
 * @returns OccupancyGrid suitable for path planning
 * 
 * @example
 * ```typescript
 * const buildings = [
 *   { position: { x: 0, y: 10, z: 50 }, width: 10, height: 20, depth: 10 }
 * ];
 * const grid = buildOccupancyGridFromThreeJS(buildings, { resolution: 2.0 });
 * ```
 */
export function buildOccupancyGridFromThreeJS(
    buildings: ThreeJSBuilding[],
    config: OccupancyGridConfig = {}
): OccupancyGrid {
    const resolution = config.resolution ?? 1.0;
    const safetyMargin = config.safetyMargin ?? 0.5;
    const maxHeight = config.maxHeight ?? 100.0;

    // Compute bounds in Physics coordinates if not provided
    const minBound: Vector3Array = [Infinity, Infinity, 0];  // Ground at z=0
    const maxBound: Vector3Array = [-Infinity, -Infinity, maxHeight];

    for (const bld of buildings) {
        const center = threeToPhysics(bld.position);
        // In Physics coords: 
        // Three.js depth (z) -> Physics x (forward)
        // Three.js width (x) -> Physics y (right)
        // Three.js height (y) -> Physics z (up)
        const halfX = bld.depth / 2;
        const halfY = bld.width / 2;

        minBound[0] = Math.min(minBound[0], center[0] - halfX - safetyMargin);
        minBound[1] = Math.min(minBound[1], center[1] - halfY - safetyMargin);
        maxBound[0] = Math.max(maxBound[0], center[0] + halfX + safetyMargin);
        maxBound[1] = Math.max(maxBound[1], center[1] + halfY + safetyMargin);
    }

    // Add some padding to bounds
    const padding = 20; // 20m padding around all buildings
    minBound[0] -= padding;
    minBound[1] -= padding;
    maxBound[0] += padding;
    maxBound[1] += padding;

    // Use provided origin/dimensions or compute from bounds
    const origin = config.origin ?? [minBound[0], minBound[1], 0] as Vector3Array;
    const dimensions = config.dimensions ?? [
        Math.ceil((maxBound[0] - origin[0]) / resolution),
        Math.ceil((maxBound[1] - origin[1]) / resolution),
        Math.ceil(maxHeight / resolution)
    ] as [number, number, number];

    // Create empty grid
    const grid = createOccupancyGrid(dimensions, resolution, origin);

    // Add each building to the grid
    for (const bld of buildings) {
        addBuildingToGrid(bld, grid, safetyMargin);
    }

    console.log(`[OccupancyGrid] Created ${dimensions[0]}x${dimensions[1]}x${dimensions[2]} grid`);
    console.log(`  Origin: [${origin.map(v => v.toFixed(1)).join(', ')}]`);
    console.log(`  Resolution: ${resolution}m`);
    console.log(`  Buildings: ${buildings.length}`);

    return grid;
}

/**
 * Add a single Three.js building to an occupancy grid
 */
function addBuildingToGrid(
    building: ThreeJSBuilding,
    grid: OccupancyGrid,
    safetyMargin: number = 0.5
): void {
    const center = threeToPhysics(building.position);

    // Building extends in Physics coordinates
    // Three.js depth -> Physics X (forward)
    // Three.js width -> Physics Y (right)  
    // Three.js height -> Physics Z (up)
    const halfX = (building.depth / 2) + safetyMargin;
    const halfY = (building.width / 2) + safetyMargin;

    // Building goes from ground (0) to height
    const minZ = 0;
    const maxZ = building.height + safetyMargin;

    // Calculate bounding box in Physics coordinates
    const minPos: Vector3Array = [center[0] - halfX, center[1] - halfY, minZ];
    const maxPos: Vector3Array = [center[0] + halfX, center[1] + halfY, maxZ];

    // Convert to grid indices
    const minIdx = positionToIndex(minPos, grid);
    const maxIdx = positionToIndex(maxPos, grid);

    // Fill all cells within the building
    for (let ix = minIdx[0]; ix <= maxIdx[0]; ix++) {
        for (let iy = minIdx[1]; iy <= maxIdx[1]; iy++) {
            for (let iz = minIdx[2]; iz <= maxIdx[2]; iz++) {
                if (isValidIndex([ix, iy, iz], grid)) {
                    setOccupancy([ix, iy, iz], 1, grid);
                }
            }
        }
    }
}

/**
 * Check if a position in Physics coordinates is collision-free
 * 
 * @param position - Position to check in Physics/MINCO coordinates
 * @param grid - Occupancy grid
 * @param safetyRadius - Additional safety radius in meters (default: 0)
 * @returns true if position is free from obstacles
 */
export function isPositionFree(
    position: Vector3Array,
    grid: OccupancyGrid,
    safetyRadius: number = 0
): boolean {
    if (safetyRadius <= 0) {
        const idx = positionToIndex(position, grid);
        if (!isValidIndex(idx, grid)) return false; // Out of bounds = blocked
        const flatIndex = idx[0] +
            idx[1] * grid.dimensions[0] +
            idx[2] * grid.dimensions[0] * grid.dimensions[1];
        return grid.data[flatIndex] === 0;
    }

    // Check sphere around position
    const steps = Math.ceil(safetyRadius / grid.resolution);
    for (let dx = -steps; dx <= steps; dx++) {
        for (let dy = -steps; dy <= steps; dy++) {
            for (let dz = -steps; dz <= steps; dz++) {
                const dist = Math.sqrt(
                    (dx * grid.resolution) ** 2 +
                    (dy * grid.resolution) ** 2 +
                    (dz * grid.resolution) ** 2
                );
                if (dist <= safetyRadius) {
                    const checkPos: Vector3Array = [
                        position[0] + dx * grid.resolution,
                        position[1] + dy * grid.resolution,
                        position[2] + dz * grid.resolution
                    ];
                    const idx = positionToIndex(checkPos, grid);
                    if (!isValidIndex(idx, grid)) return false;
                    const flatIndex = idx[0] +
                        idx[1] * grid.dimensions[0] +
                        idx[2] * grid.dimensions[0] * grid.dimensions[1];
                    if (grid.data[flatIndex] !== 0) return false;
                }
            }
        }
    }
    return true;
}

/**
 * Check if a line segment in Physics coordinates is collision-free
 * 
 * @param start - Start position in Physics coordinates
 * @param end - End position in Physics coordinates
 * @param grid - Occupancy grid
 * @param stepSize - Sampling interval along the line (default: 0.5m)
 * @returns true if entire line segment is free from obstacles
 */
export function isLineFree(
    start: Vector3Array,
    end: Vector3Array,
    grid: OccupancyGrid,
    stepSize: number = 0.5
): boolean {
    const dx = end[0] - start[0];
    const dy = end[1] - start[1];
    const dz = end[2] - start[2];
    const length = Math.sqrt(dx * dx + dy * dy + dz * dz);

    if (length < 0.001) {
        return isPositionFree(start, grid);
    }

    const steps = Math.ceil(length / stepSize);
    for (let i = 0; i <= steps; i++) {
        const t = i / steps;
        const point: Vector3Array = [
            start[0] + dx * t,
            start[1] + dy * t,
            start[2] + dz * t
        ];
        if (!isPositionFree(point, grid)) {
            return false;
        }
    }
    return true;
}

/**
 * Find the closest free position to a given position
 * 
 * @param position - Target position in Physics coordinates
 * @param grid - Occupancy grid
 * @param maxSearchRadius - Maximum search radius in meters (default: 10)
 * @returns Closest free position, or null if none found
 */
export function findClosestFreePosition(
    position: Vector3Array,
    grid: OccupancyGrid,
    maxSearchRadius: number = 10
): Vector3Array | null {
    if (isPositionFree(position, grid)) {
        return position;
    }

    const steps = Math.ceil(maxSearchRadius / grid.resolution);
    let closestDist = Infinity;
    let closestPos: Vector3Array | null = null;

    for (let dx = -steps; dx <= steps; dx++) {
        for (let dy = -steps; dy <= steps; dy++) {
            for (let dz = -steps; dz <= steps; dz++) {
                const checkPos: Vector3Array = [
                    position[0] + dx * grid.resolution,
                    position[1] + dy * grid.resolution,
                    position[2] + dz * grid.resolution
                ];
                const dist = Math.sqrt(
                    (dx * grid.resolution) ** 2 +
                    (dy * grid.resolution) ** 2 +
                    (dz * grid.resolution) ** 2
                );
                if (dist < closestDist && dist <= maxSearchRadius) {
                    if (isPositionFree(checkPos, grid)) {
                        closestDist = dist;
                        closestPos = checkPos;
                    }
                }
            }
        }
    }

    return closestPos;
}
