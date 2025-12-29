/**
 * @module sensing/environment
 * @description Environment generation, map utilities, and obstacle placement
 */

import { Obstacle, Pose, Vector2, GridMap, WorldConfig, Robot } from './types';

// ==================== Color Palette ====================

/**
 * Standard colors for robot identification
 */
export const ROBOT_COLORS = [
    '#f97316', // orange
    '#06b6d4', // cyan
    '#8b5cf6', // violet
    '#10b981', // emerald
    '#f43f5e', // rose
    '#3b82f6', // blue
    '#eab308', // yellow
    '#ec4899', // pink
];

/**
 * Gets a color associated with a robot index
 */
export function getRobotColor(index: number): string {
    return ROBOT_COLORS[index % ROBOT_COLORS.length];
}

// ==================== Map Generation ====================

/**
 * Simple seeded Random Number Generator
 */
class SeededRNG {
    private seed: number;
    constructor(seed: number = 0) {
        this.seed = seed;
    }
    /**
     * Returns a pseudorandom number between 0 and 1
     */
    next(): number {
        // Simple linear congruential generator
        this.seed = (this.seed * 9301 + 49297) % 233280;
        return this.seed / 233280;
    }
}

/**
 * Generate a set of random obstacles in the world
 * 
 * Supports legacy signature (count, world, seed) OR (count, width, height, minSize, maxSize)
 */
export function generateRandomObstacles(
    count: number,
    worldOrWidth: number | { width: number, height: number },
    heightOrSeed?: number,
    minSizeOrType?: number | string,
    maxSizeOrSize?: number
): Obstacle[] {
    let worldWidth: number;
    let worldHeight: number;
    let rng: () => number = Math.random;
    let minSize = 1;
    let maxSizeVal = 3;
    let forcedType: string | undefined;

    if (typeof worldOrWidth === 'number') {
        // Signature: (count, width, height, [minSize], [maxSize])
        worldWidth = worldOrWidth;
        worldHeight = heightOrSeed!;
        minSize = (minSizeOrType as number) || 1;
        maxSizeVal = maxSizeOrSize || 3;
    } else {
        // Signature: (count, worldConfig, seed, [type], [size])
        worldWidth = worldOrWidth.width;
        worldHeight = worldOrWidth.height;
        if (typeof heightOrSeed === 'number') {
            const seedGen = new SeededRNG(heightOrSeed);
            rng = () => seedGen.next();
        }

        if (typeof minSizeOrType === 'string') {
            forcedType = minSizeOrType;
            if (maxSizeOrSize !== undefined) {
                minSize = maxSizeOrSize;
                maxSizeVal = maxSizeOrSize;
            }
        } else if (typeof minSizeOrType === 'number') {
            minSize = minSizeOrType;
            if (maxSizeOrSize !== undefined) {
                maxSizeVal = maxSizeOrSize;
            }
        }
    }

    const obstacles: Obstacle[] = [];

    for (let i = 0; i < count; i++) {
        const x = rng() * (worldWidth - 4) + 2;
        const y = rng() * (worldHeight - 4) + 2;
        const size = minSize + rng() * (maxSizeVal - minSize);

        // Determine obstacle shape type
        let obsType: 'circle' | 'rectangle' = 'circle';
        if (forcedType === 'circle' || forcedType === 'rectangle') {
            obsType = forcedType;
        } else {
            obsType = rng() > 0.5 ? 'circle' : 'rectangle';
        }

        obstacles.push({
            id: `obs-${i}`,
            position: { x, y },
            shape: {
                type: obsType,
                radius: size / 2,        // Used if circle
                width: size,             // Used if rectangle
                height: size             // Used if rectangle
            },
            color: '#64748b'
        });
    }

    return obstacles;
}

/**
 * Configuration options for random polygon generation
 */
interface PolygonOptions {
    /** [minX, minY, maxX, maxY] placement range */
    centerRange?: [number, number, number, number];
    /** Range of average radius for vertices */
    avgRadiusRange?: [number, number];
    /** Degree of irregularity [0, 1] */
    irregularityRange?: [number, number];
    /** Degree of vertex spikiness [0, 1] */
    spikeynessRange?: [number, number];
    /** Range of vertices count */
    numVerticesRange?: [number, number];
}

/**
 * Generate random polygon obstacles (IR-SIM style)
 */
export function generateRandomPolygonObstacles(
    count: number,
    worldOrWidth: number | { width: number, height: number },
    heightOrSeed?: number,
    options?: PolygonOptions
): Obstacle[] {
    let worldWidth: number;
    let worldHeight: number;
    let rng: () => number = Math.random;
    let opts: PolygonOptions = {};

    if (typeof worldOrWidth === 'number') {
        // Signature: (count, width, height)
        worldWidth = worldOrWidth;
        worldHeight = heightOrSeed!;
    } else {
        // Signature: (count, worldConfig, seed, options)
        worldWidth = worldOrWidth.width;
        worldHeight = worldOrWidth.height;
        if (typeof heightOrSeed === 'number') {
            const seedGen = new SeededRNG(heightOrSeed);
            rng = () => seedGen.next();
        }
        opts = options || {};
    }

    const {
        centerRange = [2, 2, worldWidth - 2, worldHeight - 2],
        avgRadiusRange = [1, 2.5],
        numVerticesRange = [3, 6],
        irregularityRange = [0, 1],
        spikeynessRange = [0, 1]
    } = opts;

    const obstacles: Obstacle[] = [];

    for (let i = 0; i < count; i++) {
        const cx = centerRange[0] + rng() * (centerRange[2] - centerRange[0]);
        const cy = centerRange[1] + rng() * (centerRange[3] - centerRange[1]);

        const minV = numVerticesRange[0];
        const maxV = numVerticesRange[1];
        const numVertices = Math.floor(rng() * (maxV - minV + 1)) + minV;

        const minR = avgRadiusRange[0];
        const maxR = avgRadiusRange[1];
        const avgRadius = minR + rng() * (maxR - minR);

        // Generate vertices relative to center (0,0)
        const vertices: Vector2[] = [];
        for (let j = 0; j < numVertices; j++) {
            const angle = (j / numVertices) * Math.PI * 2 + (rng() - 0.5) * 0.5;
            const r = avgRadius * (0.8 + rng() * 0.4); // Random variance in radius
            vertices.push({
                x: Math.cos(angle) * r,
                y: Math.sin(angle) * r
            });
        }

        obstacles.push({
            id: `poly-${i}`,
            position: { x: cx, y: cy }, // Store actual center position
            shape: {
                type: 'polygon',
                vertices: vertices // Relative to position
            },
            color: '#64748b'
        });
    }

    return obstacles;
}

/**
 * Generate a circular distribution of robots (frequently used in multi-agent tests)
 */
export function generateCircleDistribution(
    numRobots: number,
    centerX: number,
    centerY: number,
    radius: number
): { start: Pose, goal: Pose }[] {
    const positions: { start: Pose, goal: Pose }[] = [];

    for (let i = 0; i < numRobots; i++) {
        const angle = (i / numRobots) * Math.PI * 2;

        positions.push({
            start: {
                x: centerX + Math.cos(angle) * radius,
                y: centerY + Math.sin(angle) * radius,
                theta: angle + Math.PI // Orient to face inward
            },
            goal: {
                x: centerX + Math.cos(angle + Math.PI) * radius,
                y: centerY + Math.sin(angle + Math.PI) * radius,
                theta: 0
            }
        });
    }

    return positions;
}

/**
 * Generate a random distribution of robot poses in the world
 */
export function generateRandomDistribution(
    numRobots: number,
    worldWidth: number,
    worldHeight: number,
    minDist: number = 1.0
): Pose[] {
    const poses: Pose[] = [];

    let attempts = 0;
    while (poses.length < numRobots && attempts < 1000) {
        attempts++;
        const x = 1 + Math.random() * (worldWidth - 2);
        const y = 1 + Math.random() * (worldHeight - 2);

        // Check distance against existing poses to avoid overlaps
        let valid = true;
        for (const p of poses) {
            const d = Math.sqrt((p.x - x) ** 2 + (p.y - y) ** 2);
            if (d < minDist) {
                valid = false;
                break;
            }
        }

        if (valid) {
            poses.push({ x, y, theta: Math.random() * Math.PI * 2 });
        }
    }

    return poses;
}

// ==================== Grid Map Helpers ====================

/**
 * Create a simple empty grid map with a surrounding boundary wall
 */
export function createSimpleGridMap(
    worldWidth: number,
    worldHeight: number,
    resolution: number
): GridMap {
    const cellsX = Math.ceil(worldWidth / resolution);
    const cellsY = Math.ceil(worldHeight / resolution);

    const data: number[][] = [];

    for (let y = 0; y < cellsY; y++) {
        const row: number[] = [];
        for (let x = 0; x < cellsX; x++) {
            // Add walls on map borders
            if (x === 0 || x === cellsX - 1 || y === 0 || y === cellsY - 1) {
                row.push(1);
            } else {
                row.push(0);
            }
        }
        data.push(row);
    }

    return {
        data,
        resolution,
        origin: { x: 0, y: 0 },
        width: cellsX,
        height: cellsY
    };
}

/**
 * Create a grid map with maze-like randomized internal walls
 */
export function createMazeGridMap(
    worldWidth: number,
    worldHeight: number,
    resolution: number,
    seed?: number
): GridMap {
    const cellsX = Math.ceil(worldWidth / resolution);
    const cellsY = Math.ceil(worldHeight / resolution);

    const data: number[][] = [];

    let rng: () => number = Math.random;
    if (typeof seed === 'number') {
        const seedGen = new SeededRNG(seed);
        rng = () => seedGen.next();
    }

    for (let y = 0; y < cellsY; y++) {
        const row: number[] = [];
        for (let x = 0; x < cellsX; x++) {
            // Borders
            if (x === 0 || x === cellsX - 1 || y === 0 || y === cellsY - 1) {
                row.push(1);
            } else if (x % 4 === 0 && y % 4 !== 0 && rng() > 0.3) {
                // Stochastic vertical wall structures
                row.push(1);
            } else if (y % 4 === 0 && x % 4 !== 0 && rng() > 0.3) {
                // Stochastic horizontal wall structures
                row.push(1);
            } else {
                row.push(0);
            }
        }
        data.push(row);
    }

    return {
        data,
        resolution,
        origin: { x: 0, y: 0 },
        width: cellsX,
        height: cellsY
    };
}

/**
 * Coordinate transform: Real-world meters to Grid cells
 */
export function worldToGrid(
    worldPos: Vector2,
    gridMap: GridMap
): { gx: number, gy: number } {
    const gx = Math.floor((worldPos.x - gridMap.origin.x) / gridMap.resolution);
    const gy = Math.floor((worldPos.y - gridMap.origin.y) / gridMap.resolution);
    return { gx, gy };
}

/**
 * Coordinate transform: Grid cells to Real-world meters (center of cell)
 */
export function gridToWorld(
    gx: number,
    gy: number,
    gridMap: GridMap
): Vector2 {
    return {
        x: gridMap.origin.x + (gx + 0.5) * gridMap.resolution,
        y: gridMap.origin.y + (gy + 0.5) * gridMap.resolution
    };
}

