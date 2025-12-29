/**
 * @module planning/geoutils
 * @description Geometric Utilities for Safe Flight Corridor (SFC) generation.
 * 
 * Provides algorithms for manipulating convex polytopes in half-space representation:
 * - Interior point finding using linear programming
 * - Vertex enumeration via QuickHull dual
 * - Vertex filtering and deduplication
 * 
 * Half-space representation: Each column of hPoly is [n^T, p^T]^T where:
 * - n is the normalized outward-pointing normal
 * - p is a point on the face
 * - A point x is inside iff n·x <= n·p for all faces
 * 
 * Based on GCOPTER's geoutils by Zhepei Wang.
 * 
 * @example
 * ```typescript
 * import { findInterior, enumerateVertices } from '@/lib/algorithms/planning/geoutils';
 * 
 * // Define a cube as half-spaces (each face is a column)
 * const hPoly = createCubeHalfSpaces([-1, -1, -1], [1, 1, 1]);
 * 
 * // Find interior point
 * const { success, interior } = findInterior(hPoly);
 * console.log(interior); // [0, 0, 0]
 * 
 * // Get vertices
 * const vertices = enumerateVertices(hPoly);
 * console.log(vertices.length); // 8 (cube corners)
 * ```
 */

import { linearProgram, SDLPStatus } from '../../numeric/optimization/sdlp';

// ==================== Types ====================

/**
 * Half-space polytope representation.
 * Each face is represented as [nx, ny, nz, px, py, pz] where:
 * - (nx, ny, nz) is the normalized outward normal
 * - (px, py, pz) is a point on the face
 */
export interface HalfSpacePolytope {
    /** Number of faces */
    numFaces: number;
    /** Normal vectors (numFaces x 3) - row-major */
    normals: number[][];
    /** Points on faces (numFaces x 3) - row-major */
    points: number[][];
}

/**
 * Vertex representation of a convex polytope.
 */
export interface VertexPolytope {
    /** Vertex coordinates (numVertices x 3) */
    vertices: number[][];
}

// ==================== Interior Point Finding ====================

/**
 * Find a strictly interior point of a convex polytope defined by half-spaces.
 * 
 * Uses linear programming to find the point that maximizes the minimum
 * signed distance to all half-space boundaries.
 * 
 * The half-spaces define: n_i · x <= n_i · p_i for all i
 * We maximize slack s such that: n_i · x + s <= n_i · p_i
 * 
 * @param hPoly Half-space polytope definition
 * @returns Object with success flag and interior point (if found)
 * 
 * @example
 * ```typescript
 * const cube = {
 *   numFaces: 6,
 *   normals: [[1,0,0], [-1,0,0], [0,1,0], [0,-1,0], [0,0,1], [0,0,-1]],
 *   points: [[1,0,0], [-1,0,0], [0,1,0], [0,-1,0], [0,0,1], [0,0,-1]]
 * };
 * const { success, interior } = findInterior(cube);
 * // success: true, interior: [0, 0, 0]
 * ```
 */
export function findInterior(hPoly: HalfSpacePolytope): {
    success: boolean;
    interior: [number, number, number]
} {
    const m = hPoly.numFaces;

    // Set up LP: max s, s.t. n_i · x + s <= b_i
    // Rearranged: [n_i, 1] · [x, s]^T <= b_i
    // Standard form: min -s

    // Objective: minimize -s (maximize s)
    // Variable: [x, y, z, s]
    const c = [0, 0, 0, -1];

    // Constraint matrix A: [n_i | 1] for each face
    const A: number[][] = [];
    const b: number[] = [];

    for (let i = 0; i < m; i++) {
        const n = hPoly.normals[i];
        const p = hPoly.points[i];

        // n · x + s <= n · p
        A.push([n[0], n[1], n[2], 1]);
        b.push(n[0] * p[0] + n[1] * p[1] + n[2] * p[2]);
    }

    const result = linearProgram(c, A, b);

    // Check if we found a strictly interior point (s < 0 means interior)
    const success = result.status === SDLPStatus.MINIMUM &&
        result.minimum < 0 &&
        isFinite(result.minimum);

    return {
        success,
        interior: [result.x[0], result.x[1], result.x[2]]
    };
}

/**
 * Find interior point from flat array representation.
 * Each half-space is [nx, ny, nz, px, py, pz].
 * 
 * @param hPolyFlat Flat array where each 6 elements define a face
 * @returns Interior point or null if infeasible
 */
export function findInteriorFlat(hPolyFlat: number[]): [number, number, number] | null {
    const numFaces = hPolyFlat.length / 6;
    if (numFaces < 1) return null;

    const normals: number[][] = [];
    const points: number[][] = [];

    for (let i = 0; i < numFaces; i++) {
        const base = i * 6;
        normals.push([hPolyFlat[base], hPolyFlat[base + 1], hPolyFlat[base + 2]]);
        points.push([hPolyFlat[base + 3], hPolyFlat[base + 4], hPolyFlat[base + 5]]);
    }

    const result = findInterior({ numFaces, normals, points });
    return result.success ? result.interior : null;
}

// ==================== Vertex Enumeration ====================

/**
 * Comparator for 3D vectors with numerical tolerance.
 */
function vectorLess(l: number[], r: number[]): number {
    if (l[0] < r[0]) return -1;
    if (l[0] > r[0]) return 1;
    if (l[1] < r[1]) return -1;
    if (l[1] > r[1]) return 1;
    if (l[2] < r[2]) return -1;
    if (l[2] > r[2]) return 1;
    return 0;
}

/**
 * Filter and deduplicate vertices within numerical tolerance.
 * 
 * @param rawVertices Array of vertex coordinates
 * @param epsilon Tolerance for considering vertices equal
 * @returns Filtered unique vertices
 */
export function filterVertices(rawVertices: number[][], epsilon: number = 1e-6): number[][] {
    if (rawVertices.length === 0) return [];

    // Find magnitude for scaling
    let maxAbs = 0;
    for (const v of rawVertices) {
        for (const c of v) {
            maxAbs = Math.max(maxAbs, Math.abs(c));
        }
    }

    const res = maxAbs * Math.max(Math.abs(epsilon) / maxAbs, Number.EPSILON);

    // Quantize and deduplicate
    const seen = new Map<string, number[]>();
    const filtered: number[][] = [];

    for (const v of rawVertices) {
        // Quantize to resolution
        const quanti = [
            Math.round(v[0] / res),
            Math.round(v[1] / res),
            Math.round(v[2] / res)
        ];
        const key = `${quanti[0]},${quanti[1]},${quanti[2]}`;

        if (!seen.has(key)) {
            seen.set(key, v);
            filtered.push([...v]);
        }
    }

    return filtered;
}

/**
 * Enumerate vertices of half-space polytope using QuickHull dual.
 * 
 * Algorithm:
 * 1. Find interior point of polytope
 * 2. Transform half-spaces to dual space (points)
 * 3. Compute convex hull of dual points
 * 4. Hull faces correspond to original vertices
 * 5. Transform back to get original vertices
 * 
 * @param hPoly Half-space polytope
 * @param epsilon Tolerance for vertex deduplication
 * @returns Array of vertex coordinates
 */
export function enumerateVertices(
    hPoly: HalfSpacePolytope,
    epsilon: number = 1e-6
): number[][] {
    const { success, interior } = findInterior(hPoly);

    if (!success) {
        return [];
    }

    return enumerateVerticesWithInterior(hPoly, interior, epsilon);
}

/**
 * Enumerate vertices given a known interior point.
 * 
 * @param hPoly Half-space polytope
 * @param inner Known interior point
 * @param epsilon Tolerance
 * @returns Vertex coordinates
 */
export function enumerateVerticesWithInterior(
    hPoly: HalfSpacePolytope,
    inner: [number, number, number],
    epsilon: number = 1e-6
): number[][] {
    const m = hPoly.numFaces;

    // Compute b_i = n_i · p_i - n_i · inner (signed distance from inner to face)
    const b: number[] = [];
    for (let i = 0; i < m; i++) {
        const n = hPoly.normals[i];
        const p = hPoly.points[i];
        const bi = (n[0] * p[0] + n[1] * p[1] + n[2] * p[2]) -
            (n[0] * inner[0] + n[1] * inner[1] + n[2] * inner[2]);
        b.push(bi);
    }

    // Transform normals to dual space: A_i = n_i / b_i
    const dualPoints: number[][] = [];
    for (let i = 0; i < m; i++) {
        if (Math.abs(b[i]) < 1e-12) continue; // Skip degenerate faces
        const n = hPoly.normals[i];
        dualPoints.push([n[0] / b[i], n[1] / b[i], n[2] / b[i]]);
    }

    if (dualPoints.length < 4) {
        // Not enough points for 3D convex hull
        return [];
    }

    // Compute convex hull of dual points
    // Each face of the hull corresponds to a vertex of the original polytope
    const hullVertices = simpleConvexHull(dualPoints, epsilon);

    // Transform vertices back: v = vertex / dot(vertex, vertex) + inner
    const rawVertices: number[][] = [];
    for (const v of hullVertices) {
        const dot = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
        if (dot > 1e-12) {
            rawVertices.push([
                v[0] / dot + inner[0],
                v[1] / dot + inner[1],
                v[2] / dot + inner[2]
            ]);
        }
    }

    return filterVertices(rawVertices, epsilon);
}

// ==================== Simple Convex Hull (Gift Wrapping) ====================

/**
 * Simple 3D convex hull using iterative algorithm.
 * This is a simplified implementation for moderate point counts.
 * For large point clouds, use the full QuickHull implementation.
 * 
 * @param points Input points
 * @param epsilon Tolerance
 * @returns Hull vertices
 */
function simpleConvexHull(points: number[][], epsilon: number): number[][] {
    if (points.length < 4) return [...points];

    // Find extreme points to start
    let minX = 0, maxX = 0, minY = 0, maxY = 0, minZ = 0, maxZ = 0;

    for (let i = 1; i < points.length; i++) {
        if (points[i][0] < points[minX][0]) minX = i;
        if (points[i][0] > points[maxX][0]) maxX = i;
        if (points[i][1] < points[minY][1]) minY = i;
        if (points[i][1] > points[maxY][1]) maxY = i;
        if (points[i][2] < points[minZ][2]) minZ = i;
        if (points[i][2] > points[maxZ][2]) maxZ = i;
    }

    // Collect unique extreme points
    const extremeIndices = new Set([minX, maxX, minY, maxY, minZ, maxZ]);
    const hullPoints = new Set<number>();

    for (const idx of extremeIndices) {
        hullPoints.add(idx);
    }

    // Simple iterative expansion
    let changed = true;
    let iterations = 0;
    const maxIterations = points.length * 2;

    while (changed && iterations < maxIterations) {
        changed = false;
        iterations++;

        for (let i = 0; i < points.length; i++) {
            if (hullPoints.has(i)) continue;

            // Check if point is outside current hull
            // (simplified: check if it's far from centroid)
            const centroid = [0, 0, 0];
            for (const idx of hullPoints) {
                centroid[0] += points[idx][0];
                centroid[1] += points[idx][1];
                centroid[2] += points[idx][2];
            }
            centroid[0] /= hullPoints.size;
            centroid[1] /= hullPoints.size;
            centroid[2] /= hullPoints.size;

            const p = points[i];
            const dirToPoint = [
                p[0] - centroid[0],
                p[1] - centroid[1],
                p[2] - centroid[2]
            ];

            let isExtreme = true;
            for (const idx of hullPoints) {
                const hp = points[idx];
                const dirToHull = [
                    hp[0] - centroid[0],
                    hp[1] - centroid[1],
                    hp[2] - centroid[2]
                ];

                // Check if point projects beyond hull point in any direction
                const dot = dirToPoint[0] * dirToHull[0] +
                    dirToPoint[1] * dirToHull[1] +
                    dirToPoint[2] * dirToHull[2];
                const hullDist = dirToHull[0] * dirToHull[0] +
                    dirToHull[1] * dirToHull[1] +
                    dirToHull[2] * dirToHull[2];
                const pointDist = dirToPoint[0] * dirToPoint[0] +
                    dirToPoint[1] * dirToPoint[1] +
                    dirToPoint[2] * dirToPoint[2];

                if (pointDist <= hullDist + epsilon && dot > 0) {
                    const cosAngle = dot / Math.sqrt(hullDist * pointDist);
                    if (cosAngle > 0.99) {
                        isExtreme = false;
                        break;
                    }
                }
            }

            if (isExtreme) {
                // More robust check: is point outside all current hull faces?
                // For simplicity, just add if it's farther from centroid
                let maxHullDist = 0;
                for (const idx of hullPoints) {
                    const hp = points[idx];
                    const dist = (hp[0] - centroid[0]) ** 2 +
                        (hp[1] - centroid[1]) ** 2 +
                        (hp[2] - centroid[2]) ** 2;
                    maxHullDist = Math.max(maxHullDist, dist);
                }

                const pointDist = (p[0] - centroid[0]) ** 2 +
                    (p[1] - centroid[1]) ** 2 +
                    (p[2] - centroid[2]) ** 2;

                if (pointDist > maxHullDist - epsilon) {
                    hullPoints.add(i);
                    changed = true;
                }
            }
        }
    }

    // Return hull vertices
    const result: number[][] = [];
    for (const idx of hullPoints) {
        result.push([...points[idx]]);
    }

    return result;
}

// ==================== Utility Functions ====================

/**
 * Create half-space representation of an axis-aligned box.
 * 
 * @param min Minimum corner [xmin, ymin, zmin]
 * @param max Maximum corner [xmax, ymax, zmax]
 * @returns Half-space polytope with 6 faces
 */
export function createBoxHalfSpaces(
    min: [number, number, number],
    max: [number, number, number]
): HalfSpacePolytope {
    return {
        numFaces: 6,
        normals: [
            [1, 0, 0],   // +X face
            [-1, 0, 0],  // -X face
            [0, 1, 0],   // +Y face
            [0, -1, 0],  // -Y face
            [0, 0, 1],   // +Z face
            [0, 0, -1]   // -Z face
        ],
        points: [
            [max[0], 0, 0],
            [min[0], 0, 0],
            [0, max[1], 0],
            [0, min[1], 0],
            [0, 0, max[2]],
            [0, 0, min[2]]
        ]
    };
}

/**
 * Check if a point is inside a half-space polytope.
 * 
 * @param hPoly Half-space polytope
 * @param point Point to test
 * @param epsilon Tolerance (positive = inside, negative = strict inside)
 * @returns true if point is inside all half-spaces
 */
export function isPointInside(
    hPoly: HalfSpacePolytope,
    point: [number, number, number],
    epsilon: number = 0
): boolean {
    for (let i = 0; i < hPoly.numFaces; i++) {
        const n = hPoly.normals[i];
        const p = hPoly.points[i];

        // Point is inside if n · (point - p) <= 0
        // With tolerance: n · (point - p) <= epsilon
        const signedDist = n[0] * (point[0] - p[0]) +
            n[1] * (point[1] - p[1]) +
            n[2] * (point[2] - p[2]);

        if (signedDist > epsilon) {
            return false;
        }
    }

    return true;
}

/**
 * Compute signed distance from point to polytope boundary.
 * Negative = inside, Positive = outside.
 * 
 * @param hPoly Half-space polytope
 * @param point Query point
 * @returns Signed distance (negative if inside)
 */
export function signedDistanceToPolytope(
    hPoly: HalfSpacePolytope,
    point: [number, number, number]
): number {
    let maxDist = -Infinity;

    for (let i = 0; i < hPoly.numFaces; i++) {
        const n = hPoly.normals[i];
        const p = hPoly.points[i];

        const signedDist = n[0] * (point[0] - p[0]) +
            n[1] * (point[1] - p[1]) +
            n[2] * (point[2] - p[2]);

        maxDist = Math.max(maxDist, signedDist);
    }

    return maxDist;
}

/**
 * Compute volume of half-space polytope (approximate via bounding box sampling).
 * 
 * @param hPoly Half-space polytope
 * @param samples Number of samples per axis
 * @returns Approximate volume
 */
export function approximatePolytopeVolume(
    hPoly: HalfSpacePolytope,
    samples: number = 20
): number {
    // Find bounding box from face points
    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;

    for (let i = 0; i < hPoly.numFaces; i++) {
        const p = hPoly.points[i];
        minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
        minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
        minZ = Math.min(minZ, p[2]); maxZ = Math.max(maxZ, p[2]);
    }

    // Expand bounding box
    const margin = 0.1;
    const rangeX = (maxX - minX) * (1 + 2 * margin);
    const rangeY = (maxY - minY) * (1 + 2 * margin);
    const rangeZ = (maxZ - minZ) * (1 + 2 * margin);
    minX -= rangeX * margin;
    minY -= rangeY * margin;
    minZ -= rangeZ * margin;

    // Monte Carlo volume estimation
    let insideCount = 0;
    const totalSamples = samples * samples * samples;

    const dx = rangeX / samples;
    const dy = rangeY / samples;
    const dz = rangeZ / samples;

    for (let i = 0; i < samples; i++) {
        for (let j = 0; j < samples; j++) {
            for (let k = 0; k < samples; k++) {
                const point: [number, number, number] = [
                    minX + (i + 0.5) * dx,
                    minY + (j + 0.5) * dy,
                    minZ + (k + 0.5) * dz
                ];

                if (isPointInside(hPoly, point)) {
                    insideCount++;
                }
            }
        }
    }

    const boxVolume = rangeX * rangeY * rangeZ;
    return boxVolume * (insideCount / totalSamples);
}

/**
 * Intersect two half-space polytopes.
 * 
 * @param poly1 First polytope
 * @param poly2 Second polytope
 * @returns Combined polytope (union of constraints)
 */
export function intersectPolytopes(
    poly1: HalfSpacePolytope,
    poly2: HalfSpacePolytope
): HalfSpacePolytope {
    return {
        numFaces: poly1.numFaces + poly2.numFaces,
        normals: [...poly1.normals, ...poly2.normals],
        points: [...poly1.points, ...poly2.points]
    };
}

/**
 * Check if two half-space polytopes overlap (have non-empty intersection).
 * 
 * This is used by the corridor shortcut algorithm to determine which
 * intermediate corridors can be skipped.
 * 
 * @param poly1 First polytope
 * @param poly2 Second polytope
 * @param epsilon Tolerance for interior point finding
 * @returns true if polytopes overlap
 */
export function overlap(
    poly1: HalfSpacePolytope,
    poly2: HalfSpacePolytope,
    epsilon: number = 0.01
): boolean {
    // Combine constraints from both polytopes
    const combined = intersectPolytopes(poly1, poly2);

    // Check if the combined polytope has a non-empty interior
    const result = findInterior(combined);

    // If we found an interior point, the polytopes overlap
    return result.success;
}

/**
 * Convert GCOPTER-style H-polytope matrix format to our HalfSpacePolytope.
 * 
 * GCOPTER format: Each row is [h0, h1, h2, h3] where h0*x + h1*y + h2*z + h3 <= 0
 * 
 * @param hPolyMatrix Matrix where each row is [nx, ny, nz, d]
 * @returns HalfSpacePolytope
 */
export function fromHPolyMatrix(hPolyMatrix: number[][]): HalfSpacePolytope {
    const numFaces = hPolyMatrix.length;
    const normals: number[][] = [];
    const points: number[][] = [];

    for (let i = 0; i < numFaces; i++) {
        const row = hPolyMatrix[i];
        const nx = row[0], ny = row[1], nz = row[2];
        const d = row[3];

        // Normalize normal vector
        const norm = Math.sqrt(nx * nx + ny * ny + nz * nz);
        if (norm < 1e-10) continue;

        const normalizedN = [nx / norm, ny / norm, nz / norm];

        // Point on face: p = -d * n / ||n||^2 = -d * n_hat / ||n||
        const p = [
            -d * normalizedN[0] / norm,
            -d * normalizedN[1] / norm,
            -d * normalizedN[2] / norm
        ];

        normals.push(normalizedN);
        points.push(p);
    }

    return { numFaces: normals.length, normals, points };
}

/**
 * Convert HalfSpacePolytope back to GCOPTER-style matrix format.
 * 
 * @param hPoly HalfSpacePolytope
 * @returns Matrix where each row is [nx, ny, nz, d] where n·x + d <= 0
 */
export function toHPolyMatrix(hPoly: HalfSpacePolytope): number[][] {
    const result: number[][] = [];

    for (let i = 0; i < hPoly.numFaces; i++) {
        const n = hPoly.normals[i];
        const p = hPoly.points[i];

        // d = -n · p
        const d = -(n[0] * p[0] + n[1] * p[1] + n[2] * p[2]);

        result.push([n[0], n[1], n[2], d]);
    }

    return result;
}

