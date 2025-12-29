/**
 * @module planning/corridor
 * @description Safe Flight Corridor generation algorithms.
 * 
 * Provides two approaches:
 * 1. FIRI (Fast Iterative Region Inflation) - obstacle-aware corridors
 * 2. OBB + Spherical Buffer - deterministic segment-aligned corridors
 * 
 * Reference: Zhepei Wang et al., "Geometrically Constrained Trajectory Optimization
 * for Multicopters", IEEE Transactions on Robotics, 2022.
 */

import {
    Vec3, Mat3,
    dot, norm, zeros, zerosMatrix,
    identity3, zeroMat3, cholesky3, svd3,
    mulMatVec3, mulMat3, transpose3, det3
} from '../../numeric/math/linear-algebra';
import { lbfgsOptimize, defaultLBFGSParameters } from '../../numeric/optimization/lbfgs';
import { linprog4, SDLPStatus } from '../../numeric/optimization/sdlp';
import type { HPolyhedron, Ellipsoid } from './types';
import type { Vector3Array } from '../trajectory/types';

const DBL_EPSILON = 2.2204460492503131e-16;

// ==================== OBB Corridor Configuration ====================

/**
 * Configuration for OBB-based corridor generation
 * 
 * ## Corridor Size Formula (lateral/vertical)
 * 
 * halfWidth = r_drone + r_state + e_track + r_margin
 * halfHeight = r_drone_z + r_state_z + e_track_z + r_margin_z
 * 
 * Where:
 * - r_drone: Drone lateral geometric envelope radius (m)
 * - r_state: Position uncertainty margin from localization/mapping (m)
 * - e_track: Trajectory tracking error upper bound (m)
 * - r_margin: Safety buffer for unmodeled factors (m)
 * 
 * ## Waypoint Sphere Formula (corner buffer)
 * 
 * r_wp = baseRadius * (1 + α * θ/π)
 * 
 * Where:
 * - baseRadius: Base buffer radius at turns (typically >= halfWidth)
 * - α: Radius growth factor (radiusGrowthFactor)
 * - θ: Turning angle in radians (0 to π, larger = sharper turn)
 * 
 * ## Example Configuration for 0.6m diameter drone:
 * - halfWidth: 0.3 (r_drone) + 0.2 (r_state) + 0.2 (margin) ≈ 0.7m
 * - halfHeight: 0.5-0.7m (height control typically more stable)
 * - waypointRadius: 0.8-1.2m (at least >= halfWidth)
 * - radiusGrowthFactor: 0.5-1.0 (0.8 recommended)
 * - sphereFaces: 12 (debug), 20-32 (production)
 */
export interface OBBCorridorOptions {
    /** 
     * Lateral clearance (meters) - half width of corridor cross-section
     * Formula: halfWidth = r_drone + r_state + e_track + r_margin
     * Typical: 0.5-2.0m for small drones, 20-50m for city-scale simulation
     */
    halfWidth: number;

    /** 
     * Vertical clearance (meters) - half height of corridor cross-section
     * Formula: halfHeight = r_drone_z + r_state_z + e_track_z + r_margin_z
     * Typical: 0.5-1.5m for small drones, 15-30m for city-scale simulation
     */
    halfHeight: number;

    /** 
     * Base radius for waypoint spherical buffer (meters)
     * Should be >= halfWidth to ensure corridor continuity at turns
     * Typical: 1.0-2.0 * halfWidth
     */
    waypointRadius: number;

    /** 
     * Factor to increase radius at turns: r = base * (1 + factor * angle/π)
     * Higher values give more room for sharp turns
     * Typical: 0.5-1.0 (0.8 recommended)
     */
    radiusGrowthFactor: number;

    /** 
     * Number of faces for sphere approximation
     * - 12: Icosahedron (fast, debug)
     * - 20: Dodecahedron (balanced)
     * - 32: Geodesic sphere (accurate, production)
     */
    sphereFaces: 12 | 20 | 32;

    /** 
     * Extension beyond segment endpoints for overlap (meters)
     * Ensures adjacent corridors overlap to avoid gaps
     * Typical: 0.3-1.0m for small drones, 5-20m for city-scale
     */
    segmentOverlap: number;
}

/**
 * Default configuration for typical small drone (0.6m diameter)
 * 
 * Assumptions:
 * - r_drone = 0.3m (drone radius)
 * - r_state = 0.2m (VIO/SLAM uncertainty)
 * - e_track = 0.1m (control tracking error)
 * - r_margin = 0.2m (safety buffer)
 * 
 * halfWidth = 0.3 + 0.2 + 0.1 + 0.2 = 0.8m
 * halfHeight = 0.3 + 0.15 + 0.1 + 0.15 = 0.7m
 */
const DEFAULT_OBB_OPTIONS: OBBCorridorOptions = {
    halfWidth: 0.8,         // r_drone(0.3) + r_state(0.2) + e_track(0.1) + margin(0.2)
    halfHeight: 0.7,        // Vertical clearance (height control typically more stable)
    waypointRadius: 1.0,    // >= halfWidth for smooth transitions
    radiusGrowthFactor: 0.8, // Moderate increase at sharp turns
    sphereFaces: 20,        // Balanced accuracy/performance
    segmentOverlap: 0.5     // 50cm overlap for continuity
};

// ==================== Sphere Approximation Vertices ====================

// Icosahedron vertices (12 faces) - normalized
const ICOSAHEDRON_VERTICES: Vec3[] = (() => {
    const phi = (1 + Math.sqrt(5)) / 2; // Golden ratio
    const a = 1 / Math.sqrt(1 + phi * phi);
    const b = phi * a;
    return [
        [0, a, b], [0, a, -b], [0, -a, b], [0, -a, -b],
        [a, b, 0], [a, -b, 0], [-a, b, 0], [-a, -b, 0],
        [b, 0, a], [b, 0, -a], [-b, 0, a], [-b, 0, -a]
    ];
})();

// Dodecahedron vertices (20 faces) - normalized
const DODECAHEDRON_VERTICES: Vec3[] = (() => {
    const phi = (1 + Math.sqrt(5)) / 2;
    const a = 1 / Math.sqrt(3);
    const b = a / phi;
    const c = a * phi;
    const verts: Vec3[] = [];
    // Cube vertices
    for (let i = -1; i <= 1; i += 2) {
        for (let j = -1; j <= 1; j += 2) {
            for (let k = -1; k <= 1; k += 2) {
                verts.push([i * a, j * a, k * a]);
            }
        }
    }
    // Rectangle vertices in 3 planes
    for (let i = -1; i <= 1; i += 2) {
        for (let j = -1; j <= 1; j += 2) {
            verts.push([0, i * b, j * c]);
            verts.push([i * b, j * c, 0]);
            verts.push([i * c, 0, j * b]);
        }
    }
    return verts;
})();

// Geodesic sphere (32 faces) - icosahedron with one subdivision level
const GEODESIC_32_NORMALS: Vec3[] = (() => {
    const normals: Vec3[] = [...ICOSAHEDRON_VERTICES];
    // Add midpoints of icosahedron edges (20 additional normals)
    const edges: [number, number][] = [
        [0, 2], [0, 4], [0, 6], [0, 8], [0, 10],
        [1, 3], [1, 4], [1, 6], [1, 9], [1, 11],
        [2, 5], [2, 7], [2, 8], [2, 10],
        [3, 5], [3, 7], [3, 9], [3, 11],
        [4, 8], [4, 9], [5, 8], [5, 9],
        [6, 10], [6, 11], [7, 10], [7, 11]
    ];
    const seen = new Set<string>();
    for (const [i, j] of edges) {
        const key = `${Math.min(i, j)}-${Math.max(i, j)}`;
        if (seen.has(key)) continue;
        seen.add(key);
        const v1 = ICOSAHEDRON_VERTICES[i];
        const v2 = ICOSAHEDRON_VERTICES[j];
        const mid: Vec3 = [
            (v1[0] + v2[0]) / 2,
            (v1[1] + v2[1]) / 2,
            (v1[2] + v2[2]) / 2
        ];
        const len = Math.sqrt(mid[0] ** 2 + mid[1] ** 2 + mid[2] ** 2);
        normals.push([mid[0] / len, mid[1] / len, mid[2] / len]);
    }
    return normals.slice(0, 32);
})();

// ==================== OBB Corridor Helpers ====================

/**
 * Build orthonormal local frame from a direction vector
 * x_local = normalized direction, y_local and z_local are perpendicular
 */
function buildLocalFrame(direction: Vec3): { x: Vec3; y: Vec3; z: Vec3 } {
    const len = Math.sqrt(direction[0] ** 2 + direction[1] ** 2 + direction[2] ** 2);
    if (len < 1e-10) {
        return { x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1] };
    }

    const x: Vec3 = [direction[0] / len, direction[1] / len, direction[2] / len];

    // Find a vector not parallel to x
    let up: Vec3 = [0, 0, 1];
    if (Math.abs(x[2]) > 0.9) {
        up = [0, 1, 0];
    }

    // z = x × up (normalized)
    let z: Vec3 = [
        x[1] * up[2] - x[2] * up[1],
        x[2] * up[0] - x[0] * up[2],
        x[0] * up[1] - x[1] * up[0]
    ];
    const zLen = Math.sqrt(z[0] ** 2 + z[1] ** 2 + z[2] ** 2);
    z = [z[0] / zLen, z[1] / zLen, z[2] / zLen];

    // y = z × x
    const y: Vec3 = [
        z[1] * x[2] - z[2] * x[1],
        z[2] * x[0] - z[0] * x[2],
        z[0] * x[1] - z[1] * x[0]
    ];

    return { x, y, z };
}

/**
 * Compute angle between two vectors in radians
 */
function vectorAngle(v1: Vec3, v2: Vec3): number {
    const len1 = Math.sqrt(v1[0] ** 2 + v1[1] ** 2 + v1[2] ** 2);
    const len2 = Math.sqrt(v2[0] ** 2 + v2[1] ** 2 + v2[2] ** 2);
    if (len1 < 1e-10 || len2 < 1e-10) return 0;

    const dot = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
    const cosAngle = Math.max(-1, Math.min(1, dot / (len1 * len2)));
    return Math.acos(cosAngle);
}

/**
 * Generate half-planes for a sphere approximation
 * Each half-plane: n·(p - center) ≤ radius → [n_x, n_y, n_z, -n·center - radius]
 */
function approximateSphere(center: Vec3, radius: number, faces: 12 | 20 | 32): number[][] {
    let normals: Vec3[];
    switch (faces) {
        case 12:
            normals = ICOSAHEDRON_VERTICES;
            break;
        case 20:
            normals = DODECAHEDRON_VERTICES;
            break;
        case 32:
            normals = GEODESIC_32_NORMALS;
            break;
        default:
            normals = ICOSAHEDRON_VERTICES;
    }

    const planes: number[][] = [];
    for (const n of normals) {
        // Half-plane: n·x + d ≤ 0 where d = -n·center - radius
        const d = -(n[0] * center[0] + n[1] * center[1] + n[2] * center[2]) - radius;
        planes.push([n[0], n[1], n[2], d]);
    }
    return planes;
}

/**
 * Generate OBB half-planes for a path segment
 * Returns 6 half-planes representing an oriented bounding box
 */
function generateSegmentOBB(
    start: Vec3,
    end: Vec3,
    frame: { x: Vec3; y: Vec3; z: Vec3 },
    halfWidth: number,
    halfHeight: number,
    overlap: number
): number[][] {
    const planes: number[][] = [];

    // Segment center and length
    const center: Vec3 = [
        (start[0] + end[0]) / 2,
        (start[1] + end[1]) / 2,
        (start[2] + end[2]) / 2
    ];
    const segLen = Math.sqrt(
        (end[0] - start[0]) ** 2 +
        (end[1] - start[1]) ** 2 +
        (end[2] - start[2]) ** 2
    );
    const halfLength = segLen / 2 + overlap;

    // +x direction (end of segment)
    planes.push([
        frame.x[0], frame.x[1], frame.x[2],
        -(frame.x[0] * center[0] + frame.x[1] * center[1] + frame.x[2] * center[2]) - halfLength
    ]);
    // -x direction (start of segment)
    planes.push([
        -frame.x[0], -frame.x[1], -frame.x[2],
        (frame.x[0] * center[0] + frame.x[1] * center[1] + frame.x[2] * center[2]) - halfLength
    ]);

    // +y direction
    planes.push([
        frame.y[0], frame.y[1], frame.y[2],
        -(frame.y[0] * center[0] + frame.y[1] * center[1] + frame.y[2] * center[2]) - halfWidth
    ]);
    // -y direction
    planes.push([
        -frame.y[0], -frame.y[1], -frame.y[2],
        (frame.y[0] * center[0] + frame.y[1] * center[1] + frame.y[2] * center[2]) - halfWidth
    ]);

    // +z direction (up)
    planes.push([
        frame.z[0], frame.z[1], frame.z[2],
        -(frame.z[0] * center[0] + frame.z[1] * center[1] + frame.z[2] * center[2]) - halfHeight
    ]);
    // -z direction (down)
    planes.push([
        -frame.z[0], -frame.z[1], -frame.z[2],
        (frame.z[0] * center[0] + frame.z[1] * center[1] + frame.z[2] * center[2]) - halfHeight
    ]);

    return planes;
}

// ==================== Main OBB Corridor Generation ====================

/**
 * Generate OBB-based Safe Flight Corridors with spherical waypoint buffers
 * 
 * Each segment corridor consists of:
 * - An OBB aligned with the segment direction
 * - Spherical buffers at start and end waypoints (intersection)
 * 
 * @param path Sequence of waypoints
 * @param options Corridor configuration
 * @returns Array of H-polyhedra (one per segment)
 */
export function generateOBBCorridor(
    path: Vector3Array[],
    options?: Partial<OBBCorridorOptions>
): HPolyhedron[] {
    const opts = { ...DEFAULT_OBB_OPTIONS, ...options };
    const corridors: HPolyhedron[] = [];

    if (path.length < 2) return corridors;

    // Precompute turning angles for waypoint radius adjustment
    // θ = angle between consecutive segments (0 = straight, π = U-turn)
    // Using θ directly so sharper turns get larger buffers
    const turningAngles: number[] = new Array(path.length).fill(0);
    for (let i = 1; i < path.length - 1; i++) {
        const v1: Vec3 = [
            path[i][0] - path[i - 1][0],
            path[i][1] - path[i - 1][1],
            path[i][2] - path[i - 1][2]
        ];
        const v2: Vec3 = [
            path[i + 1][0] - path[i][0],
            path[i + 1][1] - path[i][1],
            path[i + 1][2] - path[i][2]
        ];
        // Use angle directly: φ = arccos(v1·v2 / |v1||v2|)
        // θ = φ means: straight line → θ≈0, sharp turn → θ≈π
        turningAngles[i] = vectorAngle(v1, v2);
    }

    // Generate corridor for each segment
    for (let i = 0; i < path.length - 1; i++) {
        const start: Vec3 = [path[i][0], path[i][1], path[i][2]];
        const end: Vec3 = [path[i + 1][0], path[i + 1][1], path[i + 1][2]];

        // Segment direction and local frame
        const direction: Vec3 = [
            end[0] - start[0],
            end[1] - start[1],
            end[2] - start[2]
        ];
        const frame = buildLocalFrame(direction);

        // Generate OBB half-planes
        const obbPlanes = generateSegmentOBB(
            start, end, frame,
            opts.halfWidth, opts.halfHeight, opts.segmentOverlap
        );

        // Compute waypoint radii (adjusted for turning)
        const startRadius = opts.waypointRadius *
            (1 + opts.radiusGrowthFactor * turningAngles[i] / Math.PI);
        const endRadius = opts.waypointRadius *
            (1 + opts.radiusGrowthFactor * turningAngles[i + 1] / Math.PI);

        // Generate sphere approximations at waypoints
        const startSphere = approximateSphere(start, startRadius, opts.sphereFaces);
        const endSphere = approximateSphere(end, endRadius, opts.sphereFaces);

        // Combine all constraints: OBB ∩ StartSphere ∩ EndSphere
        const corridor: HPolyhedron = [
            ...obbPlanes,
            ...startSphere,
            ...endSphere
        ];

        corridors.push(corridor);
    }

    return corridors;
}

/**
 * Export the default options for external configuration
 */
export { DEFAULT_OBB_OPTIONS };



// ==================== Smoothed L1 Penalty ====================

function smoothedL1(mu: number, x: number): { f: number; df: number } | null {
    if (x < 0) {
        return null;
    } else if (x > mu) {
        return { f: x - 0.5 * mu, df: 1.0 };
    } else {
        const xdmu = x / mu;
        const sqrxdmu = xdmu * xdmu;
        const mumxd2 = mu - 0.5 * x;
        const f = mumxd2 * sqrxdmu * xdmu;
        const df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
        return { f, df };
    }
}

// ==================== Maximum Volume Inscribed Ellipsoid ====================

interface MVIEData {
    M: number;
    smoothEps: number;
    penaltyWt: number;
    A: number[][]; // M x 3
}

function costMVIE(x: number[], data: MVIEData): { cost: number; gradient: number[] } {
    const M = data.M;
    const smoothEps = data.smoothEps;
    const penaltyWt = data.penaltyWt;
    const A = data.A;

    // Unpack variables
    const p: Vec3 = [x[0], x[1], x[2]];
    const rtd: Vec3 = [x[3], x[4], x[5]];
    const cde: Vec3 = [x[6], x[7], x[8]];

    // Gradient accumulators
    const gdp: Vec3 = [0, 0, 0];
    const gdrtd: Vec3 = [0, 0, 0];
    const gdcde: Vec3 = [0, 0, 0];

    // Build L matrix (lower triangular Cholesky factor)
    const L: Mat3 = [
        [rtd[0] * rtd[0] + DBL_EPSILON, 0, 0],
        [cde[0], rtd[1] * rtd[1] + DBL_EPSILON, 0],
        [cde[2], cde[1], rtd[2] * rtd[2] + DBL_EPSILON]
    ];

    // Compute A * L
    const AL: number[][] = [];
    for (let i = 0; i < M; i++) {
        const row: Vec3 = [
            A[i][0] * L[0][0] + A[i][1] * L[1][0] + A[i][2] * L[2][0],
            A[i][0] * L[0][1] + A[i][1] * L[1][1] + A[i][2] * L[2][1],
            A[i][0] * L[0][2] + A[i][1] * L[1][2] + A[i][2] * L[2][2]
        ];
        AL.push(row);
    }

    // Row norms of AL
    const normAL: number[] = AL.map(row => Math.sqrt(row[0] * row[0] + row[1] * row[1] + row[2] * row[2]));

    // Adjusted normals
    const adjNormAL: number[][] = [];
    for (let i = 0; i < M; i++) {
        const n = normAL[i] > 1e-12 ? normAL[i] : 1;
        adjNormAL.push([AL[i][0] / n, AL[i][1] / n, AL[i][2] / n]);
    }

    // Constraint violation: normAL + A*p - 1
    const consViola: number[] = [];
    for (let i = 0; i < M; i++) {
        const Ap = A[i][0] * p[0] + A[i][1] * p[1] + A[i][2] * p[2];
        consViola.push(normAL[i] + Ap - 1);
    }

    // Accumulate cost and gradients
    let cost = 0;
    for (let i = 0; i < M; i++) {
        const sl = smoothedL1(smoothEps, consViola[i]);
        if (sl) {
            cost += sl.f;
            const vec: Vec3 = [sl.df * A[i][0], sl.df * A[i][1], sl.df * A[i][2]];

            gdp[0] += vec[0];
            gdp[1] += vec[1];
            gdp[2] += vec[2];

            gdrtd[0] += adjNormAL[i][0] * vec[0];
            gdrtd[1] += adjNormAL[i][1] * vec[1];
            gdrtd[2] += adjNormAL[i][2] * vec[2];

            gdcde[0] += adjNormAL[i][0] * vec[1];
            gdcde[1] += adjNormAL[i][1] * vec[2];
            gdcde[2] += adjNormAL[i][0] * vec[2];
        }
    }

    cost *= penaltyWt;
    for (let i = 0; i < 3; i++) {
        gdp[i] *= penaltyWt;
        gdrtd[i] *= penaltyWt;
        gdcde[i] *= penaltyWt;
    }

    // Log-barrier for volume
    cost -= Math.log(L[0][0]) + Math.log(L[1][1]) + Math.log(L[2][2]);
    gdrtd[0] -= 1.0 / L[0][0];
    gdrtd[1] -= 1.0 / L[1][1];
    gdrtd[2] -= 1.0 / L[2][2];

    // Chain rule for rtd
    gdrtd[0] *= 2.0 * rtd[0];
    gdrtd[1] *= 2.0 * rtd[1];
    gdrtd[2] *= 2.0 * rtd[2];

    const gradient = [
        gdp[0], gdp[1], gdp[2],
        gdrtd[0], gdrtd[1], gdrtd[2],
        gdcde[0], gdcde[1], gdcde[2]
    ];

    return { cost, gradient };
}

/**
 * Find maximum volume inscribed ellipsoid (MVIE) within a polyhedron
 */
function maxVolInsEllipsoid(
    hPoly: HPolyhedron,
    initialEllipsoid?: Ellipsoid
): Ellipsoid | null {
    const M = hPoly.length;

    // Find deepest interior point using LP
    const Alp: number[][] = [];
    const blp: number[] = [];
    const hNorm: number[] = [];

    for (let i = 0; i < M; i++) {
        const row = hPoly[i];
        const n = Math.sqrt(row[0] * row[0] + row[1] * row[1] + row[2] * row[2]);
        hNorm.push(n);
        Alp.push([row[0] / n, row[1] / n, row[2] / n, 1]);
        blp.push(-row[3] / n);
    }

    const clp: [number, number, number, number] = [0, 0, 0, -1];
    const lpResult = linprog4(clp, Alp, blp);

    const maxdepth = -lpResult.minimum;
    if (!(maxdepth > 0) || !Number.isFinite(maxdepth)) {
        return null;
    }

    const interior: Vec3 = [lpResult.x[0], lpResult.x[1], lpResult.x[2]];

    // Prepare MVIE optimization data
    const A: number[][] = [];
    for (let i = 0; i < M; i++) {
        const scale = blp[i] - (Alp[i][0] * interior[0] + Alp[i][1] * interior[1] + Alp[i][2] * interior[2]);
        A.push([Alp[i][0] / scale, Alp[i][1] / scale, Alp[i][2] / scale]);
    }

    const data: MVIEData = {
        M,
        smoothEps: 1e-2,
        penaltyWt: 1e3,
        A
    };

    // Initial guess
    let R = identity3();
    let p: Vec3 = [...interior] as Vec3;
    let r: Vec3 = [1, 1, 1];

    if (initialEllipsoid) {
        R = initialEllipsoid.rotation;
        p = [...initialEllipsoid.center] as Vec3;
        r = [...initialEllipsoid.radii] as Vec3;
    }

    // Compute initial Q = R * diag(r^2) * R^T
    const rSq: Vec3 = [r[0] * r[0], r[1] * r[1], r[2] * r[2]];
    const Q: Mat3 = [
        [R[0][0] * rSq[0] * R[0][0] + R[0][1] * rSq[1] * R[0][1] + R[0][2] * rSq[2] * R[0][2],
        R[0][0] * rSq[0] * R[1][0] + R[0][1] * rSq[1] * R[1][1] + R[0][2] * rSq[2] * R[1][2],
        R[0][0] * rSq[0] * R[2][0] + R[0][1] * rSq[1] * R[2][1] + R[0][2] * rSq[2] * R[2][2]],
        [R[1][0] * rSq[0] * R[0][0] + R[1][1] * rSq[1] * R[0][1] + R[1][2] * rSq[2] * R[0][2],
        R[1][0] * rSq[0] * R[1][0] + R[1][1] * rSq[1] * R[1][1] + R[1][2] * rSq[2] * R[1][2],
        R[1][0] * rSq[0] * R[2][0] + R[1][1] * rSq[1] * R[2][1] + R[1][2] * rSq[2] * R[2][2]],
        [R[2][0] * rSq[0] * R[0][0] + R[2][1] * rSq[1] * R[0][1] + R[2][2] * rSq[2] * R[0][2],
        R[2][0] * rSq[0] * R[1][0] + R[2][1] * rSq[1] * R[1][1] + R[2][2] * rSq[2] * R[1][2],
        R[2][0] * rSq[0] * R[2][0] + R[2][1] * rSq[1] * R[2][1] + R[2][2] * rSq[2] * R[2][2]]
    ];
    const L = cholesky3(Q);

    // Pack optimization variables
    const x0 = [
        p[0] - interior[0], p[1] - interior[1], p[2] - interior[2],
        Math.sqrt(L[0][0]), Math.sqrt(L[1][1]), Math.sqrt(L[2][2]),
        L[1][0], L[2][1], L[2][0]
    ];

    // Optimize
    const params = {
        ...defaultLBFGSParameters(),
        memSize: 18,
        gEpsilon: 0,
        minStep: 1e-32,
        past: 3,
        delta: 1e-7
    };

    const result = lbfgsOptimize(
        x0,
        (x) => costMVIE(x, data),
        params
    );

    if (result.status < 0) {
        console.warn('MVIE optimization warning:', result.status);
    }

    const xOpt = result.x;

    // Unpack result
    p = [xOpt[0] + interior[0], xOpt[1] + interior[1], xOpt[2] + interior[2]];
    const Lopt: Mat3 = [
        [xOpt[3] * xOpt[3], 0, 0],
        [xOpt[6], xOpt[4] * xOpt[4], 0],
        [xOpt[8], xOpt[7], xOpt[5] * xOpt[5]]
    ];

    // SVD of L to get R and r
    const svdResult = svd3(Lopt);
    const U = svdResult.U;
    const S = svdResult.S;

    // Ensure proper rotation (det = 1)
    if (det3(U) < 0) {
        R = [
            [U[0][1], U[0][0], U[0][2]],
            [U[1][1], U[1][0], U[1][2]],
            [U[2][1], U[2][0], U[2][2]]
        ];
        r = [S[1], S[0], S[2]];
    } else {
        R = U;
        r = S;
    }

    return {
        center: p,
        rotation: R,
        radii: r
    };
}

// ==================== Main FIRI Algorithm ====================

/**
 * Fast Iterative Region Inflation (FIRI)
 * 
 * Generates a convex polytope (H-representation) that:
 * 1. Contains the line segment from point a to point b
 * 2. Excludes all obstacle points in the point cloud
 * 3. Stays within the bounding box constraints
 * 
 * @param boundingBox H-representation of bounding constraints (Mx4)
 * @param obstacles Point cloud of obstacles (3xN)
 * @param a Start point of the path segment
 * @param b End point of the path segment
 * @param iterations Number of refinement iterations (default: 4)
 * @param epsilon Tolerance (default: 1e-6)
 * @returns H-representation of the inflated polytope, or null if infeasible
 */
export function inflateCorridorFIRI(
    boundingBox: HPolyhedron,
    obstacles: Vector3Array[],
    a: Vector3Array,
    b: Vector3Array,
    iterations: number = 4,
    epsilon: number = 1e-6
): HPolyhedron | null {
    const bd = boundingBox;
    const pc = obstacles;

    // Check if a and b are inside the bounding box
    const ah = [a[0], a[1], a[2], 1];
    const bh = [b[0], b[1], b[2], 1];

    for (const row of bd) {
        if (row[0] * ah[0] + row[1] * ah[1] + row[2] * ah[2] + row[3] > 0) {
            return null;
        }
        if (row[0] * bh[0] + row[1] * bh[1] + row[2] * bh[2] + row[3] > 0) {
            return null;
        }
    }

    const M = bd.length;
    const N = pc.length;

    // Initialize ellipsoid
    let R = identity3();
    let p: Vec3 = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
    let r: Vec3 = [1, 1, 1];

    let hPoly: HPolyhedron = [];

    for (let loop = 0; loop < iterations; loop++) {
        // Forward transformation
        const forward: Mat3 = [
            [1 / r[0] * R[0][0], 1 / r[0] * R[1][0], 1 / r[0] * R[2][0]],
            [1 / r[1] * R[0][1], 1 / r[1] * R[1][1], 1 / r[1] * R[2][1]],
            [1 / r[2] * R[0][2], 1 / r[2] * R[1][2], 1 / r[2] * R[2][2]]
        ];
        const backward: Mat3 = [
            [R[0][0] * r[0], R[0][1] * r[1], R[0][2] * r[2]],
            [R[1][0] * r[0], R[1][1] * r[1], R[1][2] * r[2]],
            [R[2][0] * r[0], R[2][1] * r[1], R[2][2] * r[2]]
        ];

        // Transform bounding box
        const forwardB: number[][] = [];
        const forwardD: number[] = [];
        for (let i = 0; i < M; i++) {
            const n = bd[i];
            forwardB.push([
                n[0] * backward[0][0] + n[1] * backward[1][0] + n[2] * backward[2][0],
                n[0] * backward[0][1] + n[1] * backward[1][1] + n[2] * backward[2][1],
                n[0] * backward[0][2] + n[1] * backward[1][2] + n[2] * backward[2][2]
            ]);
            forwardD.push(n[3] + n[0] * p[0] + n[1] * p[1] + n[2] * p[2]);
        }

        // Transform point cloud
        const forwardPC: Vec3[] = [];
        for (let i = 0; i < N; i++) {
            const pt = pc[i];
            const shifted: Vec3 = [pt[0] - p[0], pt[1] - p[1], pt[2] - p[2]];
            forwardPC.push(mulMatVec3(forward, shifted));
        }

        // Transform endpoints
        const fwdA = mulMatVec3(forward, [a[0] - p[0], a[1] - p[1], a[2] - p[2]]);
        const fwdB = mulMatVec3(forward, [b[0] - p[0], b[1] - p[1], b[2] - p[2]]);

        // Compute distances
        const distDs: number[] = forwardD.map((d, i) => {
            const rowNorm = Math.sqrt(forwardB[i][0] ** 2 + forwardB[i][1] ** 2 + forwardB[i][2] ** 2);
            return Math.abs(d) / (rowNorm > 1e-12 ? rowNorm : 1);
        });

        // Compute tangent planes for obstacles
        const tangents: number[][] = [];
        const distRs: number[] = [];

        for (let i = 0; i < N; i++) {
            const pt = forwardPC[i];
            const dist = Math.sqrt(pt[0] ** 2 + pt[1] ** 2 + pt[2] ** 2);
            distRs.push(dist);

            // Initial tangent plane at point
            let normal: Vec3 = dist > 1e-12 ? [pt[0] / dist, pt[1] / dist, pt[2] / dist] : [1, 0, 0];
            let d = -dist;

            // Check if tangent violates endpoint a
            if (normal[0] * fwdA[0] + normal[1] * fwdA[1] + normal[2] * fwdA[2] + d > epsilon) {
                // Project to ensure a is on the correct side
                const delta: Vec3 = [pt[0] - fwdA[0], pt[1] - fwdA[1], pt[2] - fwdA[2]];
                const dotDeltaA = delta[0] * fwdA[0] + delta[1] * fwdA[1] + delta[2] * fwdA[2];
                const deltaSq = delta[0] ** 2 + delta[1] ** 2 + delta[2] ** 2;
                if (deltaSq > 1e-12) {
                    const proj: Vec3 = [
                        fwdA[0] - (dotDeltaA / deltaSq) * delta[0],
                        fwdA[1] - (dotDeltaA / deltaSq) * delta[1],
                        fwdA[2] - (dotDeltaA / deltaSq) * delta[2]
                    ];
                    const projDist = Math.sqrt(proj[0] ** 2 + proj[1] ** 2 + proj[2] ** 2);
                    if (projDist > 1e-12) {
                        distRs[i] = projDist;
                        d = -projDist;
                        normal = [proj[0] / projDist, proj[1] / projDist, proj[2] / projDist];
                    }
                }
            }

            // Check if tangent violates endpoint b
            if (normal[0] * fwdB[0] + normal[1] * fwdB[1] + normal[2] * fwdB[2] + d > epsilon) {
                const delta: Vec3 = [pt[0] - fwdB[0], pt[1] - fwdB[1], pt[2] - fwdB[2]];
                const dotDeltaB = delta[0] * fwdB[0] + delta[1] * fwdB[1] + delta[2] * fwdB[2];
                const deltaSq = delta[0] ** 2 + delta[1] ** 2 + delta[2] ** 2;
                if (deltaSq > 1e-12) {
                    const proj: Vec3 = [
                        fwdB[0] - (dotDeltaB / deltaSq) * delta[0],
                        fwdB[1] - (dotDeltaB / deltaSq) * delta[1],
                        fwdB[2] - (dotDeltaB / deltaSq) * delta[2]
                    ];
                    const projDist = Math.sqrt(proj[0] ** 2 + proj[1] ** 2 + proj[2] ** 2);
                    if (projDist > 1e-12) {
                        distRs[i] = projDist;
                        d = -projDist;
                        normal = [proj[0] / projDist, proj[1] / projDist, proj[2] / projDist];
                    }
                }
            }

            // Final check: if still violating a, use plane through a, b, point
            if (normal[0] * fwdA[0] + normal[1] * fwdA[1] + normal[2] * fwdA[2] + d > epsilon) {
                const va: Vec3 = [fwdA[0] - pt[0], fwdA[1] - pt[1], fwdA[2] - pt[2]];
                const vb: Vec3 = [fwdB[0] - pt[0], fwdB[1] - pt[1], fwdB[2] - pt[2]];
                const cross: Vec3 = [
                    va[1] * vb[2] - va[2] * vb[1],
                    va[2] * vb[0] - va[0] * vb[2],
                    va[0] * vb[1] - va[1] * vb[0]
                ];
                const crossNorm = Math.sqrt(cross[0] ** 2 + cross[1] ** 2 + cross[2] ** 2);
                if (crossNorm > 1e-12) {
                    normal = [cross[0] / crossNorm, cross[1] / crossNorm, cross[2] / crossNorm];
                    d = -(normal[0] * fwdA[0] + normal[1] * fwdA[1] + normal[2] * fwdA[2]);
                    if (d > 0) {
                        normal = [-normal[0], -normal[1], -normal[2]];
                        d = -d;
                    }
                }
            }

            tangents.push([normal[0], normal[1], normal[2], d]);
        }

        // Greedy selection of half-spaces
        const bdFlags = new Array(M).fill(true);
        const pcFlags = new Array(N).fill(true);
        const forwardH: number[][] = [];

        let completed = false;
        while (!completed) {
            // Find closest boundary constraint
            let bdMinId = -1;
            let minSqrD = Infinity;
            for (let j = 0; j < M; j++) {
                if (bdFlags[j] && distDs[j] < minSqrD) {
                    bdMinId = j;
                    minSqrD = distDs[j];
                }
            }

            // Find closest obstacle
            let pcMinId = -1;
            let minSqrR = Infinity;
            for (let j = 0; j < N; j++) {
                if (pcFlags[j] && distRs[j] < minSqrR) {
                    pcMinId = j;
                    minSqrR = distRs[j];
                }
            }

            if (bdMinId < 0 && pcMinId < 0) {
                completed = true;
                break;
            }

            let newH: number[];
            if (minSqrD < minSqrR) {
                newH = [...forwardB[bdMinId], forwardD[bdMinId]];
                bdFlags[bdMinId] = false;
            } else {
                newH = [...tangents[pcMinId]];
                pcFlags[pcMinId] = false;
            }
            forwardH.push(newH);

            // Check which obstacles are now outside
            for (let j = 0; j < N; j++) {
                if (pcFlags[j]) {
                    const pt = forwardPC[j];
                    if (newH[0] * pt[0] + newH[1] * pt[1] + newH[2] * pt[2] + newH[3] > -epsilon) {
                        pcFlags[j] = false;
                    }
                }
            }

            // Check completion
            completed = true;
            for (let j = 0; j < M; j++) {
                if (bdFlags[j]) { completed = false; break; }
            }
            for (let j = 0; j < N; j++) {
                if (pcFlags[j]) { completed = false; break; }
            }
        }

        // Transform back to original coordinates
        hPoly = [];
        for (const h of forwardH) {
            const n = [h[0], h[1], h[2]];
            const nBack = [
                n[0] * forward[0][0] + n[1] * forward[0][1] + n[2] * forward[0][2],
                n[0] * forward[1][0] + n[1] * forward[1][1] + n[2] * forward[1][2],
                n[0] * forward[2][0] + n[1] * forward[2][1] + n[2] * forward[2][2]
            ];
            const dBack = h[3] - (nBack[0] * p[0] + nBack[1] * p[1] + nBack[2] * p[2]);
            hPoly.push([nBack[0], nBack[1], nBack[2], dBack]);
        }

        if (loop === iterations - 1) {
            break;
        }

        // Update ellipsoid for next iteration
        const newEllipsoid = maxVolInsEllipsoid(hPoly, { center: p, rotation: R, radii: r });
        if (newEllipsoid) {
            R = newEllipsoid.rotation;
            p = newEllipsoid.center;
            r = newEllipsoid.radii;
        }
    }

    return hPoly;
}

/**
 * Generate a Safe Flight Corridor (SFC) along a path
 * 
 * @param path Sequence of waypoints defining the path
 * @param obstacles Point cloud of obstacles
 * @param bounds World bounding box as H-representation
 * @param options Configuration options
 * @returns Array of H-polyhedra forming the corridor
 */
export function generateSafeFlightCorridor(
    path: Vector3Array[],
    obstacles: Vector3Array[],
    bounds: HPolyhedron,
    options?: { iterations?: number; epsilon?: number }
): HPolyhedron[] {
    const opts = {
        iterations: 4,
        epsilon: 1e-6,
        ...options
    };

    const corridor: HPolyhedron[] = [];

    for (let i = 0; i < path.length - 1; i++) {
        const poly = inflateCorridorFIRI(
            bounds,
            obstacles,
            path[i],
            path[i + 1],
            opts.iterations,
            opts.epsilon
        );

        if (poly) {
            corridor.push(poly);
        } else {
            // Use bounding box as fallback
            corridor.push([...bounds]);
        }
    }

    return corridor;
}

/**
 * Create a bounding box as H-representation
 */
export function createBoundingBox(
    min: Vector3Array,
    max: Vector3Array
): HPolyhedron {
    return [
        [-1, 0, 0, min[0]],  // x >= min[0]
        [1, 0, 0, -max[0]],  // x <= max[0]
        [0, -1, 0, min[1]],  // y >= min[1]
        [0, 1, 0, -max[1]],  // y <= max[1]
        [0, 0, -1, min[2]],  // z >= min[2]
        [0, 0, 1, -max[2]]   // z <= max[2]
    ];
}
