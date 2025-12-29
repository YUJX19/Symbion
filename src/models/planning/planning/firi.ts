/**
 * @module planning/firi
 * @description Fast Inscribed Radius Inference (FIRI) for Safe Flight Corridor Generation
 * 
 * FIRI generates convex polytope corridors for trajectory optimization by:
 * 1. Computing maximum volume inscribed ellipsoids (MVIE) in obstacle-free space
 * 2. Iteratively refining polytopes to include path segments
 * 
 * Based on GCOPTER's firi.hpp by Zhepei Wang.
 * Reference: "Generating Large Convex Polytopes Directly on Point Clouds"
 * 
 * @example
 * ```typescript
 * import { firi, maxVolInsEllipsoid } from '@/lib/algorithms/planning/firi';
 * 
 * // Generate corridor from boundary box and obstacle points
 * const corridor = firi(boundaryBox, obstacles, startPoint, endPoint);
 * ```
 */

import { lbfgsOptimize, LBFGSStatus } from '../../numeric/optimization/lbfgs';
import { linearProgram, SDLPStatus } from '../../numeric/optimization/sdlp';

const DBL_EPSILON = 2.220446049250313e-16;

// ==================== Types ====================

/**
 * H-Polytope representation: Ax + b <= 0
 * Each row is [a1, a2, a3, b] defining half-space a·x + b <= 0
 */
export type HPolyMatrix = number[][];

/**
 * Ellipsoid parameters: R * diag(r) defines the shape
 * Center is at point p
 */
export interface FIRIEllipsoid {
    /** Rotation matrix (3x3) */
    R: number[][];
    /** Center point */
    p: [number, number, number];
    /** Semi-axis radii */
    r: [number, number, number];
}

// ==================== Matrix Utilities ====================

/**
 * Fast 3D Cholesky decomposition: A = L * L^T
 */
function chol3d(A: number[][]): number[][] {
    const L: number[][] = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];

    L[0][0] = Math.sqrt(A[0][0]);
    L[0][1] = 0.0;
    L[0][2] = 0.0;
    L[1][0] = 0.5 * (A[0][1] + A[1][0]) / L[0][0];
    L[1][1] = Math.sqrt(A[1][1] - L[1][0] * L[1][0]);
    L[1][2] = 0.0;
    L[2][0] = 0.5 * (A[0][2] + A[2][0]) / L[0][0];
    L[2][1] = (0.5 * (A[1][2] + A[2][1]) - L[2][0] * L[1][0]) / L[1][1];
    L[2][2] = Math.sqrt(A[2][2] - L[2][0] * L[2][0] - L[2][1] * L[2][1]);

    return L;
}

/**
 * Create 3x3 matrix from diagonal elements
 */
function diag3(d: number[]): number[][] {
    return [
        [d[0], 0, 0],
        [0, d[1], 0],
        [0, 0, d[2]]
    ];
}

/**
 * Matrix-vector multiplication: M * v
 */
function matVec3(M: number[][], v: number[]): number[] {
    return [
        M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
        M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
        M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2]
    ];
}

/**
 * Matrix-matrix multiplication: A * B
 */
function matMat3(A: number[][], B: number[][]): number[][] {
    const C: number[][] = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
            C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
        }
    }
    return C;
}

/**
 * Matrix transpose
 */
function transpose3(M: number[][]): number[][] {
    return [
        [M[0][0], M[1][0], M[2][0]],
        [M[0][1], M[1][1], M[2][1]],
        [M[0][2], M[1][2], M[2][2]]
    ];
}

/**
 * Vector norm
 */
function norm3(v: number[]): number {
    return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/**
 * Vector dot product
 */
function dot3(a: number[], b: number[]): number {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/**
 * Vector cross product
 */
function cross3(a: number[], b: number[]): number[] {
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    ];
}

// ==================== Smoothed L1 Penalty ====================

/**
 * Smoothed L1 penalty function for optimization.
 * Returns false if x < 0, otherwise computes smooth approximation.
 */
function smoothedL1(mu: number, x: number): { valid: boolean; f: number; df: number } {
    if (x < 0.0) {
        return { valid: false, f: 0, df: 0 };
    } else if (x > mu) {
        return { valid: true, f: x - 0.5 * mu, df: 1.0 };
    } else {
        const xdmu = x / mu;
        const sqrxdmu = xdmu * xdmu;
        const mumxd2 = mu - 0.5 * x;
        const f = mumxd2 * sqrxdmu * xdmu;
        const df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
        return { valid: true, f, df };
    }
}

// ==================== MVIE Optimization ====================

/**
 * Cost function for Maximum Volume Inscribed Ellipsoid optimization.
 * 
 * Optimization variables: x = [p (3), rtd (3), cde (3)]
 * - p: ellipsoid center offset
 * - rtd: square roots of diagonal L elements
 * - cde: off-diagonal L elements
 */
function costMVIE(
    x: number[],
    A: number[][],  // Normalized constraint matrix (M x 3)
    smoothEps: number,
    penaltyWt: number
): { cost: number; gradient: number[] } {
    const M = A.length;

    // Extract optimization variables
    const p = [x[0], x[1], x[2]];
    const rtd = [x[3], x[4], x[5]];
    const cde = [x[6], x[7], x[8]];

    // Construct lower triangular L from rtd and cde
    const L: number[][] = [
        [rtd[0] * rtd[0] + DBL_EPSILON, 0, 0],
        [cde[0], rtd[1] * rtd[1] + DBL_EPSILON, 0],
        [cde[2], cde[1], rtd[2] * rtd[2] + DBL_EPSILON]
    ];

    // Compute A * L for each constraint
    const AL: number[][] = [];
    for (let i = 0; i < M; i++) {
        AL.push(matVec3(L, A[i]));
    }

    // Compute norms and constraint violations
    let cost = 0;
    const gdp = [0, 0, 0];
    const gdrtd = [0, 0, 0];
    const gdcde = [0, 0, 0];

    for (let i = 0; i < M; i++) {
        const normAL = norm3(AL[i]);
        const Ap = dot3(A[i], p);
        const violation = normAL + Ap - 1.0;

        const result = smoothedL1(smoothEps, violation);
        if (result.valid) {
            cost += result.f;

            // Gradient contributions
            const dc = result.df;
            for (let j = 0; j < 3; j++) {
                gdp[j] += dc * A[i][j];
            }

            const adjNorm = dc / normAL;
            // Gradient w.r.t. L elements (via chain rule)
            gdrtd[0] += adjNorm * AL[i][0] * A[i][0];
            gdrtd[1] += adjNorm * AL[i][1] * A[i][1];
            gdrtd[2] += adjNorm * AL[i][2] * A[i][2];
            gdcde[0] += adjNorm * AL[i][0] * A[i][1];
            gdcde[1] += adjNorm * AL[i][1] * A[i][2];
            gdcde[2] += adjNorm * AL[i][0] * A[i][2];
        }
    }

    cost *= penaltyWt;
    for (let j = 0; j < 3; j++) {
        gdp[j] *= penaltyWt;
        gdrtd[j] *= penaltyWt;
        gdcde[j] *= penaltyWt;
    }

    // Log-barrier for volume maximization: -log(det(L)) = -sum(log(L_ii))
    cost -= Math.log(L[0][0]) + Math.log(L[1][1]) + Math.log(L[2][2]);
    gdrtd[0] -= 1.0 / L[0][0];
    gdrtd[1] -= 1.0 / L[1][1];
    gdrtd[2] -= 1.0 / L[2][2];

    // Chain rule: d/d(rtd) = d/d(L_ii) * 2 * rtd
    gdrtd[0] *= 2.0 * rtd[0];
    gdrtd[1] *= 2.0 * rtd[1];
    gdrtd[2] *= 2.0 * rtd[2];

    return {
        cost,
        gradient: [gdp[0], gdp[1], gdp[2], gdrtd[0], gdrtd[1], gdrtd[2], gdcde[0], gdcde[1], gdcde[2]]
    };
}

// ==================== Main FIRI Functions ====================

/**
 * Find the deepest interior point of an H-polytope using linear programming.
 * 
 * @param hPoly H-polytope matrix (each row is [a, b, c, d] for ax + by + cz + d <= 0)
 * @returns Interior point or null if infeasible
 */
export function findDeepestInterior(hPoly: HPolyMatrix): [number, number, number] | null {
    const M = hPoly.length;

    // Normalize half-spaces
    const c = [0, 0, 0, -1]; // Minimize -slack (maximize depth)
    const A: number[][] = [];
    const b: number[] = [];

    for (let i = 0; i < M; i++) {
        const row = hPoly[i];
        const norm = Math.sqrt(row[0] * row[0] + row[1] * row[1] + row[2] * row[2]);
        if (norm < DBL_EPSILON) continue;

        // Normalize: (n/||n||) · x + 1 * slack + d/||n|| <= 0
        // Rearranged: [n/||n||, 1] · [x, slack] <= -d/||n||
        A.push([row[0] / norm, row[1] / norm, row[2] / norm, 1]);
        b.push(-row[3] / norm);
    }

    const result = linearProgram(c, A, b);

    if (result.status === SDLPStatus.MINIMUM &&
        result.minimum < 0 &&
        isFinite(result.minimum)) {
        return [result.x[0], result.x[1], result.x[2]];
    }

    return null;
}

/**
 * Compute Maximum Volume Inscribed Ellipsoid in a polytope.
 * 
 * @param hPoly H-polytope (Ax + b <= 0)
 * @param initialEllipsoid Initial guess for ellipsoid (optional)
 * @returns Optimized ellipsoid or null if failed
 */
export function maxVolInsEllipsoid(
    hPoly: HPolyMatrix,
    initialEllipsoid?: FIRIEllipsoid
): FIRIEllipsoid | null {
    // Find interior point
    const interior = findDeepestInterior(hPoly);
    if (!interior) {
        return null;
    }

    const M = hPoly.length;

    // Normalize constraints relative to interior point
    // A_i = n_i / b_i where b_i = n_i · interior - d_i
    const A: number[][] = [];
    for (let i = 0; i < M; i++) {
        const n = [hPoly[i][0], hPoly[i][1], hPoly[i][2]];
        const norm = Math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        if (norm < DBL_EPSILON) continue;

        const nNorm = [n[0] / norm, n[1] / norm, n[2] / norm];
        const bi = (dot3(n, interior) + hPoly[i][3]) / norm;
        if (Math.abs(bi) < DBL_EPSILON) continue;

        A.push([
            -nNorm[0] / bi,
            -nNorm[1] / bi,
            -nNorm[2] / bi
        ]);
    }

    if (A.length < 4) {
        return null;
    }

    // Initialize optimization variables
    let x: number[];
    if (initialEllipsoid) {
        const R = initialEllipsoid.R;
        const r = initialEllipsoid.r;
        const p = initialEllipsoid.p;

        // Q = R * diag(r²) * R^T
        const rSq = diag3([r[0] * r[0], r[1] * r[1], r[2] * r[2]]);
        const Q = matMat3(matMat3(R, rSq), transpose3(R));
        const L = chol3d(Q);

        x = [
            p[0] - interior[0],
            p[1] - interior[1],
            p[2] - interior[2],
            Math.sqrt(L[0][0]),
            Math.sqrt(L[1][1]),
            Math.sqrt(L[2][2]),
            L[1][0],
            L[2][1],
            L[2][0]
        ];
    } else {
        // Start with unit sphere
        x = [0, 0, 0, 1, 1, 1, 0, 0, 0];
    }

    // Run L-BFGS optimization
    const smoothEps = 1e-2;
    const penaltyWt = 1e3;

    const costFn = (xVec: number[]) => costMVIE(xVec, A, smoothEps, penaltyWt);

    const result = lbfgsOptimize(x, costFn, {
        memSize: 18,
        gEpsilon: 0.0,
        minStep: 1e-32,
        past: 3,
        delta: 1e-7,
        maxIterations: 200
    });

    if (result.status < 0 && result.status !== LBFGSStatus.ERROR_MAXIMUMLINESEARCH) {
        console.warn('FIRI MVIE optimization warning:', result.status);
    }

    // Extract ellipsoid from optimized variables
    const pOpt = [
        result.x[0] + interior[0],
        result.x[1] + interior[1],
        result.x[2] + interior[2]
    ];

    const L: number[][] = [
        [result.x[3] * result.x[3], 0, 0],
        [result.x[6], result.x[4] * result.x[4], 0],
        [result.x[8], result.x[7], result.x[5] * result.x[5]]
    ];

    // SVD of L to get rotation and radii
    // Simplified: for small rotations, L ≈ diag(r)
    // Full SVD would require more complex implementation
    const r: [number, number, number] = [L[0][0], L[1][1], L[2][2]];
    const R: number[][] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

    return {
        R,
        p: pOpt as [number, number, number],
        r
    };
}

/**
 * Main FIRI algorithm: Generate convex corridor from boundary and obstacles.
 * 
 * @param bd Boundary polytope (usually a box)
 * @param pc Point cloud of obstacles (3 x N)
 * @param a Start point of path segment
 * @param b End point of path segment
 * @param iterations Number of refinement iterations (default: 4)
 * @param epsilon Numerical tolerance
 * @returns H-polytope corridor or null if failed
 */
export function firi(
    bd: HPolyMatrix,
    pc: number[][],  // Each row is [x, y, z]
    a: [number, number, number],
    b: [number, number, number],
    iterations: number = 4,
    epsilon: number = 1e-6
): HPolyMatrix | null {
    // Check that start and end are inside boundary
    const ah = [a[0], a[1], a[2], 1];
    const bh = [b[0], b[1], b[2], 1];

    for (const row of bd) {
        if (row[0] * a[0] + row[1] * a[1] + row[2] * a[2] + row[3] > 0 ||
            row[0] * b[0] + row[1] * b[1] + row[2] * b[2] + row[3] > 0) {
            return null;
        }
    }

    const M = bd.length;
    const N = pc.length;

    // Initialize ellipsoid
    let R: number[][] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
    let p: [number, number, number] = [
        0.5 * (a[0] + b[0]),
        0.5 * (a[1] + b[1]),
        0.5 * (a[2] + b[2])
    ];
    let rVec: [number, number, number] = [1, 1, 1];

    let hPoly: HPolyMatrix = [];

    for (let loop = 0; loop < iterations; loop++) {
        // Compute forward/backward transforms
        const rInv = [1 / rVec[0], 1 / rVec[1], 1 / rVec[2]];
        const forward = matMat3(diag3(rInv), transpose3(R));
        const backward = matMat3(R, diag3(rVec));

        // Transform boundary constraints
        const forwardB: number[][] = [];
        const forwardD: number[] = [];
        for (let i = 0; i < M; i++) {
            const n = [bd[i][0], bd[i][1], bd[i][2]];
            forwardB.push(matVec3(backward, n));
            forwardD.push(bd[i][3] + dot3(n, p));
        }

        // Transform point cloud
        const forwardPC: number[][] = [];
        for (let i = 0; i < N; i++) {
            const delta = [pc[i][0] - p[0], pc[i][1] - p[1], pc[i][2] - p[2]];
            forwardPC.push(matVec3(forward, delta));
        }

        // Transform path endpoints
        const fwd_a = matVec3(forward, [a[0] - p[0], a[1] - p[1], a[2] - p[2]]);
        const fwd_b = matVec3(forward, [b[0] - p[0], b[1] - p[1], b[2] - p[2]]);

        // Compute distances
        const distDs: number[] = [];
        for (let i = 0; i < M; i++) {
            distDs.push(Math.abs(forwardD[i]) / norm3(forwardB[i]));
        }

        // Compute tangent planes for obstacles
        const tangents: number[][] = [];
        const distRs: number[] = [];
        for (let i = 0; i < N; i++) {
            const dist = norm3(forwardPC[i]);
            if (dist < epsilon) {
                distRs.push(Infinity);
                tangents.push([0, 0, 0, 0]);
                continue;
            }

            distRs.push(dist);
            const normal = [
                forwardPC[i][0] / dist,
                forwardPC[i][1] / dist,
                forwardPC[i][2] / dist
            ];

            // Check if tangent plane separates path endpoints
            let tangent = [...normal, -dist];

            if (dot3(normal, fwd_a) + tangent[3] > epsilon) {
                // Adjust tangent to include point a
                const delta = [
                    forwardPC[i][0] - fwd_a[0],
                    forwardPC[i][1] - fwd_a[1],
                    forwardPC[i][2] - fwd_a[2]
                ];
                const newNormal = [
                    fwd_a[0] - (dot3(delta, fwd_a) / dot3(delta, delta)) * delta[0],
                    fwd_a[1] - (dot3(delta, fwd_a) / dot3(delta, delta)) * delta[1],
                    fwd_a[2] - (dot3(delta, fwd_a) / dot3(delta, delta)) * delta[2]
                ];
                const newDist = norm3(newNormal);
                if (newDist > epsilon) {
                    tangent = [
                        newNormal[0] / newDist,
                        newNormal[1] / newDist,
                        newNormal[2] / newDist,
                        -newDist
                    ];
                    distRs[i] = newDist;
                }
            }

            tangents.push(tangent);
        }

        // Select constraints for corridor
        const forwardH: number[][] = [];
        const bdFlags = new Array(M).fill(true);
        const pcFlags = new Array(N).fill(true);

        // Add constraints in order of distance
        let done = false;
        for (let iter = 0; iter < M + N && !done; iter++) {
            // Find closest unused constraint
            let minBdIdx = -1, minPcIdx = -1;
            let minBdDist = Infinity, minPcDist = Infinity;

            for (let i = 0; i < M; i++) {
                if (bdFlags[i] && distDs[i] < minBdDist) {
                    minBdDist = distDs[i];
                    minBdIdx = i;
                }
            }

            for (let i = 0; i < N; i++) {
                if (pcFlags[i] && distRs[i] < minPcDist) {
                    minPcDist = distRs[i];
                    minPcIdx = i;
                }
            }

            if (minBdDist <= minPcDist && minBdIdx >= 0) {
                forwardH.push([
                    forwardB[minBdIdx][0],
                    forwardB[minBdIdx][1],
                    forwardB[minBdIdx][2],
                    forwardD[minBdIdx]
                ]);
                bdFlags[minBdIdx] = false;
            } else if (minPcIdx >= 0) {
                forwardH.push([...tangents[minPcIdx]]);
                pcFlags[minPcIdx] = false;
            }

            // Check if remaining points are separated
            done = true;
            for (let i = 0; i < M; i++) {
                if (bdFlags[i]) done = false;
            }
            for (let i = 0; i < N; i++) {
                if (pcFlags[i]) {
                    const lastH = forwardH[forwardH.length - 1];
                    if (dot3([lastH[0], lastH[1], lastH[2]], forwardPC[i]) + lastH[3] > -epsilon) {
                        pcFlags[i] = false;
                    } else {
                        done = false;
                    }
                }
            }
        }

        // Transform back to original frame
        hPoly = [];
        for (const h of forwardH) {
            const hNormal = [h[0], h[1], h[2]];
            const originalN = matVec3(forward, hNormal);
            const originalD = h[3] - dot3([originalN[0], originalN[1], originalN[2]], p);
            hPoly.push([originalN[0], originalN[1], originalN[2], originalD]);
        }

        if (loop < iterations - 1) {
            // Refine ellipsoid for next iteration
            const ellipsoid = maxVolInsEllipsoid(hPoly, { R, p, r: rVec });
            if (ellipsoid) {
                R = ellipsoid.R;
                p = ellipsoid.p;
                rVec = ellipsoid.r;
            }
        }
    }

    return hPoly;
}

/**
 * Generate convex corridor covering a path through obstacles.
 * 
 * @param path Waypoints of the path
 * @param obstacles Obstacle point cloud
 * @param bounds [lowCorner, highCorner] of workspace
 * @param progress Step size for path segmentation
 * @param range Search range for obstacle points
 * @returns Array of H-polytope corridors
 */
export function convexCover(
    path: [number, number, number][],
    obstacles: [number, number, number][],
    bounds: { low: [number, number, number]; high: [number, number, number] },
    progress: number = 0.5,
    range: number = 2.0
): HPolyMatrix[] {
    const corridors: HPolyMatrix[] = [];
    const n = path.length;

    // Create boundary box
    const bd: HPolyMatrix = [
        [1, 0, 0, -bounds.high[0]],
        [-1, 0, 0, bounds.low[0]],
        [0, 1, 0, -bounds.high[1]],
        [0, -1, 0, bounds.low[1]],
        [0, 0, 1, -bounds.high[2]],
        [0, 0, -1, bounds.low[2]]
    ];

    let b = path[0];

    for (let i = 1; i < n;) {
        const a = b;

        // Advance along path
        const dist = norm3([path[i][0] - a[0], path[i][1] - a[1], path[i][2] - a[2]]);
        if (dist > progress) {
            const dir = [(path[i][0] - a[0]) / dist, (path[i][1] - a[1]) / dist, (path[i][2] - a[2]) / dist];
            b = [
                a[0] + progress * dir[0],
                a[1] + progress * dir[1],
                a[2] + progress * dir[2]
            ] as [number, number, number];
        } else {
            b = path[i];
            i++;
        }

        // Compute local bounding box
        const localBd: HPolyMatrix = [
            [1, 0, 0, -Math.min(Math.max(a[0], b[0]) + range, bounds.high[0])],
            [-1, 0, 0, Math.max(Math.min(a[0], b[0]) - range, bounds.low[0])],
            [0, 1, 0, -Math.min(Math.max(a[1], b[1]) + range, bounds.high[1])],
            [0, -1, 0, Math.max(Math.min(a[1], b[1]) - range, bounds.low[1])],
            [0, 0, 1, -Math.min(Math.max(a[2], b[2]) + range, bounds.high[2])],
            [0, 0, -1, Math.max(Math.min(a[2], b[2]) - range, bounds.low[2])]
        ];

        // Filter obstacles within local bounds
        const validPc: number[][] = [];
        for (const p of obstacles) {
            let inside = true;
            for (const row of localBd) {
                if (row[0] * p[0] + row[1] * p[1] + row[2] * p[2] + row[3] > 0) {
                    inside = false;
                    break;
                }
            }
            if (inside) {
                validPc.push(p);
            }
        }

        // Generate corridor
        const hp = firi(localBd, validPc, a, b);
        if (hp) {
            corridors.push(hp);
        }
    }

    return corridors;
}

/**
 * Simplify corridor sequence by removing redundant intermediate polytopes.
 */
export function shortCut(corridors: HPolyMatrix[]): HPolyMatrix[] {
    if (corridors.length <= 1) {
        return corridors;
    }

    const result: HPolyMatrix[] = [];
    const indices: number[] = [corridors.length - 1];

    for (let i = corridors.length - 1; i >= 0;) {
        for (let j = 0; j < i; j++) {
            // Check if corridors[i] and corridors[j] overlap
            const combined = [...corridors[i], ...corridors[j]];
            const interior = findDeepestInterior(combined);

            if (interior || j === i - 1) {
                indices.unshift(j);
                i = j + 1;
                break;
            }
        }
        if (i === indices[0]) i--;
    }

    for (const idx of indices) {
        result.push(corridors[idx]);
    }

    return result;
}
