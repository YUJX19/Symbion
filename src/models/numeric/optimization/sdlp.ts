/**
 * @module optimization/sdlp
 * @description Seidel's algorithm for Linear Programming.
 * 
 * Solves small-dimensional linear programs efficiently using Seidel's randomized
 * incremental algorithm. Designed for low-dimensional problems (d <= 4).
 * 
 * Reference: Seidel, R. (1991), "Small-dimensional linear programming and convex
 * hulls made easy", Discrete & Computational Geometry 6 (1): 423-434.
 * 
 * Original implementation by Michael E. Hohmeyer, adapted with Eigen interface
 * by Zhepei Wang. Ported to TypeScript for this library.
 */

import { dot, norm, squaredNorm, zeros, zerosMatrix, scale } from '../math/linear-algebra';

const EPS = 1e-12;

// ==================== Status Codes ====================

export enum SDLPStatus {
    /** Minimum found */
    MINIMUM = 0,
    /** No feasible region */
    INFEASIBLE = 1,
    /** Unbounded solution */
    UNBOUNDED = 2,
    /** Ambiguous (vertex solution) */
    AMBIGUOUS = 3,
}

// ==================== 2D Helper Functions ====================

function dot2(a: number[], b: number[]): number {
    return a[0] * b[0] + a[1] * b[1];
}

function cross2(a: number[], b: number[]): number {
    return a[0] * b[1] - a[1] * b[0];
}

function unit2(a: number[], b: number[]): boolean {
    const mag = Math.sqrt(a[0] * a[0] + a[1] * a[1]);
    if (mag < 2.0 * EPS) {
        return true;
    }
    b[0] = a[0] / mag;
    b[1] = a[1] / mag;
    return false;
}

// ==================== Random Permutation ====================

function randPermutation(n: number, p: number[]): void {
    for (let i = 0; i < n; i++) {
        p[i] = i;
    }
    for (let i = 0; i < n; i++) {
        const j = Math.floor(Math.random() * (n - i)) + i;
        const k = p[j];
        p[j] = p[i];
        p[i] = k;
    }
}

// ==================== Move to Front ====================

function moveToFront(i: number, next: number[], prev: number[]): number {
    if (i === 0 || i === next[0]) {
        return i;
    }
    const previ = prev[i];
    next[prev[i]] = next[i];
    prev[next[i]] = prev[i];
    next[i] = next[0];
    prev[i] = 0;
    prev[next[i]] = i;
    next[0] = i;
    return previ;
}

// ==================== 2D Base Case ====================

function lpMinLinRat(
    degen: boolean,
    cwVec: number[],
    ccwVec: number[],
    nVec: number[],
    dVec: number[],
    opt: number[]
): void {
    const dCw = dot2(cwVec, dVec);
    const dCcw = dot2(ccwVec, dVec);
    const nCw = dot2(cwVec, nVec);
    const nCcw = dot2(ccwVec, nVec);

    if (degen) {
        if (nCw / dCw < nCcw / dCcw) {
            opt[0] = cwVec[0];
            opt[1] = cwVec[1];
        } else {
            opt[0] = ccwVec[0];
            opt[1] = ccwVec[1];
        }
    } else if (Math.abs(dCw) > 2.0 * EPS && Math.abs(dCcw) > 2.0 * EPS) {
        if (dCw * dCcw > 0.0) {
            if (nCw / dCw < nCcw / dCcw) {
                opt[0] = cwVec[0];
                opt[1] = cwVec[1];
            } else {
                opt[0] = ccwVec[0];
                opt[1] = ccwVec[1];
            }
        } else {
            if (dCw > 0.0) {
                opt[0] = -dVec[1];
                opt[1] = dVec[0];
            } else {
                opt[0] = dVec[1];
                opt[1] = -dVec[0];
            }
        }
    } else if (Math.abs(dCw) > 2.0 * EPS) {
        if (nCcw * dCw > 0.0) {
            opt[0] = cwVec[0];
            opt[1] = cwVec[1];
        } else {
            opt[0] = ccwVec[0];
            opt[1] = ccwVec[1];
        }
    } else if (Math.abs(dCcw) > 2.0 * EPS) {
        if (nCw * dCcw > 2.0 * EPS) {
            opt[0] = ccwVec[0];
            opt[1] = ccwVec[1];
        } else {
            opt[0] = cwVec[0];
            opt[1] = cwVec[1];
        }
    } else {
        if (cross2(dVec, nVec) > 0.0) {
            opt[0] = cwVec[0];
            opt[1] = cwVec[1];
        } else {
            opt[0] = ccwVec[0];
            opt[1] = ccwVec[1];
        }
    }
}

function wedge(
    halves: number[][],
    m: number,
    next: number[],
    prev: number[],
    cwVec: number[],
    ccwVec: number[]
): { status: SDLPStatus; degen: boolean } {
    let degen = false;
    let i = 0;

    // Find first non-degenerate half-space
    while (i !== m) {
        if (!unit2(halves[i], ccwVec)) {
            cwVec[0] = ccwVec[1];
            cwVec[1] = -ccwVec[0];
            ccwVec[0] = -cwVec[0];
            ccwVec[1] = -cwVec[1];
            break;
        }
        i = next[i];
    }

    if (i === m) {
        return { status: SDLPStatus.UNBOUNDED, degen };
    }

    i = 0;
    while (i !== m) {
        let offensive = false;
        const dCw = dot2(cwVec, halves[i]);
        const dCcw = dot2(ccwVec, halves[i]);

        if (dCcw >= 2.0 * EPS) {
            if (dCw <= -2.0 * EPS) {
                cwVec[0] = halves[i][1];
                cwVec[1] = -halves[i][0];
                unit2(cwVec, cwVec);
                offensive = true;
            }
        } else if (dCw >= 2.0 * EPS) {
            if (dCcw <= -2.0 * EPS) {
                ccwVec[0] = -halves[i][1];
                ccwVec[1] = halves[i][0];
                unit2(ccwVec, ccwVec);
                offensive = true;
            }
        } else if (dCcw <= -2.0 * EPS && dCw <= -2.0 * EPS) {
            return { status: SDLPStatus.INFEASIBLE, degen };
        } else if (dCw <= -2.0 * EPS || dCcw <= -2.0 * EPS || cross2(cwVec, halves[i]) < 0.0) {
            if (dCw <= -2.0 * EPS) {
                unit2(ccwVec, cwVec);
            } else if (dCcw <= -2.0 * EPS) {
                unit2(cwVec, ccwVec);
            }
            degen = true;
            offensive = true;
        }

        if (offensive) {
            i = moveToFront(i, next, prev);
        }
        i = next[i];

        if (degen) break;
    }

    if (degen) {
        while (i !== m) {
            const dCw = dot2(cwVec, halves[i]);
            const dCcw = dot2(ccwVec, halves[i]);

            if (dCw < -2.0 * EPS) {
                if (dCcw < -2.0 * EPS) {
                    return { status: SDLPStatus.INFEASIBLE, degen };
                } else {
                    cwVec[0] = ccwVec[0];
                    cwVec[1] = ccwVec[1];
                }
            } else if (dCcw < -2.0 * EPS) {
                ccwVec[0] = cwVec[0];
                ccwVec[1] = cwVec[1];
            }
            i = next[i];
        }
    }

    return { status: SDLPStatus.MINIMUM, degen };
}

function lpBaseCase(
    halves: number[][],
    m: number,
    nVec: number[],
    dVec: number[],
    opt: number[],
    next: number[],
    prev: number[]
): SDLPStatus {
    const cwVec = [0, 0];
    const ccwVec = [0, 0];

    const wedgeResult = wedge(halves, m, next, prev, cwVec, ccwVec);

    if (wedgeResult.status === SDLPStatus.INFEASIBLE) {
        return SDLPStatus.INFEASIBLE;
    }

    if (wedgeResult.status === SDLPStatus.UNBOUNDED) {
        return lpNoCon1(nVec, dVec, opt);
    }

    if (Math.abs(cross2(nVec, dVec)) < 2.0 * EPS * EPS) {
        if (dot2(nVec, nVec) < 2.0 * EPS * EPS || dot2(dVec, dVec) > 2.0 * EPS * EPS) {
            opt[0] = cwVec[0];
            opt[1] = cwVec[1];
            return SDLPStatus.AMBIGUOUS;
        } else {
            if (!wedgeResult.degen && cross2(cwVec, nVec) <= 0.0 && cross2(nVec, ccwVec) <= 0.0) {
                opt[0] = -nVec[0];
                opt[1] = -nVec[1];
            } else if (dot2(nVec, cwVec) > dot2(nVec, ccwVec)) {
                opt[0] = ccwVec[0];
                opt[1] = ccwVec[1];
            } else {
                opt[0] = cwVec[0];
                opt[1] = cwVec[1];
            }
            return SDLPStatus.MINIMUM;
        }
    } else {
        lpMinLinRat(wedgeResult.degen, cwVec, ccwVec, nVec, dVec, opt);
        return SDLPStatus.MINIMUM;
    }
}

// ==================== General Dimension Functions ====================

function lpNoCon(d: number, nVec: number[], dVec: number[], opt: number[]): SDLPStatus {
    let nDotD = 0;
    let dDotD = 0;
    for (let i = 0; i <= d; i++) {
        nDotD += nVec[i] * dVec[i];
        dDotD += dVec[i] * dVec[i];
    }
    if (dDotD < EPS * EPS) {
        nDotD = 0;
        dDotD = 1;
    }
    for (let i = 0; i <= d; i++) {
        opt[i] = -nVec[i] + dVec[i] * nDotD / dDotD;
    }

    // Normalize
    let mag = 0;
    for (let i = 0; i <= d; i++) {
        mag += opt[i] * opt[i];
    }
    if (mag < (d + 1) * EPS * EPS) {
        opt[d] = 1;
        return SDLPStatus.AMBIGUOUS;
    }
    mag = 1 / Math.sqrt(mag);
    for (let i = 0; i <= d; i++) {
        opt[i] *= mag;
    }
    return SDLPStatus.MINIMUM;
}

function lpNoCon1(nVec: number[], dVec: number[], opt: number[]): SDLPStatus {
    return lpNoCon(1, nVec, dVec, opt);
}

function findImax(d: number, pln: number[]): number {
    let imax = 0;
    let rmax = Math.abs(pln[0]);
    for (let i = 1; i <= d; i++) {
        const ab = Math.abs(pln[i]);
        if (ab > rmax) {
            imax = i;
            rmax = ab;
        }
    }
    return imax;
}

// ==================== Recursive LP Solver ====================

/**
 * Recursive linear fractional programming for d dimensions.
 */
function linfracprog(
    d: number,
    halves: number[][],
    maxSize: number,
    m: number,
    nVec: number[],
    dVec: number[],
    opt: number[],
    next: number[],
    prev: number[]
): SDLPStatus {
    // Base case: d = 1
    if (d === 1) {
        if (m > 0) {
            return lpBaseCase(halves, m, nVec, dVec, opt, next, prev);
        } else {
            return lpNoCon1(nVec, dVec, opt);
        }
    }

    let dVecZero = 0;
    for (let j = 0; j <= d; j++) {
        dVecZero += dVec[j] * dVec[j];
    }
    const isDVecZero = dVecZero < (d + 1) * EPS * EPS;

    // Unconstrained minimum
    let status = lpNoCon(d, nVec, dVec, opt);
    if (m <= 0) {
        return status;
    }

    // Allocate for lower dimension
    const newOpt = zeros(d);
    const newNVec = zeros(d);
    const newDVec = zeros(d);
    const newHalves: number[][] = [];
    for (let i = 0; i < maxSize; i++) {
        newHalves.push(zeros(d));
    }

    let i = 0;
    while (i !== m) {
        const planeI = halves[i];

        // Check if optimum violates this half-space
        let val = 0;
        for (let j = 0; j <= d; j++) {
            val += opt[j] * planeI[j];
        }

        if (val < -(d + 1) * EPS) {
            // Find largest coefficient
            const imax = findImax(d, planeI);

            // Project other planes
            if (i !== 0) {
                const fac = 1 / planeI[imax];
                let j = 0;
                while (j !== i) {
                    const oldPlane = halves[j];
                    const crit = oldPlane[imax] * fac;
                    for (let k = 0; k <= d; k++) {
                        if (k !== imax) {
                            const l = k < imax ? k : k - 1;
                            newHalves[j][l] = oldPlane[k] - planeI[k] * crit;
                        }
                    }
                    j = next[j];
                }
            }

            // Project objective
            if (isDVecZero) {
                vectorDown(d, planeI, imax, nVec, newNVec);
                for (let j = 0; j < d; j++) newDVec[j] = 0;
            } else {
                planeDown(d, planeI, imax, nVec, newNVec);
                planeDown(d, planeI, imax, dVec, newDVec);
            }

            // Solve sub-problem
            status = linfracprog(d - 1, newHalves, maxSize, i, newNVec, newDVec, newOpt, next, prev);

            if (status !== SDLPStatus.INFEASIBLE) {
                // Back substitution
                vectorUp(d, planeI, imax, newOpt, opt);

                // Normalize
                let mag = 0;
                for (let j = 0; j <= d; j++) {
                    mag += opt[j] * opt[j];
                }
                mag = 1 / Math.sqrt(mag);
                for (let j = 0; j <= d; j++) {
                    opt[j] *= mag;
                }
            } else {
                return status;
            }

            // Move to front
            i = moveToFront(i, next, prev);
        }

        i = next[i];
    }

    return status;
}

function vectorDown(d: number, elimEqn: number[], ivar: number, oldVec: number[], newVec: number[]): void {
    let ve = 0;
    let ee = 0;
    for (let i = 0; i <= d; i++) {
        ve += oldVec[i] * elimEqn[i];
        ee += elimEqn[i] * elimEqn[i];
    }
    const fac = ve / ee;
    for (let i = 0; i <= d; i++) {
        if (i !== ivar) {
            newVec[i < ivar ? i : i - 1] = oldVec[i] - elimEqn[i] * fac;
        }
    }
}

function vectorUp(d: number, equation: number[], ivar: number, lowVec: number[], vec: number[]): void {
    vec[ivar] = 0;
    for (let i = 0; i <= d; i++) {
        if (i !== ivar) {
            const j = i < ivar ? i : i - 1;
            vec[i] = lowVec[j];
            vec[ivar] -= equation[i] * lowVec[j];
        }
    }
    vec[ivar] /= equation[ivar];
}

function planeDown(d: number, elimEqn: number[], ivar: number, oldPlane: number[], newPlane: number[]): void {
    const crit = oldPlane[ivar] / elimEqn[ivar];
    for (let i = 0; i <= d; i++) {
        if (i !== ivar) {
            newPlane[i < ivar ? i : i - 1] = oldPlane[i] - elimEqn[i] * crit;
        }
    }
}

// ==================== Public API ====================

/**
 * Solve a linear program: min c^T x, s.t. Ax <= b
 * 
 * @param c Objective vector (d-dimensional)
 * @param A Constraint matrix (m x d)
 * @param b Constraint vector (m-dimensional)
 * @returns Object with optimal x and minimum value
 */
export function linearProgram(
    c: number[],
    A: number[][],
    b: number[]
): { x: number[]; minimum: number; status: SDLPStatus } {
    const d = c.length;
    const m = b.length + 1;

    const x = zeros(d);

    if (m <= 1) {
        const minVal = Math.max(...c.map(Math.abs)) > 0 ? -Infinity : 0;
        return { x, minimum: minVal, status: SDLPStatus.MINIMUM };
    }

    // Build halves array (normalized constraints in homogeneous form)
    const halves: number[][] = [];

    // First half-space: x[d] >= 0
    const h0 = zeros(d + 1);
    h0[d] = 1;
    halves.push(h0);

    // Constraint half-spaces
    for (let i = 0; i < b.length; i++) {
        const hi: number[] = [];
        for (let j = 0; j < d; j++) {
            hi.push(-A[i][j]);
        }
        hi.push(b[i]);

        // Normalize
        let normVal = 0;
        for (let j = 0; j <= d; j++) {
            normVal += hi[j] * hi[j];
        }
        normVal = Math.sqrt(normVal);
        if (normVal > EPS) {
            for (let j = 0; j <= d; j++) {
                hi[j] /= normVal;
            }
        }
        halves.push(hi);
    }

    // Objective vectors
    const nVec = zeros(d + 1);
    for (let i = 0; i < d; i++) {
        nVec[i] = c[i];
    }
    const dVec = zeros(d + 1);
    dVec[d] = 1;

    // Linked list setup
    const perm = zeros(m - 1);
    randPermutation(m - 1, perm);

    const next = zeros(m);
    const prev = zeros(m + 1);

    prev[0] = 0;
    next[0] = perm[0] + 1;
    prev[perm[0] + 1] = 0;

    for (let i = 0; i < m - 2; i++) {
        next[perm[i] + 1] = perm[i + 1] + 1;
        prev[perm[i + 1] + 1] = perm[i] + 1;
    }
    next[perm[m - 2] + 1] = m;

    const opt = zeros(d + 1);

    const status = linfracprog(d, halves, m, m, nVec, dVec, opt, next, prev);

    let minimum = Infinity;
    if (status !== SDLPStatus.INFEASIBLE) {
        if (opt[d] !== 0 && status !== SDLPStatus.UNBOUNDED) {
            for (let i = 0; i < d; i++) {
                x[i] = opt[i] / opt[d];
            }
            minimum = dot(c.slice(0, d), x);
        } else if (opt[d] === 0 || status === SDLPStatus.UNBOUNDED) {
            for (let i = 0; i < d; i++) {
                x[i] = opt[i];
            }
            minimum = -Infinity;
        }
    }

    return { x, minimum, status };
}

/**
 * Specialized 3D linear program for FIRI.
 */
export function linprog3(
    c: [number, number, number],
    A: number[][],
    b: number[]
): { x: [number, number, number]; minimum: number } {
    const result = linearProgram(c as number[], A, b);
    return {
        x: [result.x[0], result.x[1], result.x[2]] as [number, number, number],
        minimum: result.minimum
    };
}

/**
 * Specialized 4D linear program for FIRI.
 */
export function linprog4(
    c: [number, number, number, number],
    A: number[][],
    b: number[]
): { x: [number, number, number, number]; minimum: number } {
    const result = linearProgram(c as number[], A, b);
    return {
        x: [result.x[0], result.x[1], result.x[2], result.x[3]] as [number, number, number, number],
        minimum: result.minimum
    };
}
