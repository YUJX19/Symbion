/**
 * @module optimization/lbfgs
 * @description L-BFGS (Limited-memory Broyden–Fletcher–Goldfarb–Shanno) optimization algorithm.
 * 
 * A quasi-Newton optimization method that approximates the inverse Hessian matrix
 * using limited memory. Suitable for large-scale unconstrained optimization problems.
 * 
 * Reference: Jorge Nocedal, "Updating Quasi-Newton Matrices with Limited Storage",
 * Mathematics of Computation, Vol. 35, No. 151, pp. 773-782, 1980.
 */

import {
    dot, norm, squaredNorm, infNorm,
    add, subtract, scale, negate,
    copyVector, zeros, axpy
} from '../math/linear-algebra';
import type { LBFGSConfig, OptimizationResult, CostFunction } from './types';

// ==================== Return Status Codes ====================

export enum LBFGSStatus {
    /** L-BFGS reached convergence (gradient epsilon) */
    CONVERGENCE = 0,
    /** L-BFGS met stopping criteria (delta-based) */
    STOP = 1,
    /** Iteration canceled by user callback */
    CANCELED = 2,
    /** Unknown error */
    ERROR_UNKNOWN = -1024,
    /** Invalid number of variables */
    ERROR_INVALID_N = -1023,
    /** Invalid memory size */
    ERROR_INVALID_MEMSIZE = -1022,
    /** Invalid gradient epsilon */
    ERROR_INVALID_GEPSILON = -1021,
    /** Invalid test period (past) */
    ERROR_INVALID_TESTPERIOD = -1020,
    /** Invalid delta */
    ERROR_INVALID_DELTA = -1019,
    /** Invalid minimum step */
    ERROR_INVALID_MINSTEP = -1018,
    /** Invalid maximum step */
    ERROR_INVALID_MAXSTEP = -1017,
    /** Invalid function decrease coefficient */
    ERROR_INVALID_FDECCOEFF = -1016,
    /** Invalid curvature coefficient */
    ERROR_INVALID_SCURVCOEFF = -1015,
    /** Invalid machine precision */
    ERROR_INVALID_MACHINEPREC = -1014,
    /** Invalid max line search */
    ERROR_INVALID_MAXLINESEARCH = -1013,
    /** Function value became NaN or Inf */
    ERROR_INVALID_FUNCVAL = -1012,
    /** Step smaller than minimum */
    ERROR_MINIMUMSTEP = -1011,
    /** Step larger than maximum */
    ERROR_MAXIMUMSTEP = -1010,
    /** Maximum line search iterations reached */
    ERROR_MAXIMUMLINESEARCH = -1009,
    /** Maximum iterations reached */
    ERROR_MAXIMUMITERATION = -1008,
    /** Search interval too small */
    ERROR_WIDTHTOOSMALL = -1007,
    /** Invalid parameters (e.g., negative step) */
    ERROR_INVALIDPARAMETERS = -1006,
    /** Search direction increases cost */
    ERROR_INCREASEGRADIENT = -1005,
}

// ==================== L-BFGS Parameters ====================

/**
 * Full L-BFGS parameter set with all tunable options
 */
export interface LBFGSParameters {
    /** Number of corrections to approximate inverse Hessian (default: 8) */
    memSize: number;
    /** Epsilon for gradient convergence test (default: 1e-5) */
    gEpsilon: number;
    /** Distance in iterations for delta-based test (default: 3) */
    past: number;
    /** Delta for convergence test (default: 1e-6) */
    delta: number;
    /** Maximum number of iterations (0 = unlimited) */
    maxIterations: number;
    /** Maximum line search iterations (default: 64) */
    maxLinesearch: number;
    /** Minimum step size (default: 1e-20) */
    minStep: number;
    /** Maximum step size (default: 1e+20) */
    maxStep: number;
    /** Armijo condition coefficient (default: 1e-4) */
    fDecCoeff: number;
    /** Wolfe curvature coefficient (default: 0.9) */
    sCurvCoeff: number;
    /** Cautious update factor (default: 1e-6) */
    cautiousFactor: number;
    /** Machine precision (default: 1e-16) */
    machinePrec: number;
}

/**
 * Default L-BFGS parameters
 */
export function defaultLBFGSParameters(): LBFGSParameters {
    return {
        memSize: 8,
        gEpsilon: 1e-5,
        past: 3,
        delta: 1e-6,
        maxIterations: 0,
        maxLinesearch: 64,
        minStep: 1e-20,
        maxStep: 1e+20,
        fDecCoeff: 1e-4,
        sCurvCoeff: 0.9,
        cautiousFactor: 1e-6,
        machinePrec: 1e-16,
    };
}

/**
 * Create L-BFGS parameters from simplified config
 */
export function createLBFGSParameters(config?: Partial<LBFGSConfig>): LBFGSParameters {
    const defaults = defaultLBFGSParameters();
    if (!config) return defaults;

    return {
        ...defaults,
        memSize: config.memorySize ?? defaults.memSize,
        gEpsilon: config.tolerance ?? defaults.gEpsilon,
        maxIterations: config.maxIterations ?? defaults.maxIterations,
        delta: config.relCostTol ?? defaults.delta,
    };
}

// ==================== Callback Types ====================

/**
 * Evaluate function: computes cost and gradient at point x
 * @param x Current point
 * @returns Object with cost value and gradient vector
 */
export type EvaluateFunction = (x: number[]) => { cost: number; gradient: number[] };

/**
 * Step bound function: provides upper bound on step size
 * @param x Current point before line search
 * @param d Search direction
 * @returns Maximum allowed step size
 */
export type StepBoundFunction = (x: number[], d: number[]) => number;

/**
 * Progress callback: monitors optimization progress
 * @param x Current point
 * @param g Current gradient
 * @param fx Current function value
 * @param step Line search step used
 * @param k Iteration number
 * @param ls Number of line search evaluations
 * @returns Non-zero to cancel optimization
 */
export type ProgressFunction = (
    x: number[], g: number[], fx: number,
    step: number, k: number, ls: number
) => number;

// ==================== Lewis-Overton Line Search ====================

/**
 * Lewis-Overton line search for smooth or nonsmooth functions.
 * Satisfies both Armijo condition and weak Wolfe condition.
 * 
 * Reference: Adrian S. Lewis and Michael L. Overton,
 * "Nonsmooth optimization via quasi-Newton methods",
 * Mathematical Programming, Vol 141, No 1, pp. 135-163, 2013.
 */
function lineSearchLewisOverton(
    x: number[],         // Updated in place
    f: { value: number }, // Updated in place
    g: number[],         // Updated in place
    stp: { value: number }, // Updated in place
    s: number[],         // Search direction
    xp: number[],        // Previous x
    gp: number[],        // Previous gradient
    stpMin: number,
    stpMax: number,
    evaluate: EvaluateFunction,
    param: LBFGSParameters
): number {
    let count = 0;
    let brackt = false;
    let touched = false;

    // Check input parameters
    if (!(stp.value > 0)) {
        return LBFGSStatus.ERROR_INVALIDPARAMETERS;
    }

    // Initial gradient in search direction
    const dginit = dot(gp, s);

    // Make sure s is a descent direction
    if (dginit > 0) {
        return LBFGSStatus.ERROR_INCREASEGRADIENT;
    }

    // Initial function value
    const finit = f.value;
    const dgtest = param.fDecCoeff * dginit;
    const dstest = param.sCurvCoeff * dginit;

    let mu = 0;
    let nu = stpMax;

    while (true) {
        // x = xp + stp * s
        for (let i = 0; i < x.length; i++) {
            x[i] = xp[i] + stp.value * s[i];
        }

        // Evaluate function and gradient
        const result = evaluate(x);
        f.value = result.cost;
        for (let i = 0; i < g.length; i++) {
            g[i] = result.gradient[i];
        }
        count++;

        // Test for errors
        if (!Number.isFinite(f.value)) {
            return LBFGSStatus.ERROR_INVALID_FUNCVAL;
        }

        // Check Armijo condition
        if (f.value > finit + stp.value * dgtest) {
            nu = stp.value;
            brackt = true;
        } else {
            // Check weak Wolfe condition
            if (dot(g, s) < dstest) {
                mu = stp.value;
            } else {
                return count;
            }
        }

        if (param.maxLinesearch <= count) {
            return LBFGSStatus.ERROR_MAXIMUMLINESEARCH;
        }

        if (brackt && (nu - mu) < param.machinePrec * nu) {
            return LBFGSStatus.ERROR_WIDTHTOOSMALL;
        }

        if (brackt) {
            stp.value = 0.5 * (mu + nu);
        } else {
            stp.value *= 2.0;
        }

        if (stp.value < stpMin) {
            return LBFGSStatus.ERROR_MINIMUMSTEP;
        }

        if (stp.value > stpMax) {
            if (touched) {
                return LBFGSStatus.ERROR_MAXIMUMSTEP;
            } else {
                touched = true;
                stp.value = stpMax;
            }
        }
    }
}

// ==================== Main L-BFGS Optimization ====================

/**
 * L-BFGS optimization algorithm.
 * 
 * Minimizes an unconstrained function using limited-memory BFGS method.
 * 
 * @param x0 Initial guess (modified in place to contain solution)
 * @param evaluate Function to compute cost and gradient
 * @param param L-BFGS parameters
 * @param stepBound Optional step bound function
 * @param progress Optional progress callback
 * @returns Object with final cost, status code, and iteration count
 */
export function lbfgsOptimize(
    x0: number[],
    evaluate: EvaluateFunction,
    param?: Partial<LBFGSParameters>,
    stepBound?: StepBoundFunction,
    progress?: ProgressFunction
): { x: number[]; cost: number; status: LBFGSStatus; iterations: number } {

    const p = { ...defaultLBFGSParameters(), ...param };
    const n = x0.length;
    const m = p.memSize;

    // Validate parameters
    if (n <= 0) return { x: x0, cost: Infinity, status: LBFGSStatus.ERROR_INVALID_N, iterations: 0 };
    if (m <= 0) return { x: x0, cost: Infinity, status: LBFGSStatus.ERROR_INVALID_MEMSIZE, iterations: 0 };
    if (p.gEpsilon < 0) return { x: x0, cost: Infinity, status: LBFGSStatus.ERROR_INVALID_GEPSILON, iterations: 0 };
    if (p.past < 0) return { x: x0, cost: Infinity, status: LBFGSStatus.ERROR_INVALID_TESTPERIOD, iterations: 0 };
    if (p.delta < 0) return { x: x0, cost: Infinity, status: LBFGSStatus.ERROR_INVALID_DELTA, iterations: 0 };
    if (p.minStep < 0) return { x: x0, cost: Infinity, status: LBFGSStatus.ERROR_INVALID_MINSTEP, iterations: 0 };
    if (p.maxStep < p.minStep) return { x: x0, cost: Infinity, status: LBFGSStatus.ERROR_INVALID_MAXSTEP, iterations: 0 };
    if (!(p.fDecCoeff > 0 && p.fDecCoeff < 1)) return { x: x0, cost: Infinity, status: LBFGSStatus.ERROR_INVALID_FDECCOEFF, iterations: 0 };
    if (!(p.sCurvCoeff < 1 && p.sCurvCoeff > p.fDecCoeff)) return { x: x0, cost: Infinity, status: LBFGSStatus.ERROR_INVALID_SCURVCOEFF, iterations: 0 };
    if (!(p.machinePrec > 0)) return { x: x0, cost: Infinity, status: LBFGSStatus.ERROR_INVALID_MACHINEPREC, iterations: 0 };
    if (p.maxLinesearch <= 0) return { x: x0, cost: Infinity, status: LBFGSStatus.ERROR_INVALID_MAXLINESEARCH, iterations: 0 };

    // Working variables
    const x = copyVector(x0);
    const xp = zeros(n);
    const g = zeros(n);
    const gp = zeros(n);
    const d = zeros(n);
    const pf = zeros(Math.max(1, p.past));

    // Limited memory storage
    const lmAlpha = zeros(m);
    const lmS: number[][] = [];
    const lmY: number[][] = [];
    const lmYs = zeros(m);

    for (let i = 0; i < m; i++) {
        lmS.push(zeros(n));
        lmY.push(zeros(n));
    }

    // Initial evaluation
    const initial = evaluate(x);
    let fx = initial.cost;
    for (let i = 0; i < n; i++) {
        g[i] = initial.gradient[i];
    }

    // Store initial cost
    pf[0] = fx;

    // Initial direction: negative gradient
    for (let i = 0; i < n; i++) {
        d[i] = -g[i];
    }

    // Check if already at stationary point
    const gnormInf = infNorm(g);
    const xnormInf = infNorm(x);

    let status: LBFGSStatus;
    let k = 1;
    let end = 0;
    let bound = 0;

    if (gnormInf / Math.max(1, xnormInf) < p.gEpsilon) {
        for (let i = 0; i < n; i++) x0[i] = x[i];
        return { x: x0, cost: fx, status: LBFGSStatus.CONVERGENCE, iterations: 0 };
    }

    // Initial step
    let step = 1 / norm(d);

    // Main loop
    while (true) {
        // Store current position and gradient
        for (let i = 0; i < n; i++) {
            xp[i] = x[i];
            gp[i] = g[i];
        }

        // Apply dynamic step bound if provided
        const stepMin = p.minStep;
        let stepMax = p.maxStep;
        if (stepBound) {
            stepMax = Math.min(stepBound(xp, d), p.maxStep);
            step = Math.min(step, 0.5 * stepMax);
        }

        // Line search
        const fRef = { value: fx };
        const stepRef = { value: step };
        const ls = lineSearchLewisOverton(x, fRef, g, stepRef, d, xp, gp, stepMin, stepMax, evaluate, p);
        fx = fRef.value;
        step = stepRef.value;

        if (ls < 0) {
            // Line search failed, revert
            for (let i = 0; i < n; i++) {
                x[i] = xp[i];
                g[i] = gp[i];
            }
            status = ls;
            break;
        }

        // Progress callback
        if (progress) {
            if (progress(x, g, fx, step, k, ls) !== 0) {
                status = LBFGSStatus.CANCELED;
                break;
            }
        }

        // Convergence test (gradient)
        const currentGnormInf = infNorm(g);
        const currentXnormInf = infNorm(x);
        if (currentGnormInf / Math.max(1, currentXnormInf) < p.gEpsilon) {
            status = LBFGSStatus.CONVERGENCE;
            break;
        }

        // Stopping criterion (delta-based)
        if (p.past > 0 && p.past <= k) {
            const rate = Math.abs(pf[k % p.past] - fx) / Math.max(1, Math.abs(fx));
            if (rate < p.delta) {
                status = LBFGSStatus.STOP;
                break;
            }
        }
        pf[k % p.past] = fx;

        // Max iterations check
        if (p.maxIterations !== 0 && p.maxIterations <= k) {
            status = LBFGSStatus.ERROR_MAXIMUMITERATION;
            break;
        }

        k++;

        // Update s and y vectors
        for (let i = 0; i < n; i++) {
            lmS[end][i] = x[i] - xp[i];
            lmY[end][i] = g[i] - gp[i];
        }

        // Compute ys and yy
        const ys = dot(lmY[end], lmS[end]);
        const yy = squaredNorm(lmY[end]);
        lmYs[end] = ys;

        // Compute negative gradient as initial direction
        for (let i = 0; i < n; i++) {
            d[i] = -g[i];
        }

        // Cautious update check
        const cau = squaredNorm(lmS[end]) * norm(gp) * p.cautiousFactor;

        if (ys > cau) {
            // L-BFGS two-loop recursion
            bound = Math.min(bound + 1, m);
            end = (end + 1) % m;

            let j = end;
            for (let i = 0; i < bound; i++) {
                j = (j + m - 1) % m;
                lmAlpha[j] = dot(lmS[j], d) / lmYs[j];
                for (let l = 0; l < n; l++) {
                    d[l] += (-lmAlpha[j]) * lmY[j][l];
                }
            }

            // Scale by ys/yy
            const scale = ys / yy;
            for (let i = 0; i < n; i++) {
                d[i] *= scale;
            }

            for (let i = 0; i < bound; i++) {
                const beta = dot(lmY[j], d) / lmYs[j];
                for (let l = 0; l < n; l++) {
                    d[l] += (lmAlpha[j] - beta) * lmS[j][l];
                }
                j = (j + 1) % m;
            }
        }

        // Try step = 1 for next iteration
        step = 1.0;
    }

    // Copy result back
    for (let i = 0; i < n; i++) {
        x0[i] = x[i];
    }

    return { x: x0, cost: fx, status, iterations: k };
}

/**
 * Simple L-BFGS optimization with basic interface.
 * Conforms to OptimizationResult type for compatibility.
 */
export function minimize(
    costFn: CostFunction,
    x0: number[],
    config?: Partial<LBFGSConfig>
): OptimizationResult {
    const params = createLBFGSParameters(config);

    const result = lbfgsOptimize(
        copyVector(x0),
        (x) => costFn(x),
        params
    );

    return {
        solution: result.x,
        cost: result.cost,
        iterations: result.iterations,
        converged: result.status >= 0,
        gradientNorm: 0, // Would need additional computation
    };
}

/**
 * Get human-readable description of L-BFGS status
 */
export function lbfgsStatusMessage(status: LBFGSStatus): string {
    switch (status) {
        case LBFGSStatus.CONVERGENCE:
            return 'Success: reached convergence (gradient epsilon).';
        case LBFGSStatus.STOP:
            return 'Success: met stopping criteria (delta-based).';
        case LBFGSStatus.CANCELED:
            return 'Canceled by user callback.';
        case LBFGSStatus.ERROR_INVALID_N:
            return 'Invalid number of variables.';
        case LBFGSStatus.ERROR_INVALID_MEMSIZE:
            return 'Invalid memory size.';
        case LBFGSStatus.ERROR_INVALID_FUNCVAL:
            return 'Function value became NaN or Inf.';
        case LBFGSStatus.ERROR_MINIMUMSTEP:
            return 'Line search step too small.';
        case LBFGSStatus.ERROR_MAXIMUMSTEP:
            return 'Line search step too large.';
        case LBFGSStatus.ERROR_MAXIMUMLINESEARCH:
            return 'Maximum line search iterations reached.';
        case LBFGSStatus.ERROR_MAXIMUMITERATION:
            return 'Maximum iterations reached.';
        case LBFGSStatus.ERROR_WIDTHTOOSMALL:
            return 'Search interval width too small.';
        case LBFGSStatus.ERROR_INCREASEGRADIENT:
            return 'Search direction increases cost.';
        default:
            return `Unknown status: ${status}`;
    }
}
