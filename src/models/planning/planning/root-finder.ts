/**
 * @module planning/root-finder
 * @description Polynomial root finding algorithms for trajectory optimization.
 * 
 * Provides robust numerical methods for finding polynomial roots:
 * - Cubic/Quartic closed-form solutions
 * - Sturm sequence-based isolation for high-order polynomials
 * - Safe Newton-Raphson with bisection fallback
 * - Eigenvalue-based root finding via companion matrix
 * 
 * These algorithms are essential for:
 * - Finding maximum velocity/acceleration along trajectories
 * - Solving constrained optimization sub-problems
 * - Time allocation in minimum snap trajectories
 * 
 * Based on GCOPTER/Fast-Racing polynomial utilities by Zhepei Wang.
 * 
 * @example
 * ```typescript
 * import { solveCubic, solveQuartic, isolateRealRoots } from '@/lib/algorithms/planning/root-finder';
 * 
 * // Solve x³ - 6x² + 11x - 6 = 0 (roots: 1, 2, 3)
 * const cubicRoots = solveCubic(1, -6, 11, -6);
 * console.log(cubicRoots); // [1, 2, 3]
 * 
 * // Find roots of high-order polynomial in interval [0, 10]
 * const coeffs = [1, -15, 85, -225, 274, -120]; // (x-1)(x-2)(x-3)(x-4)(x-5)
 * const roots = isolateRealRoots(coeffs, 0, 10, 1e-10);
 * console.log(roots); // [1, 2, 3, 4, 5]
 * ```
 */

const DBL_EPSILON = 2.220446049250313e-16;

// ==================== Polynomial Evaluation ====================

/**
 * Evaluate polynomial at point x.
 * Uses stable summation (not Horner scheme) to avoid catastrophic cancellation
 * near zeros, which is critical for root-finding algorithms.
 * 
 * @param coeffs Polynomial coefficients [a_n, a_{n-1}, ..., a_1, a_0]
 * @param x Point to evaluate at
 * @returns p(x) = a_n*x^n + a_{n-1}*x^{n-1} + ... + a_1*x + a_0
 */
export function polyEval(coeffs: number[], x: number): number {
    const len = coeffs.length;
    if (len === 0) return 0;

    if (Math.abs(x) < DBL_EPSILON) {
        return coeffs[len - 1];
    }

    if (x === 1.0) {
        let sum = 0;
        for (let i = len - 1; i >= 0; i--) {
            sum += coeffs[i];
        }
        return sum;
    }

    // Evaluate as sum of terms from lowest to highest power
    let result = 0;
    let xn = 1.0;
    for (let i = len - 1; i >= 0; i--) {
        result += coeffs[i] * xn;
        xn *= x;
    }

    return result;
}

/**
 * Compute polynomial derivative coefficients.
 * 
 * @param coeffs Polynomial coefficients [a_n, ..., a_0]
 * @returns Derivative coefficients [n*a_n, (n-1)*a_{n-1}, ..., a_1]
 */
export function polyDerivative(coeffs: number[]): number[] {
    const order = coeffs.length - 1;
    if (order <= 0) return [0];

    const dcoeffs: number[] = new Array(order);
    for (let i = 0; i < order; i++) {
        dcoeffs[i] = (order - i) * coeffs[i];
    }
    return dcoeffs;
}

/**
 * Compute polynomial convolution (multiplication).
 * 
 * @param lCoef First polynomial coefficients
 * @param rCoef Second polynomial coefficients
 * @returns Coefficients of product polynomial
 */
export function polyConv(lCoef: number[], rCoef: number[]): number[] {
    const resultLen = lCoef.length + rCoef.length - 1;
    const result: number[] = new Array(resultLen).fill(0);

    for (let i = 0; i < resultLen; i++) {
        for (let j = 0; j <= i; j++) {
            if (j < lCoef.length && (i - j) < rCoef.length) {
                result[i] += lCoef[j] * rCoef[i - j];
            }
        }
    }

    return result;
}

/**
 * Compute square of a polynomial: p(x)² = p(x) * p(x)
 * 
 * This is frequently used in trajectory optimization to compute
 * squared velocity/acceleration magnitudes for constraint checking.
 * 
 * @param coeffs Polynomial coefficients [a_n, a_{n-1}, ..., a_0]
 * @returns Coefficients of p(x)²
 * 
 * @example
 * ```typescript
 * // For p(x) = x + 1, p(x)² = x² + 2x + 1
 * const squared = polySqr([1, 1]);
 * console.log(squared); // [1, 2, 1]
 * ```
 */
export function polySqr(coeffs: number[]): number[] {
    return polyConv(coeffs, coeffs);
}

/**
 * Count the number of real roots in an interval using Sturm's theorem.
 * 
 * @param coeffs Polynomial coefficients
 * @param lbound Lower bound of interval
 * @param ubound Upper bound of interval
 * @returns Number of distinct real roots in (lbound, ubound)
 */
export function countRoots(coeffs: number[], lbound: number, ubound: number): number {
    if (coeffs.length === 0 || Math.abs(coeffs[0]) < DBL_EPSILON) {
        return 0;
    }

    const order = coeffs.length - 1;

    // Create monic polynomial
    const monicCoeffs: number[] = new Array(order + 1);
    monicCoeffs[0] = 1.0;
    for (let i = 1; i <= order; i++) {
        monicCoeffs[i] = coeffs[i] / coeffs[0];
    }

    // Build Sturm sequence
    const sturmSeqs: number[][] = [];
    sturmSeqs.push([...monicCoeffs]);

    const derivCoeffs: number[] = new Array(order);
    for (let i = 0; i < order; i++) {
        derivCoeffs[i] = (order - i) * monicCoeffs[i] / order;
    }
    sturmSeqs.push(derivCoeffs);

    let idx = 0;
    while (sturmSeqs[idx + 1].length > 1 && idx < 64) {
        const remainder = polyModInternal(sturmSeqs[idx], sturmSeqs[idx + 1]);
        if (remainder.length > 0 && remainder[0] !== 0) {
            const leadSign = remainder[0] > 0 ? -1 : 1;
            const leadAbs = Math.abs(remainder[0]);
            for (let i = 0; i < remainder.length; i++) {
                remainder[i] = (i === 0) ? leadSign : (-remainder[i] / leadAbs);
            }
        }
        sturmSeqs.push(remainder);
        idx++;
    }

    // Count sign variations at bounds
    const lnv = numSignVarInternal(lbound, sturmSeqs);
    const rnv = numSignVarInternal(ubound, sturmSeqs);

    return Math.abs(lnv - rnv);
}

/**
 * Count sign variations at a point (internal helper for countRoots).
 */
function numSignVarInternal(x: number, sturmSeqs: number[][]): number {
    let signVar = 0;
    let lasty = polyEval(sturmSeqs[0], x);

    for (let i = 1; i < sturmSeqs.length; i++) {
        const y = polyEval(sturmSeqs[i], x);
        if (lasty === 0 || lasty * y < 0) {
            signVar++;
        }
        lasty = y;
    }

    return signVar;
}

/**
 * Polynomial modulus helper (internal for countRoots).
 */
function polyModInternal(u: number[], v: number[]): number[] {
    const orderu = u.length - 1;
    const orderv = v.length - 1;
    const r = [...u];

    if (v[0] < 0) {
        for (let i = orderv + 1; i <= orderu; i += 2) {
            r[i] = -r[i];
        }
        for (let i = 0; i <= orderu - orderv; i++) {
            for (let j = i + 1; j <= orderv + i; j++) {
                r[j] = -r[j] - r[i] * v[j - i];
            }
        }
    } else {
        for (let i = 0; i <= orderu - orderv; i++) {
            for (let j = i + 1; j <= orderv + i; j++) {
                r[j] = r[j] - r[i] * v[j - i];
            }
        }
    }

    let k = orderv - 1;
    while (k >= 0 && Math.abs(r[orderu - k]) < DBL_EPSILON) {
        r[orderu - k] = 0;
        k--;
    }

    const start = k <= 0 ? orderu : orderu - k;
    return r.slice(start);
}

// ==================== Cubic Equation Solver ====================

/**
 * Find all real roots of cubic equation: a*x³ + b*x² + c*x + d = 0
 * 
 * Uses Cardano's formula with proper handling of:
 * - Degenerate cases (linear, quadratic)
 * - Triple real roots
 * - Complex conjugate pairs
 * 
 * @param a Cubic coefficient
 * @param b Quadratic coefficient
 * @param c Linear coefficient
 * @param d Constant term
 * @returns Sorted array of unique real roots
 */
export function solveCubic(a: number, b: number, c: number, d: number): number[] {
    const roots = new Set<number>();

    const cos120 = -0.5;
    const sin120 = 0.8660254037844386;  // sin(120°) = √3/2

    // Handle x=0 as root
    if (Math.abs(d) < DBL_EPSILON) {
        roots.add(0.0);
        d = c;
        c = b;
        b = a;
        a = 0;
    }

    if (Math.abs(a) < DBL_EPSILON) {
        if (Math.abs(b) < DBL_EPSILON) {
            // Linear: c*x + d = 0
            if (Math.abs(c) > DBL_EPSILON) {
                roots.add(-d / c);
            }
        } else {
            // Quadratic: b*x² + c*x + d = 0
            const discriminant = c * c - 4.0 * b * d;
            if (discriminant >= 0) {
                const inv2b = 1.0 / (2.0 * b);
                const y = Math.sqrt(discriminant);
                roots.add((-c + y) * inv2b);
                roots.add((-c - y) * inv2b);
            }
        }
    } else {
        // Full cubic equation
        const inva = 1.0 / a;
        const invaa = inva * inva;
        const bb = b * b;
        const bover3a = b * (1.0 / 3.0) * inva;
        const p = (3.0 * a * c - bb) * (1.0 / 3.0) * invaa;
        const halfq = (2.0 * bb * b - 9.0 * a * b * c + 27.0 * a * a * d) * (0.5 / 27.0) * invaa * inva;
        const yy = p * p * p / 27.0 + halfq * halfq;

        if (yy > DBL_EPSILON) {
            // One real root
            const y = Math.sqrt(yy);
            const uuu = -halfq + y;
            const vvv = -halfq - y;
            const www = Math.abs(uuu) > Math.abs(vvv) ? uuu : vvv;
            const w = www < 0 ? -Math.pow(Math.abs(www), 1.0 / 3.0) : Math.pow(www, 1.0 / 3.0);
            roots.add(w - p / (3.0 * w) - bover3a);
        } else if (yy < -DBL_EPSILON) {
            // Three real roots
            const x = -halfq;
            const y = Math.sqrt(-yy);
            let theta: number;
            let r: number;

            if (Math.abs(x) > DBL_EPSILON) {
                theta = x > 0 ? Math.atan(y / x) : Math.atan(y / x) + Math.PI;
                r = Math.sqrt(x * x - yy);
            } else {
                theta = Math.PI / 2.0;
                r = y;
            }

            theta /= 3.0;
            r = Math.pow(r, 1.0 / 3.0);

            const ux = Math.cos(theta) * r;
            const uyi = Math.sin(theta) * r;

            roots.add(ux + ux - bover3a);
            roots.add(2.0 * (ux * cos120 - uyi * sin120) - bover3a);
            roots.add(2.0 * (ux * cos120 + uyi * sin120) - bover3a);
        } else {
            // Two real roots (one double)
            const www = -halfq;
            const w = www < 0 ? -Math.pow(Math.abs(www), 1.0 / 3.0) : Math.pow(www, 1.0 / 3.0);
            roots.add(w + w - bover3a);
            roots.add(2.0 * w * cos120 - bover3a);
        }
    }

    return Array.from(roots).sort((a, b) => a - b);
}

// ==================== Quartic Equation Solver ====================

/**
 * Solve resolvent cubic for quartic equation.
 * 
 * @param a Coefficient
 * @param b Coefficient
 * @param c Coefficient
 * @returns Object with roots array and count
 */
function solveResolvent(a: number, b: number, c: number): { x: number[], count: number } {
    const x: number[] = [0, 0, 0];
    const a2 = a * a;
    const q = (a2 - 3.0 * b) / 9.0;
    const r = (a * (2.0 * a2 - 9.0 * b) + 27.0 * c) / 54.0;
    const r2 = r * r;
    const q3 = q * q * q;

    if (r2 < q3) {
        let t = r / Math.sqrt(q3);
        t = Math.max(-1.0, Math.min(1.0, t));
        t = Math.acos(t);
        const aDiv3 = a / 3.0;
        const qNeg2Sqrt = -2.0 * Math.sqrt(q);
        x[0] = qNeg2Sqrt * Math.cos(t / 3.0) - aDiv3;
        x[1] = qNeg2Sqrt * Math.cos((t + Math.PI * 2.0) / 3.0) - aDiv3;
        x[2] = qNeg2Sqrt * Math.cos((t - Math.PI * 2.0) / 3.0) - aDiv3;
        return { x, count: 3 };
    } else {
        const A = -Math.pow(Math.abs(r) + Math.sqrt(r2 - q3), 1.0 / 3.0);
        const Asign = r < 0 ? -A : A;
        const B = Asign === 0 ? 0 : q / Asign;
        const aDiv3 = a / 3.0;
        x[0] = (Asign + B) - aDiv3;
        x[1] = -0.5 * (Asign + B) - aDiv3;
        x[2] = 0.5 * Math.sqrt(3.0) * (Asign - B);

        if (Math.abs(x[2]) < DBL_EPSILON) {
            x[2] = x[1];
            return { x, count: 2 };
        }
        return { x, count: 1 };
    }
}

/**
 * Find all real roots of monic quartic: x⁴ + a*x³ + b*x² + c*x + d = 0
 */
function solveQuarticMonic(a: number, b: number, c: number, d: number): number[] {
    const roots = new Set<number>();

    const a3 = -b;
    const b3 = a * c - 4.0 * d;
    const c3 = -a * a * d - c * c + 4.0 * b * d;

    const { x: x3, count: iZeroes } = solveResolvent(a3, b3, c3);

    // Choose y with maximal absolute value
    let y = x3[0];
    if (iZeroes !== 1) {
        if (Math.abs(x3[1]) > Math.abs(y)) y = x3[1];
        if (Math.abs(x3[2]) > Math.abs(y)) y = x3[2];
    }

    let q1: number, q2: number, p1: number, p2: number;
    let D = y * y - 4.0 * d;

    if (Math.abs(D) < DBL_EPSILON) {
        q1 = q2 = y * 0.5;
        D = a * a - 4.0 * (b - y);
        if (Math.abs(D) < DBL_EPSILON) {
            p1 = p2 = a * 0.5;
        } else {
            const sqrtD = Math.sqrt(D);
            p1 = (a + sqrtD) * 0.5;
            p2 = (a - sqrtD) * 0.5;
        }
    } else {
        const sqrtD = Math.sqrt(D);
        q1 = (y + sqrtD) * 0.5;
        q2 = (y - sqrtD) * 0.5;
        p1 = (a * q1 - c) / (q1 - q2);
        p2 = (c - a * q2) / (q1 - q2);
    }

    // Solve x² + p1*x + q1 = 0
    D = p1 * p1 - 4.0 * q1;
    if (Math.abs(D) < DBL_EPSILON) {
        roots.add(-p1 * 0.5);
    } else if (D > 0) {
        const sqrtD = Math.sqrt(D);
        roots.add((-p1 + sqrtD) * 0.5);
        roots.add((-p1 - sqrtD) * 0.5);
    }

    // Solve x² + p2*x + q2 = 0
    D = p2 * p2 - 4.0 * q2;
    if (Math.abs(D) < DBL_EPSILON) {
        roots.add(-p2 * 0.5);
    } else if (D > 0) {
        const sqrtD = Math.sqrt(D);
        roots.add((-p2 + sqrtD) * 0.5);
        roots.add((-p2 - sqrtD) * 0.5);
    }

    return Array.from(roots).sort((a, b) => a - b);
}

/**
 * Find all real roots of quartic equation: a*x⁴ + b*x³ + c*x² + d*x + e = 0
 * 
 * @param a Quartic coefficient
 * @param b Cubic coefficient
 * @param c Quadratic coefficient
 * @param d Linear coefficient
 * @param e Constant term
 * @returns Sorted array of unique real roots
 */
export function solveQuartic(a: number, b: number, c: number, d: number, e: number): number[] {
    if (Math.abs(a) < DBL_EPSILON) {
        return solveCubic(b, c, d, e);
    }
    return solveQuarticMonic(b / a, c / a, d / a, e / a);
}

// ==================== Safe Newton-Raphson ====================

/**
 * Safe Newton-Raphson with bisection fallback.
 * Guaranteed to converge for continuous functions where f(l)*f(h) <= 0.
 * 
 * @param func Function to find root of
 * @param dfunc Derivative of function
 * @param l Lower bound (must have f(l)*f(h) <= 0)
 * @param h Upper bound
 * @param tol Tolerance for convergence
 * @param maxIts Maximum iterations
 * @returns Root within [l, h]
 */
export function safeNewton(
    func: (x: number) => number,
    dfunc: (x: number) => number,
    l: number,
    h: number,
    tol: number,
    maxIts: number
): number {
    const fl = func(l);
    const fh = func(h);

    if (fl === 0.0) return l;
    if (fh === 0.0) return h;

    let xl: number, xh: number;
    if (fl < 0) {
        xl = l;
        xh = h;
    } else {
        xh = l;
        xl = h;
    }

    let rts = 0.5 * (xl + xh);
    let dxold = Math.abs(xh - xl);
    let dx = dxold;
    let f = func(rts);
    let df = dfunc(rts);

    for (let j = 0; j < maxIts; j++) {
        // Use bisection if Newton would go out of bounds or converge too slowly
        if ((((rts - xh) * df - f) * ((rts - xl) * df - f) > 0) ||
            (Math.abs(2.0 * f) > Math.abs(dxold * df))) {
            dxold = dx;
            dx = 0.5 * (xh - xl);
            rts = xl + dx;
            if (xl === rts) break;
        } else {
            // Newton step
            dxold = dx;
            dx = f / df;
            const temp = rts;
            rts -= dx;
            if (temp === rts) break;
        }

        if (Math.abs(dx) < tol) break;

        f = func(rts);
        df = dfunc(rts);

        if (f < 0) {
            xl = rts;
        } else {
            xh = rts;
        }
    }

    return rts;
}

// ==================== Polynomial Modulus for Sturm Sequences ====================

/**
 * Compute polynomial modulus for Sturm sequence construction.
 * Computes r(x) such that u(x) = q(x)*v(x) + r(x)
 * 
 * @param u Dividend polynomial
 * @param v Divisor polynomial (leading coeff must be ±1)
 * @returns Remainder polynomial (trimmed)
 */
function polyMod(u: number[], v: number[]): number[] {
    const orderu = u.length - 1;
    const orderv = v.length - 1;
    const r = [...u];

    if (v[0] < 0) {
        for (let i = orderv + 1; i <= orderu; i += 2) {
            r[i] = -r[i];
        }
        for (let i = 0; i <= orderu - orderv; i++) {
            for (let j = i + 1; j <= orderv + i; j++) {
                r[j] = -r[j] - r[i] * v[j - i];
            }
        }
    } else {
        for (let i = 0; i <= orderu - orderv; i++) {
            for (let j = i + 1; j <= orderv + i; j++) {
                r[j] = r[j] - r[i] * v[j - i];
            }
        }
    }

    // Trim leading zeros
    let k = orderv - 1;
    while (k >= 0 && Math.abs(r[orderu - k]) < DBL_EPSILON) {
        r[orderu - k] = 0;
        k--;
    }

    // Return remainder from index (orderu - k) to end
    const start = k <= 0 ? orderu : orderu - k;
    return r.slice(start);
}

/**
 * Count sign variations in Sturm sequence at point x.
 */
function numSignVar(x: number, sturmSeqs: number[][]): number {
    let signVar = 0;
    let lasty = polyEval(sturmSeqs[0], x);

    for (let i = 1; i < sturmSeqs.length; i++) {
        const y = polyEval(sturmSeqs[i], x);
        if (lasty === 0 || lasty * y < 0) {
            signVar++;
        }
        lasty = y;
    }

    return signVar;
}

/**
 * Shrink interval to find a single root using safe Newton.
 */
function shrinkInterval(coeffs: number[], lbound: number, ubound: number, tol: number): number {
    const dcoeffs = polyDerivative(coeffs);
    const func = (x: number) => polyEval(coeffs, x);
    const dfunc = (x: number) => polyEval(dcoeffs, x);
    return safeNewton(func, dfunc, lbound, ubound, tol, 128);
}

/**
 * Recursively isolate roots using Sturm sequences.
 */
function recurIsolate(
    l: number, r: number,
    fl: number, fr: number,
    lnv: number, rnv: number,
    tol: number,
    sturmSeqs: number[][],
    roots: Set<number>
): void {
    const nrts = lnv - rnv;

    if (nrts === 0) return;

    if (nrts === 1) {
        if (fl * fr < 0) {
            roots.add(shrinkInterval(sturmSeqs[0], l, r, tol));
            return;
        } else {
            // Bisect for even multiplicity root
            let lo = l, hi = r;
            let flo = fl, fhi = fr;
            const maxIts = 128;

            for (let i = 0; i < maxIts; i++) {
                if (flo * fhi < 0) {
                    roots.add(shrinkInterval(sturmSeqs[1], lo, hi, tol));
                    return;
                }

                const m = (lo + hi) / 2;
                const fm = polyEval(sturmSeqs[0], m);

                if (fm === 0 || Math.abs(hi - lo) < tol) {
                    roots.add(m);
                    return;
                }

                if (lnv === numSignVar(m, sturmSeqs)) {
                    lo = m;
                    flo = fm;
                } else {
                    hi = m;
                    fhi = fm;
                }
            }
            roots.add((lo + hi) / 2);
            return;
        }
    }

    // More than one root - bisect
    const maxIts = 128;
    let lo = l, hi = r;
    let flo = fl, fhi = fr;
    let bias = 0;
    let biased = false;

    for (let i = 0; i < maxIts; i++) {
        const currentBias = biased ? bias : 0;
        let m: number;

        if (!biased) {
            m = (lo + hi) / 2;
        } else {
            m = (hi - lo) / Math.pow(2.0, currentBias + 1.0) + lo;
            biased = false;
        }

        const mnv = numSignVar(m, sturmSeqs);

        if (Math.abs(hi - lo) < tol) {
            roots.add(m);
            return;
        }

        const fm = polyEval(sturmSeqs[0], m);

        if (fm === 0) {
            bias++;
            biased = true;
        } else if (lnv !== mnv && rnv !== mnv) {
            // Split interval
            recurIsolate(lo, m, flo, fm, lnv, mnv, tol, sturmSeqs, roots);
            recurIsolate(m, hi, fm, fhi, mnv, rnv, tol, sturmSeqs, roots);
            return;
        } else if (lnv === mnv) {
            lo = m;
            flo = fm;
        } else {
            hi = m;
            fhi = fm;
        }
    }

    roots.add((lo + hi) / 2);
}

/**
 * Find all real roots of polynomial in interval using Sturm sequences.
 * 
 * This is the most robust method for finding all roots of high-order
 * polynomials. It first isolates intervals containing exactly one root,
 * then refines each using safe Newton-Raphson.
 * 
 * @param coeffs Polynomial coefficients [a_n, a_{n-1}, ..., a_0]
 * @param lbound Lower bound of search interval
 * @param ubound Upper bound of search interval
 * @param tol Tolerance for root accuracy
 * @returns Sorted array of roots in (lbound, ubound)
 * 
 * @example
 * ```typescript
 * // Find roots of x⁵ - 15x⁴ + 85x³ - 225x² + 274x - 120 in [0, 10]
 * const roots = isolateRealRoots([1, -15, 85, -225, 274, -120], 0, 10, 1e-10);
 * // Returns [1, 2, 3, 4, 5]
 * ```
 */
export function isolateRealRoots(
    coeffs: number[],
    lbound: number,
    ubound: number,
    tol: number
): number[] {
    if (coeffs.length === 0 || Math.abs(coeffs[0]) < DBL_EPSILON) {
        return [];
    }

    const order = coeffs.length - 1;

    // Create monic polynomial
    const monicCoeffs: number[] = new Array(order + 1);
    monicCoeffs[0] = 1.0;
    for (let i = 1; i <= order; i++) {
        monicCoeffs[i] = coeffs[i] / coeffs[0];
    }

    // Compute Cauchy's bound
    let maxAbsCoeff = 0;
    for (let i = 1; i <= order; i++) {
        maxAbsCoeff = Math.max(maxAbsCoeff, Math.abs(monicCoeffs[i]));
    }
    const rho = maxAbsCoeff + 1;

    // Tighten bounds
    lbound = Math.max(lbound, -rho);
    ubound = Math.min(ubound, rho);

    // Build Sturm sequence
    const sturmSeqs: number[][] = [];

    // p0 = polynomial
    sturmSeqs.push([...monicCoeffs]);

    // p1 = derivative (normalized)
    const derivCoeffs: number[] = new Array(order);
    for (let i = 0; i < order; i++) {
        derivCoeffs[i] = (order - i) * monicCoeffs[i] / order;
    }
    sturmSeqs.push(derivCoeffs);

    // Build rest of Sturm sequence via polynomial modulus
    let idx = 0;
    while (sturmSeqs[idx + 1].length > 1) {
        const remainder = polyMod(sturmSeqs[idx], sturmSeqs[idx + 1]);

        // Normalize and negate remainder
        if (remainder.length > 0 && remainder[0] !== 0) {
            const leadSign = remainder[0] > 0 ? -1 : 1;
            const leadAbs = Math.abs(remainder[0]);
            for (let i = 0; i < remainder.length; i++) {
                remainder[i] = (i === 0) ? leadSign : (-remainder[i] / leadAbs);
            }
        }

        sturmSeqs.push(remainder);
        idx++;

        // Safety check
        if (idx > 64) break;
    }

    // Find roots
    const roots = new Set<number>();

    const fl = polyEval(sturmSeqs[0], lbound);
    const fr = polyEval(sturmSeqs[0], ubound);
    const lnv = numSignVar(lbound, sturmSeqs);
    const rnv = numSignVar(ubound, sturmSeqs);

    if (lnv !== rnv) {
        recurIsolate(lbound, ubound, fl, fr, lnv, rnv, tol, sturmSeqs, roots);
    }

    return Array.from(roots).sort((a, b) => a - b);
}

/**
 * Find roots using companion matrix eigenvalues.
 * Alternative method that may be faster for moderate-order polynomials.
 * 
 * Note: This is a simplified implementation. For production use,
 * a proper eigenvalue solver like numpy.linalg.eig would be needed.
 * 
 * @param coeffs Polynomial coefficients
 * @param lbound Lower bound
 * @param ubound Upper bound  
 * @param tol Tolerance for imaginary part
 * @returns Real roots in interval
 */
export function eigenSolveRealRoots(
    coeffs: number[],
    lbound: number,
    ubound: number,
    tol: number
): number[] {
    // For now, delegate to Sturm sequence method
    // A full implementation would compute companion matrix eigenvalues
    return isolateRealRoots(coeffs, lbound, ubound, tol);
}

/**
 * Find maximum of polynomial magnitude in interval.
 * Useful for finding maximum velocity/acceleration along trajectories.
 * 
 * @param coeffs Polynomial coefficients
 * @param tStart Start of interval
 * @param tEnd End of interval
 * @param samples Number of initial samples
 * @returns Maximum absolute value and time at which it occurs
 */
export function findPolyMaxMagnitude(
    coeffs: number[],
    tStart: number,
    tEnd: number,
    samples: number = 20
): { maxVal: number; maxT: number } {
    // Find critical points (where derivative = 0)
    const deriv = polyDerivative(coeffs);
    const criticalPoints = isolateRealRoots(deriv, tStart, tEnd, 1e-10);

    // Evaluate at critical points and endpoints
    const checkPoints = [tStart, ...criticalPoints, tEnd];

    let maxVal = 0;
    let maxT = tStart;

    for (const t of checkPoints) {
        const val = Math.abs(polyEval(coeffs, t));
        if (val > maxVal) {
            maxVal = val;
            maxT = t;
        }
    }

    return { maxVal, maxT };
}

/**
 * Check if polynomial exceeds threshold in interval.
 * Efficiently determines if max|p(t)| > threshold for t in [tStart, tEnd].
 * 
 * @param coeffs Polynomial coefficients
 * @param threshold Maximum allowed value
 * @param tStart Start of interval
 * @param tEnd End of interval
 * @returns true if polynomial exceeds threshold
 */
export function polyExceedsThreshold(
    coeffs: number[],
    threshold: number,
    tStart: number,
    tEnd: number
): boolean {
    // Quick check at endpoints
    if (Math.abs(polyEval(coeffs, tStart)) > threshold) return true;
    if (Math.abs(polyEval(coeffs, tEnd)) > threshold) return true;

    // Check at critical points
    const deriv = polyDerivative(coeffs);
    const criticalPoints = isolateRealRoots(deriv, tStart, tEnd, 1e-8);

    for (const t of criticalPoints) {
        if (Math.abs(polyEval(coeffs, t)) > threshold) return true;
    }

    return false;
}
