/**
 * Test Utilities for Symbion Library
 * Provides common helpers for numerical testing
 */

/**
 * Check if two numbers are approximately equal
 */
export function isClose(a: number, b: number, rtol = 1e-5, atol = 1e-8): boolean {
    return Math.abs(a - b) <= atol + rtol * Math.abs(b);
}

/**
 * Check if two arrays are approximately equal element-wise
 */
export function arraysClose(
    a: number[],
    b: number[],
    rtol = 1e-5,
    atol = 1e-8
): boolean {
    if (a.length !== b.length) return false;
    return a.every((val, i) => isClose(val, b[i], rtol, atol));
}

/**
 * Calculate the L2 norm of a vector
 */
export function norm(v: number[]): number {
    return Math.sqrt(v.reduce((sum, x) => sum + x * x, 0));
}

/**
 * Calculate the relative error between two numbers
 */
export function relativeError(actual: number, expected: number): number {
    if (expected === 0) return Math.abs(actual);
    return Math.abs((actual - expected) / expected);
}

/**
 * Generate a random number in a range
 */
export function randomInRange(min: number, max: number): number {
    return min + Math.random() * (max - min);
}

/**
 * Generate an array of random numbers
 */
export function randomArray(length: number, min = 0, max = 1): number[] {
    return Array.from({ length }, () => randomInRange(min, max));
}

/**
 * Calculate mean of an array
 */
export function mean(arr: number[]): number {
    return arr.reduce((a, b) => a + b, 0) / arr.length;
}

/**
 * Calculate variance of an array
 */
export function variance(arr: number[]): number {
    const m = mean(arr);
    return arr.reduce((sum, x) => sum + (x - m) ** 2, 0) / arr.length;
}

/**
 * Calculate standard deviation of an array
 */
export function std(arr: number[]): number {
    return Math.sqrt(variance(arr));
}

/**
 * Matrix multiplication for 2D arrays
 */
export function matMul(A: number[][], B: number[][]): number[][] {
    const m = A.length;
    const n = B[0].length;
    const k = B.length;
    const C: number[][] = Array.from({ length: m }, () => Array(n).fill(0));

    for (let i = 0; i < m; i++) {
        for (let j = 0; j < n; j++) {
            for (let l = 0; l < k; l++) {
                C[i][j] += A[i][l] * B[l][j];
            }
        }
    }
    return C;
}

/**
 * Transpose a 2D matrix
 */
export function transpose(A: number[][]): number[][] {
    const m = A.length;
    const n = A[0].length;
    return Array.from({ length: n }, (_, j) =>
        Array.from({ length: m }, (_, i) => A[i][j])
    );
}

/**
 * Check if a matrix is approximately symmetric
 */
export function isSymmetric(A: number[][], tol = 1e-10): boolean {
    const n = A.length;
    for (let i = 0; i < n; i++) {
        for (let j = i + 1; j < n; j++) {
            if (Math.abs(A[i][j] - A[j][i]) > tol) return false;
        }
    }
    return true;
}

/**
 * Identity matrix
 */
export function eye(n: number): number[][] {
    return Array.from({ length: n }, (_, i) =>
        Array.from({ length: n }, (_, j) => (i === j ? 1 : 0))
    );
}

/**
 * Zeros matrix
 */
export function zeros(m: number, n: number): number[][] {
    return Array.from({ length: m }, () => Array(n).fill(0));
}

/**
 * Calculate Frobenius norm of a matrix
 */
export function frobeniusNorm(A: number[][]): number {
    return Math.sqrt(
        A.reduce((sum, row) => sum + row.reduce((s, x) => s + x * x, 0), 0)
    );
}

/**
 * Check if two matrices are approximately equal
 */
export function matricesClose(
    A: number[][],
    B: number[][],
    tol = 1e-10
): boolean {
    if (A.length !== B.length || A[0].length !== B[0].length) return false;
    for (let i = 0; i < A.length; i++) {
        for (let j = 0; j < A[0].length; j++) {
            if (Math.abs(A[i][j] - B[i][j]) > tol) return false;
        }
    }
    return true;
}

/**
 * Generate a positive definite matrix (for testing decompositions)
 */
export function randomPositiveDefinite(n: number): number[][] {
    // Create A = R^T * R where R is random
    const R = Array.from({ length: n }, () =>
        Array.from({ length: n }, () => Math.random())
    );
    const RT = transpose(R);
    return matMul(RT, R);
}

/**
 * Type guard for checking if a value is a number
 */
export function isNumber(x: unknown): x is number {
    return typeof x === 'number' && !Number.isNaN(x);
}

/**
 * Type guard for checking if a value is a finite number
 */
export function isFiniteNumber(x: unknown): x is number {
    return typeof x === 'number' && Number.isFinite(x);
}

/**
 * Seeded random number generator for deterministic tests
 */
export class SeededRNG {
    private seed: number;

    constructor(seed: number = 42) {
        this.seed = seed;
    }

    next(): number {
        this.seed = (this.seed * 1103515245 + 12345) % 2147483648;
        return this.seed / 2147483648;
    }

    nextInRange(min: number, max: number): number {
        return min + this.next() * (max - min);
    }

    nextInt(min: number, max: number): number {
        return Math.floor(this.nextInRange(min, max + 1));
    }

    nextArray(length: number, min = 0, max = 1): number[] {
        return Array.from({ length }, () => this.nextInRange(min, max));
    }

    reset(seed?: number): void {
        this.seed = seed ?? 42;
    }
}

/**
 * Create a mock RNG with a specific seed for reproducible tests
 */
export function createMockRNG(seed: number = 42): SeededRNG {
    return new SeededRNG(seed);
}

/**
 * Assert that a function throws an error
 */
export function assertThrows(
    fn: () => void,
    expectedMessage?: string | RegExp
): Error | null {
    try {
        fn();
        return null; // Did not throw
    } catch (error) {
        if (expectedMessage) {
            const message = error instanceof Error ? error.message : String(error);
            if (typeof expectedMessage === 'string') {
                if (!message.includes(expectedMessage)) {
                    return null;
                }
            } else if (!expectedMessage.test(message)) {
                return null;
            }
        }
        return error instanceof Error ? error : new Error(String(error));
    }
}

/**
 * Complex number representation for testing
 */
export interface Complex {
    real: number;
    imag: number;
}

/**
 * Check if two complex numbers are approximately equal
 */
export function complexClose(a: Complex, b: Complex, tol = 1e-10): boolean {
    return Math.abs(a.real - b.real) <= tol && Math.abs(a.imag - b.imag) <= tol;
}

/**
 * Check if two complex matrices are approximately equal
 */
export function complexMatricesClose(
    A: { real: number[][]; imag: number[][] },
    B: { real: number[][]; imag: number[][] },
    tol = 1e-10
): boolean {
    if (A.real.length !== B.real.length || A.real[0].length !== B.real[0].length) {
        return false;
    }
    for (let i = 0; i < A.real.length; i++) {
        for (let j = 0; j < A.real[0].length; j++) {
            if (
                Math.abs(A.real[i][j] - B.real[i][j]) > tol ||
                Math.abs(A.imag[i][j] - B.imag[i][j]) > tol
            ) {
                return false;
            }
        }
    }
    return true;
}

/**
 * Estimate matrix rank using SVD-like approach (simplified)
 * Returns approximate rank based on singular value thresholding
 */
export function estimateMatrixRank(A: number[][], tol = 1e-10): number {
    const m = A.length;
    const n = A[0].length;
    const minDim = Math.min(m, n);

    // Simple approach: count non-zero pivots from Gaussian elimination
    const B = A.map((row) => [...row]); // Copy matrix

    let rank = 0;
    for (let col = 0; col < n && rank < m; col++) {
        // Find pivot
        let maxRow = rank;
        for (let row = rank + 1; row < m; row++) {
            if (Math.abs(B[row][col]) > Math.abs(B[maxRow][col])) {
                maxRow = row;
            }
        }

        if (Math.abs(B[maxRow][col]) < tol) {
            continue; // No valid pivot in this column
        }

        // Swap rows
        [B[rank], B[maxRow]] = [B[maxRow], B[rank]];

        // Eliminate below
        for (let row = rank + 1; row < m; row++) {
            const factor = B[row][col] / B[rank][col];
            for (let j = col; j < n; j++) {
                B[row][j] -= factor * B[rank][j];
            }
        }

        rank++;
    }

    return rank;
}

/**
 * Generate standard test data for various scenarios
 */
export const generateTestData = {
    /**
     * Generate a random vector
     */
    vector(n: number, seed?: number): number[] {
        const rng = new SeededRNG(seed);
        return rng.nextArray(n, -1, 1);
    },

    /**
     * Generate a random matrix
     */
    matrix(m: number, n: number, seed?: number): number[][] {
        const rng = new SeededRNG(seed);
        return Array.from({ length: m }, () => rng.nextArray(n, -1, 1));
    },

    /**
     * Generate a diagonal matrix
     */
    diagonal(n: number, values?: number[]): number[][] {
        const diag = values || Array.from({ length: n }, (_, i) => i + 1);
        return Array.from({ length: n }, (_, i) =>
            Array.from({ length: n }, (_, j) => (i === j ? diag[i] : 0))
        );
    },

    /**
     * Generate a symmetric positive definite matrix
     */
    spd(n: number, seed?: number): number[][] {
        const rng = new SeededRNG(seed);
        const A = Array.from({ length: n }, () => rng.nextArray(n, -1, 1));
        // A^T * A is always positive semi-definite, add small diagonal to ensure positive definite
        const AT = transpose(A);
        const ATA = matMul(AT, A);
        for (let i = 0; i < n; i++) {
            ATA[i][i] += 0.1; // Ensure positive definiteness
        }
        return ATA;
    },

    /**
     * Generate a complex matrix
     */
    complexMatrix(
        m: number,
        n: number,
        seed?: number
    ): { real: number[][]; imag: number[][] } {
        const rng = new SeededRNG(seed);
        return {
            real: Array.from({ length: m }, () => rng.nextArray(n, -1, 1)),
            imag: Array.from({ length: m }, () => rng.nextArray(n, -1, 1)),
        };
    },

    /**
     * Generate an ill-conditioned matrix
     */
    illConditioned(n: number, conditionNumber: number = 1e6): number[][] {
        // Create a diagonal matrix with specified condition number
        const diag = Array.from({ length: n }, (_, i) =>
            Math.pow(conditionNumber, -i / (n - 1))
        );
        return generateTestData.diagonal(n, diag);
    },

    /**
     * Generate a near-singular matrix
     */
    nearSingular(n: number, epsilon: number = 1e-10): number[][] {
        const A = eye(n);
        A[n - 1][n - 1] = epsilon;
        return A;
    },
};

/**
 * Performance test wrapper
 */
export interface PerformanceResult {
    elapsed: number;
    iterations: number;
    avgTime: number;
    opsPerSecond: number;
}

/**
 * Run a performance test and return timing statistics
 */
export function performanceTest(
    fn: () => void,
    options: { iterations?: number; warmup?: number } = {}
): PerformanceResult {
    const { iterations = 100, warmup = 10 } = options;

    // Warmup
    for (let i = 0; i < warmup; i++) {
        fn();
    }

    // Timed runs
    const start = performance.now();
    for (let i = 0; i < iterations; i++) {
        fn();
    }
    const elapsed = performance.now() - start;

    return {
        elapsed,
        iterations,
        avgTime: elapsed / iterations,
        opsPerSecond: (iterations / elapsed) * 1000,
    };
}

/**
 * Assert that a value is within a range
 */
export function inRange(value: number, min: number, max: number): boolean {
    return value >= min && value <= max;
}

/**
 * Create a sparse matrix representation for testing
 */
export function createSparseMatrix(
    m: number,
    n: number,
    density: number = 0.1,
    seed?: number
): number[][] {
    const rng = new SeededRNG(seed);
    return Array.from({ length: m }, () =>
        Array.from({ length: n }, () =>
            rng.next() < density ? rng.nextInRange(-1, 1) : 0
        )
    );
}
