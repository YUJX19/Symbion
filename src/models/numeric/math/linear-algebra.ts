/**
 * @module math/linear-algebra
 * @description Lightweight linear algebra utilities for optimization and planning algorithms.
 * Provides vector and matrix operations without external dependencies.
 */

// ==================== Vector Operations ====================

/**
 * Compute the dot product of two vectors
 */
export function dot(a: number[], b: number[]): number {
    let sum = 0;
    for (let i = 0; i < a.length; i++) {
        sum += a[i] * b[i];
    }
    return sum;
}

/**
 * Compute the Euclidean norm (L2 norm) of a vector
 */
export function norm(v: number[]): number {
    return Math.sqrt(squaredNorm(v));
}

/**
 * Compute the squared Euclidean norm of a vector
 */
export function squaredNorm(v: number[]): number {
    let sum = 0;
    for (let i = 0; i < v.length; i++) {
        sum += v[i] * v[i];
    }
    return sum;
}

/**
 * Compute the infinity norm (max absolute value) of a vector
 */
export function infNorm(v: number[]): number {
    let maxVal = 0;
    for (let i = 0; i < v.length; i++) {
        const absVal = Math.abs(v[i]);
        if (absVal > maxVal) maxVal = absVal;
    }
    return maxVal;
}

/**
 * Add two vectors: a + b
 */
export function add(a: number[], b: number[]): number[] {
    const result: number[] = new Array(a.length);
    for (let i = 0; i < a.length; i++) {
        result[i] = a[i] + b[i];
    }
    return result;
}

/**
 * Subtract two vectors: a - b
 */
export function subtract(a: number[], b: number[]): number[] {
    const result: number[] = new Array(a.length);
    for (let i = 0; i < a.length; i++) {
        result[i] = a[i] - b[i];
    }
    return result;
}

/**
 * Scale a vector by a scalar: s * v
 */
export function scale(v: number[], s: number): number[] {
    const result: number[] = new Array(v.length);
    for (let i = 0; i < v.length; i++) {
        result[i] = v[i] * s;
    }
    return result;
}

/**
 * Negate a vector: -v
 */
export function negate(v: number[]): number[] {
    return scale(v, -1);
}

/**
 * Create a copy of a vector
 */
export function copyVector(v: number[]): number[] {
    return [...v];
}

/**
 * Create a zero vector of given size
 */
export function zeros(n: number): number[] {
    return new Array(n).fill(0);
}

/**
 * Element-wise absolute value
 */
export function absVector(v: number[]): number[] {
    return v.map(Math.abs);
}

/**
 * Linear combination: a + s * b
 */
export function axpy(a: number[], s: number, b: number[]): number[] {
    const result: number[] = new Array(a.length);
    for (let i = 0; i < a.length; i++) {
        result[i] = a[i] + s * b[i];
    }
    return result;
}

// ==================== 3D Vector Operations ====================

export type Vec3 = [number, number, number];

/**
 * Type alias for Vector3Array compatibility with trajectory module
 */
export type Vector3 = Vec3;

/**
 * 3D cross product: a × b
 */
export function cross3(a: Vec3, b: Vec3): Vec3 {
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    ];
}

/**
 * 3D dot product: a · b
 */
export function dot3(a: Vec3, b: Vec3): number {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/**
 * Normalize a 3D vector to unit length
 */
export function normalize3(v: Vec3): Vec3 {
    const n = Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n < 1e-12) return [0, 0, 0];
    return [v[0] / n, v[1] / n, v[2] / n];
}

/**
 * 3D vector magnitude (Euclidean length)
 */
export function magnitude3(v: Vec3): number {
    return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

/**
 * 3D vector addition: a + b
 */
export function add3(a: Vec3, b: Vec3): Vec3 {
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

/**
 * 3D vector subtraction: a - b
 */
export function sub3(a: Vec3, b: Vec3): Vec3 {
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

/**
 * 3D vector scalar multiplication: s * v
 */
export function scale3(v: Vec3, s: number): Vec3 {
    return [v[0] * s, v[1] * s, v[2] * s];
}

/**
 * Distance between two 3D points
 */
export function distance3(a: Vec3, b: Vec3): number {
    return magnitude3(sub3(a, b));
}

/**
 * Linear interpolation between two vectors
 */
export function lerp3(a: Vec3, b: Vec3, t: number): Vec3 {
    return [
        a[0] + t * (b[0] - a[0]),
        a[1] + t * (b[1] - a[1]),
        a[2] + t * (b[2] - a[2])
    ];
}

// ==================== Unified Vector Ops (namespace-style export) ====================

/**
 * Unified vector operations for use across the library.
 * All functions work with Vec3 = [number, number, number].
 */
export const vec3 = {
    /** Cross product: a × b */
    cross: cross3,
    /** Dot product: a · b */
    dot: dot3,
    /** Normalize to unit length */
    normalize: normalize3,
    /** Euclidean magnitude */
    magnitude: magnitude3,
    /** Vector addition: a + b */
    add: add3,
    /** Vector subtraction: a - b */
    sub: sub3,
    /** Scalar multiplication: s * v */
    scale: scale3,
    /** Distance between two points */
    distance: distance3,
    /** Linear interpolation */
    lerp: lerp3,
    /** Create a zero vector */
    zero: (): Vec3 => [0, 0, 0],
    /** Clone a vector */
    clone: (v: Vec3): Vec3 => [v[0], v[1], v[2]],
    /** Check if vectors are equal within tolerance */
    equals: (a: Vec3, b: Vec3, eps: number = 1e-9): boolean =>
        Math.abs(a[0] - b[0]) < eps &&
        Math.abs(a[1] - b[1]) < eps &&
        Math.abs(a[2] - b[2]) < eps,
};


// ==================== Matrix Operations ====================

export type Mat3 = [[number, number, number], [number, number, number], [number, number, number]];

/**
 * Create a 3x3 identity matrix
 */
export function identity3(): Mat3 {
    return [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
    ];
}

/**
 * Create a 3x3 zero matrix
 */
export function zeroMat3(): Mat3 {
    return [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ];
}

/**
 * Multiply a 3x3 matrix by a 3D vector
 */
export function mulMatVec3(M: Mat3, v: Vec3): Vec3 {
    return [
        M[0][0] * v[0] + M[0][1] * v[1] + M[0][2] * v[2],
        M[1][0] * v[0] + M[1][1] * v[1] + M[1][2] * v[2],
        M[2][0] * v[0] + M[2][1] * v[1] + M[2][2] * v[2]
    ];
}

/**
 * Multiply two 3x3 matrices
 */
export function mulMat3(A: Mat3, B: Mat3): Mat3 {
    const result = zeroMat3();
    for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
            for (let k = 0; k < 3; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return result;
}

/**
 * Transpose a 3x3 matrix
 */
export function transpose3(M: Mat3): Mat3 {
    return [
        [M[0][0], M[1][0], M[2][0]],
        [M[0][1], M[1][1], M[2][1]],
        [M[0][2], M[1][2], M[2][2]]
    ];
}

/**
 * Compute the determinant of a 3x3 matrix
 */
export function det3(M: Mat3): number {
    return (
        M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
        M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
        M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])
    );
}

/**
 * Cholesky decomposition of a 3x3 positive definite matrix.
 * Returns lower triangular matrix L such that A = L * L^T
 */
export function cholesky3(A: Mat3): Mat3 {
    const L = zeroMat3();

    L[0][0] = Math.sqrt(A[0][0]);
    L[1][0] = 0.5 * (A[0][1] + A[1][0]) / L[0][0];
    L[1][1] = Math.sqrt(A[1][1] - L[1][0] * L[1][0]);
    L[2][0] = 0.5 * (A[0][2] + A[2][0]) / L[0][0];
    L[2][1] = (0.5 * (A[1][2] + A[2][1]) - L[2][0] * L[1][0]) / L[1][1];
    L[2][2] = Math.sqrt(A[2][2] - L[2][0] * L[2][0] - L[2][1] * L[2][1]);

    return L;
}

/**
 * Simplified SVD for a 3x3 matrix.
 * Returns { U, S, V } where A ≈ U * diag(S) * V^T
 * Uses Jacobi iteration for symmetric matrices (for ellipsoid fitting).
 */
export function svd3(A: Mat3): { U: Mat3; S: Vec3; V: Mat3 } {
    // For FIRI, we only need SVD of L (lower triangular from Cholesky).
    // We use a simplified approach: compute A^T * A eigenvalues
    const AtA = mulMat3(transpose3(A), A);

    // Power iteration to find principal components (simplified)
    // For a more robust implementation, use Jacobi rotation
    const eigenResult = jacobi3(AtA);

    const S: Vec3 = [
        Math.sqrt(Math.max(0, eigenResult.eigenvalues[0])),
        Math.sqrt(Math.max(0, eigenResult.eigenvalues[1])),
        Math.sqrt(Math.max(0, eigenResult.eigenvalues[2]))
    ];

    const V = eigenResult.eigenvectors;

    // Compute U = A * V * S^{-1}
    const U = zeroMat3();
    for (let j = 0; j < 3; j++) {
        if (S[j] > 1e-10) {
            const vj: Vec3 = [V[0][j], V[1][j], V[2][j]];
            const Avj = mulMatVec3(A, vj);
            for (let i = 0; i < 3; i++) {
                U[i][j] = Avj[i] / S[j];
            }
        }
    }

    return { U, S, V };
}

/**
 * Jacobi eigenvalue algorithm for a 3x3 symmetric matrix.
 */
function jacobi3(A: Mat3): { eigenvalues: Vec3; eigenvectors: Mat3 } {
    const a: Mat3 = [
        [A[0][0], A[0][1], A[0][2]],
        [A[1][0], A[1][1], A[1][2]],
        [A[2][0], A[2][1], A[2][2]]
    ];
    const v = identity3();

    const maxIter = 50;
    const eps = 1e-10;

    for (let iter = 0; iter < maxIter; iter++) {
        // Find largest off-diagonal element
        let maxVal = 0;
        let p = 0, q = 1;
        for (let i = 0; i < 3; i++) {
            for (let j = i + 1; j < 3; j++) {
                if (Math.abs(a[i][j]) > maxVal) {
                    maxVal = Math.abs(a[i][j]);
                    p = i;
                    q = j;
                }
            }
        }

        if (maxVal < eps) break;

        // Jacobi rotation
        const theta = 0.5 * Math.atan2(2 * a[p][q], a[q][q] - a[p][p]);
        const c = Math.cos(theta);
        const s = Math.sin(theta);

        // Apply rotation to a
        const app = a[p][p];
        const aqq = a[q][q];
        const apq = a[p][q];

        a[p][p] = c * c * app - 2 * s * c * apq + s * s * aqq;
        a[q][q] = s * s * app + 2 * s * c * apq + c * c * aqq;
        a[p][q] = 0;
        a[q][p] = 0;

        for (let i = 0; i < 3; i++) {
            if (i !== p && i !== q) {
                const aip = a[i][p];
                const aiq = a[i][q];
                a[i][p] = c * aip - s * aiq;
                a[p][i] = a[i][p];
                a[i][q] = s * aip + c * aiq;
                a[q][i] = a[i][q];
            }
        }

        // Apply rotation to v (eigenvectors)
        for (let i = 0; i < 3; i++) {
            const vip = v[i][p];
            const viq = v[i][q];
            v[i][p] = c * vip - s * viq;
            v[i][q] = s * vip + c * viq;
        }
    }

    // Sort eigenvalues in descending order
    const eigenvalues: Vec3 = [a[0][0], a[1][1], a[2][2]];
    const indices = [0, 1, 2].sort((i, j) => eigenvalues[j] - eigenvalues[i]);

    const sortedEigenvalues: Vec3 = [
        eigenvalues[indices[0]],
        eigenvalues[indices[1]],
        eigenvalues[indices[2]]
    ];

    const sortedEigenvectors: Mat3 = [
        [v[0][indices[0]], v[0][indices[1]], v[0][indices[2]]],
        [v[1][indices[0]], v[1][indices[1]], v[1][indices[2]]],
        [v[2][indices[0]], v[2][indices[1]], v[2][indices[2]]]
    ];

    return { eigenvalues: sortedEigenvalues, eigenvectors: sortedEigenvectors };
}

// ==================== Matrix-X Operations ====================

/**
 * Create a 2D array (matrix) filled with zeros
 */
export function zerosMatrix(rows: number, cols: number): number[][] {
    const result: number[][] = [];
    for (let i = 0; i < rows; i++) {
        result.push(new Array(cols).fill(0));
    }
    return result;
}

/**
 * Matrix-vector multiplication for general matrices
 */
export function mulMatVec(M: number[][], v: number[]): number[] {
    const rows = M.length;
    const result: number[] = new Array(rows).fill(0);
    for (let i = 0; i < rows; i++) {
        for (let j = 0; j < v.length; j++) {
            result[i] += M[i][j] * v[j];
        }
    }
    return result;
}

/**
 * Compute row-wise norms of a matrix
 */
export function rowNorms(M: number[][]): number[] {
    return M.map(row => norm(row));
}

/**
 * Normalize matrix rows for SDLP
 */
export function normalizeRows(M: number[][]): number[][] {
    return M.map(row => {
        const n = norm(row);
        return n > 1e-12 ? scale(row, 1 / n) : row;
    });
}
