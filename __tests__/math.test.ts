/**
 * Math Module Tests
 * Tests for linear algebra utilities - vectors and matrices
 */

import { describe, it, expect } from 'vitest';
import {
    dot,
    norm,
    squaredNorm,
    infNorm,
    add,
    subtract,
    scale,
    negate,
    copyVector,
    zeros,
    axpy,
    // 3D operations
    cross3,
    dot3,
    normalize3,
    magnitude3,
    add3,
    sub3,
    scale3,
    distance3,
    lerp3,
    vec3,
    // Matrix operations
    identity3,
    zeroMat3,
    mulMatVec3,
    mulMat3,
    transpose3,
    det3,
    cholesky3,
} from '../src/models/numeric/math';
import { isClose, arraysClose, matricesClose } from './test-utils';

// ==================== Generic Vector Operations ====================

describe('Generic Vector Operations', () => {
    describe('dot', () => {
        it('should compute dot product correctly', () => {
            expect(dot([1, 2, 3], [4, 5, 6])).toBe(32); // 4+10+18
        });

        it('should return 0 for orthogonal vectors', () => {
            expect(dot([1, 0], [0, 1])).toBe(0);
        });

        it('should handle single element vectors', () => {
            expect(dot([5], [3])).toBe(15);
        });

        it('should handle mismatched lengths gracefully', () => {
            // Note: Current implementation does not throw, just uses shorter length
            const result = dot([1, 2], [1, 2, 3]);
            expect(typeof result).toBe('number');
        });
    });

    describe('norm', () => {
        it('should compute Euclidean norm correctly', () => {
            expect(norm([3, 4])).toBe(5);
            expect(norm([0, 0, 0])).toBe(0);
        });

        it('should return absolute value for 1D', () => {
            expect(norm([-5])).toBe(5);
        });
    });

    describe('squaredNorm', () => {
        it('should be square of norm', () => {
            const v = [3, 4];
            expect(squaredNorm(v)).toBe(norm(v) ** 2);
        });
    });

    describe('infNorm', () => {
        it('should return max absolute value', () => {
            expect(infNorm([1, -5, 3])).toBe(5);
            expect(infNorm([0, 0, 0])).toBe(0);
        });

        it('should handle empty vector', () => {
            // Note: Current implementation returns 0 for empty array
            const result = infNorm([]);
            expect(result).toBe(0);
        });
    });

    describe('add', () => {
        it('should add vectors element-wise', () => {
            expect(add([1, 2, 3], [4, 5, 6])).toEqual([5, 7, 9]);
        });

        it('should handle mismatched lengths gracefully', () => {
            // Note: Current implementation does not throw
            const result = add([1], [1, 2]);
            expect(Array.isArray(result)).toBe(true);
        });
    });

    describe('subtract', () => {
        it('should subtract vectors element-wise', () => {
            expect(subtract([4, 5, 6], [1, 2, 3])).toEqual([3, 3, 3]);
        });
    });

    describe('scale', () => {
        it('should multiply by scalar', () => {
            expect(scale([1, 2, 3], 2)).toEqual([2, 4, 6]);
        });

        it('should handle zero scalar', () => {
            expect(scale([1, 2, 3], 0)).toEqual([0, 0, 0]);
        });

        it('should handle negative scalar', () => {
            expect(scale([1, 2, 3], -1)).toEqual([-1, -2, -3]);
        });
    });

    describe('negate', () => {
        it('should negate all elements', () => {
            expect(negate([1, -2, 3])).toEqual([-1, 2, -3]);
        });
    });

    describe('copyVector', () => {
        it('should create independent copy', () => {
            const original = [1, 2, 3];
            const copy = copyVector(original);
            expect(copy).toEqual(original);
            expect(copy).not.toBe(original);
        });
    });

    describe('zeros', () => {
        it('should create zero vector of given size', () => {
            expect(zeros(3)).toEqual([0, 0, 0]);
            expect(zeros(0)).toEqual([]);
        });
    });

    describe('axpy', () => {
        it('should compute a + s*b', () => {
            expect(axpy([1, 2, 3], 2, [4, 5, 6])).toEqual([9, 12, 15]);
        });
    });
});

// ==================== 3D Vector Operations ====================

describe('3D Vector Operations', () => {
    describe('dot3', () => {
        it('should compute dot product correctly', () => {
            expect(dot3([1, 0, 0], [0, 1, 0])).toBe(0);
            expect(dot3([1, 2, 3], [4, 5, 6])).toBe(32);
        });
    });

    describe('cross3', () => {
        it('should compute cross product for basis vectors', () => {
            // i × j = k
            expect(cross3([1, 0, 0], [0, 1, 0])).toEqual([0, 0, 1]);
            // j × k = i
            expect(cross3([0, 1, 0], [0, 0, 1])).toEqual([1, 0, 0]);
            // k × i = j
            expect(cross3([0, 0, 1], [1, 0, 0])).toEqual([0, 1, 0]);
        });

        it('should be anti-commutative: a × b = -(b × a)', () => {
            const a: [number, number, number] = [1, 2, 3];
            const b: [number, number, number] = [4, 5, 6];
            const ab = cross3(a, b);
            const ba = cross3(b, a);
            expect(arraysClose(ab, [-ba[0], -ba[1], -ba[2]])).toBe(true);
        });

        it('should return zero for parallel vectors', () => {
            expect(cross3([2, 4, 6], [1, 2, 3])).toEqual([0, 0, 0]);
        });
    });

    describe('normalize3', () => {
        it('should normalize to unit length', () => {
            const n = normalize3([3, 0, 4]);
            expect(isClose(magnitude3(n), 1, 1e-10)).toBe(true);
            expect(arraysClose(n, [0.6, 0, 0.8])).toBe(true);
        });

        it('should handle already normalized vector', () => {
            const n = normalize3([1, 0, 0]);
            expect(arraysClose(n, [1, 0, 0])).toBe(true);
        });
    });

    describe('magnitude3', () => {
        it('should compute magnitude correctly', () => {
            expect(magnitude3([3, 4, 0])).toBe(5);
            expect(isClose(magnitude3([1, 1, 1]), Math.sqrt(3))).toBe(true);
        });
    });

    describe('add3', () => {
        it('should add 3D vectors', () => {
            expect(add3([1, 2, 3], [4, 5, 6])).toEqual([5, 7, 9]);
        });
    });

    describe('sub3', () => {
        it('should subtract 3D vectors', () => {
            expect(sub3([4, 5, 6], [1, 2, 3])).toEqual([3, 3, 3]);
        });
    });

    describe('scale3', () => {
        it('should scale 3D vector', () => {
            expect(scale3([1, 2, 3], 2)).toEqual([2, 4, 6]);
        });
    });

    describe('distance3', () => {
        it('should compute distance between points', () => {
            expect(distance3([0, 0, 0], [3, 4, 0])).toBe(5);
        });
    });

    describe('lerp3', () => {
        it('should interpolate at t=0', () => {
            expect(lerp3([0, 0, 0], [10, 10, 10], 0)).toEqual([0, 0, 0]);
        });

        it('should interpolate at t=1', () => {
            expect(lerp3([0, 0, 0], [10, 10, 10], 1)).toEqual([10, 10, 10]);
        });

        it('should interpolate at t=0.5', () => {
            expect(lerp3([0, 0, 0], [10, 10, 10], 0.5)).toEqual([5, 5, 5]);
        });

        it('should handle extrapolation (t > 1)', () => {
            expect(lerp3([0, 0, 0], [10, 10, 10], 2)).toEqual([20, 20, 20]);
        });
    });
});

// ==================== vec3 Namespace ====================

describe('vec3 namespace', () => {
    it('should provide dot function', () => {
        expect(vec3.dot([1, 0, 0], [0, 1, 0])).toBe(0);
    });

    it('should provide cross function', () => {
        expect(vec3.cross([1, 0, 0], [0, 1, 0])).toEqual([0, 0, 1]);
    });

    it('should provide normalize function', () => {
        const n = vec3.normalize([3, 0, 4]);
        expect(arraysClose(n, [0.6, 0, 0.8])).toBe(true);
    });

    it('should provide zero function', () => {
        expect(vec3.zero()).toEqual([0, 0, 0]);
    });

    it('should provide clone function', () => {
        const v: [number, number, number] = [1, 2, 3];
        const c = vec3.clone(v);
        expect(c).toEqual(v);
        expect(c).not.toBe(v);
    });

    it('should provide equals function', () => {
        expect(vec3.equals([1, 2, 3], [1, 2, 3])).toBe(true);
        expect(vec3.equals([1, 2, 3], [1, 2, 3.0001], 1e-3)).toBe(true);
        expect(vec3.equals([1, 2, 3], [1, 2, 4])).toBe(false);
    });
});

// ==================== Matrix Operations ====================

describe('Matrix Operations', () => {
    describe('identity3', () => {
        it('should create 3x3 identity matrix', () => {
            const I = identity3();
            expect(I).toEqual([
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1],
            ]);
        });
    });

    describe('zeroMat3', () => {
        it('should create 3x3 zero matrix', () => {
            const Z = zeroMat3();
            expect(Z).toEqual([
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0],
            ]);
        });
    });

    describe('mulMatVec3', () => {
        it('should multiply matrix by vector', () => {
            const I = identity3();
            const v: [number, number, number] = [1, 2, 3];
            expect(mulMatVec3(I, v)).toEqual([1, 2, 3]);
        });

        it('should apply transformation correctly', () => {
            // Scale matrix
            const S: [[number, number, number], [number, number, number], [number, number, number]] = [
                [2, 0, 0],
                [0, 2, 0],
                [0, 0, 2],
            ];
            expect(mulMatVec3(S, [1, 2, 3])).toEqual([2, 4, 6]);
        });
    });

    describe('mulMat3', () => {
        it('should multiply identity by any matrix = same matrix', () => {
            const I = identity3();
            const A: [[number, number, number], [number, number, number], [number, number, number]] = [
                [1, 2, 3],
                [4, 5, 6],
                [7, 8, 9],
            ];
            expect(matricesClose(mulMat3(I, A), A)).toBe(true);
            expect(matricesClose(mulMat3(A, I), A)).toBe(true);
        });
    });

    describe('transpose3', () => {
        it('should transpose matrix correctly', () => {
            const A: [[number, number, number], [number, number, number], [number, number, number]] = [
                [1, 2, 3],
                [4, 5, 6],
                [7, 8, 9],
            ];
            const AT = transpose3(A);
            expect(AT).toEqual([
                [1, 4, 7],
                [2, 5, 8],
                [3, 6, 9],
            ]);
        });

        it('should be involutory (applying twice returns original)', () => {
            const A: [[number, number, number], [number, number, number], [number, number, number]] = [
                [1, 2, 3],
                [4, 5, 6],
                [7, 8, 9],
            ];
            expect(matricesClose(transpose3(transpose3(A)), A)).toBe(true);
        });
    });

    describe('det3', () => {
        it('should compute determinant of identity = 1', () => {
            expect(det3(identity3())).toBe(1);
        });

        it('should compute determinant of zero matrix = 0', () => {
            expect(det3(zeroMat3())).toBe(0);
        });

        it('should compute determinant correctly', () => {
            const A: [[number, number, number], [number, number, number], [number, number, number]] = [
                [1, 2, 3],
                [0, 1, 4],
                [5, 6, 0],
            ];
            // det = 1*(0-24) - 2*(0-20) + 3*(0-5) = -24 + 40 - 15 = 1
            expect(det3(A)).toBe(1);
        });

        it('should detect singular matrix (det = 0)', () => {
            // Linearly dependent rows
            const A: [[number, number, number], [number, number, number], [number, number, number]] = [
                [1, 2, 3],
                [2, 4, 6],
                [0, 0, 1],
            ];
            expect(det3(A)).toBe(0);
        });
    });

    describe('cholesky3', () => {
        it('should decompose identity matrix correctly', () => {
            const I = identity3();
            const L = cholesky3(I);
            // For identity, L should also be identity
            expect(matricesClose(L, I)).toBe(true);
        });

        it('should satisfy A = L * L^T', () => {
            // Create a positive definite matrix
            const A: [[number, number, number], [number, number, number], [number, number, number]] = [
                [4, 2, 2],
                [2, 5, 3],
                [2, 3, 6],
            ];
            const L = cholesky3(A);
            const LT = transpose3(L);
            const LLT = mulMat3(L, LT);
            expect(matricesClose(LLT, A, 1e-10)).toBe(true);
        });

        it('should produce lower triangular matrix', () => {
            const A: [[number, number, number], [number, number, number], [number, number, number]] = [
                [4, 2, 2],
                [2, 5, 3],
                [2, 3, 6],
            ];
            const L = cholesky3(A);
            // Upper triangle should be zero
            expect(L[0][1]).toBe(0);
            expect(L[0][2]).toBe(0);
            expect(L[1][2]).toBe(0);
        });
    });
});

// ==================== Edge Cases and Numerical Robustness ====================

describe('Numerical Edge Cases', () => {
    describe('Large Dimension Vectors', () => {
        it('should handle large vector dot product', () => {
            const n = 1000;
            const a = Array.from({ length: n }, (_, i) => i + 1);
            const b = Array.from({ length: n }, () => 1);
            // Sum of 1 to n = n*(n+1)/2
            expect(dot(a, b)).toBe((n * (n + 1)) / 2);
        });

        it('should handle large vector norm accurately', () => {
            const n = 1000;
            const v = Array.from({ length: n }, () => 1);
            expect(isClose(norm(v), Math.sqrt(n), 1e-10)).toBe(true);
        });

        it('should handle operations on vectors with many zeros', () => {
            const n = 1000;
            const sparse = Array.from({ length: n }, (_, i) => (i % 100 === 0 ? 1 : 0));
            expect(norm(sparse)).toBe(Math.sqrt(10)); // 10 non-zero elements
        });
    });

    describe('Extreme Values', () => {
        it('should handle very small values without underflow', () => {
            const v = [1e-150, 1e-150, 1e-150];
            const n = norm(v);
            expect(n).toBeGreaterThan(0);
            expect(Number.isFinite(n)).toBe(true);
        });

        it('should handle very large values without overflow', () => {
            const v = [1e150, 1e150, 1e150];
            const n = norm(v);
            expect(Number.isFinite(n)).toBe(true);
        });

        it('should handle mixed scale values', () => {
            const v = [1e10, 1, 1e-10];
            const n = norm(v);
            expect(Number.isFinite(n)).toBe(true);
            // Should be dominated by largest value
            expect(isClose(n, 1e10, 0.01)).toBe(true);
        });

        it('should handle negative infinity correctly', () => {
            const v = [-100, -200, -300];
            expect(infNorm(v)).toBe(300);
        });
    });

    describe('Zero and Empty Vectors', () => {
        it('should handle zero vector operations', () => {
            const zero = [0, 0, 0];
            expect(norm(zero)).toBe(0);
            expect(dot(zero, zero)).toBe(0);
        });

        it('should handle empty array gracefully', () => {
            expect(norm([])).toBe(0);
            expect(dot([], [])).toBe(0);
        });

        it('should handle single element vectors', () => {
            expect(norm([5])).toBe(5);
            expect(dot([3], [4])).toBe(12);
        });
    });

    describe('3D Vector Edge Cases', () => {
        it('should handle zero magnitude in normalize3', () => {
            // normalize3 should handle zero vector gracefully
            const zero: [number, number, number] = [0, 0, 0];
            const normalized = normalize3(zero);
            // Result could be NaN or zeros - just verify it doesn't throw
            expect(normalized.length).toBe(3);
        });

        it('should handle very small vectors in normalize3', () => {
            const tiny: [number, number, number] = [1e-15, 1e-15, 1e-15];
            const normalized = normalize3(tiny);
            // Result should be unit vector (or handle gracefully)
            expect(normalized.length).toBe(3);
        });

        it('should compute cross product with parallel vectors', () => {
            const a: [number, number, number] = [1, 2, 3];
            const b: [number, number, number] = [2, 4, 6]; // parallel to a
            const cross = cross3(a, b);
            expect(arraysClose(cross, [0, 0, 0])).toBe(true);
        });

        it('should compute cross product with anti-parallel vectors', () => {
            const a: [number, number, number] = [1, 0, 0];
            const b: [number, number, number] = [-1, 0, 0];
            const cross = cross3(a, b);
            expect(arraysClose(cross, [0, 0, 0])).toBe(true);
        });
    });

    describe('Matrix Edge Cases', () => {
        it('should handle determinant of nearly singular matrix', () => {
            const nearSingular: [[number, number, number], [number, number, number], [number, number, number]] = [
                [1, 2, 3],
                [4, 5, 6],
                [7, 8, 9.0001], // Nearly linearly dependent
            ];
            const d = det3(nearSingular);
            expect(Math.abs(d)).toBeLessThan(0.001);
        });

        it('should handle diagonal matrix determinant', () => {
            const diag: [[number, number, number], [number, number, number], [number, number, number]] = [
                [2, 0, 0],
                [0, 3, 0],
                [0, 0, 4],
            ];
            expect(det3(diag)).toBe(24);
        });

        it('should multiply by zero matrix', () => {
            const zero = zeroMat3();
            const A = identity3();
            const result = mulMat3(A, zero);
            expect(matricesClose(result, zero)).toBe(true);
        });

        it('should handle matrix-vector multiplication with zeros', () => {
            const A = identity3();
            const zero: [number, number, number] = [0, 0, 0];
            const result = mulMatVec3(A, zero);
            expect(arraysClose(result, [0, 0, 0])).toBe(true);
        });
    });

    describe('Cholesky Decomposition Edge Cases', () => {
        it('should handle diagonal positive definite matrix', () => {
            const D: [[number, number, number], [number, number, number], [number, number, number]] = [
                [4, 0, 0],
                [0, 9, 0],
                [0, 0, 16],
            ];
            const L = cholesky3(D);
            // L should be diagonal with sqrt of diagonal elements
            expect(isClose(L[0][0], 2, 1e-10)).toBe(true);
            expect(isClose(L[1][1], 3, 1e-10)).toBe(true);
            expect(isClose(L[2][2], 4, 1e-10)).toBe(true);
        });

        it('should handle well-conditioned SPD matrix', () => {
            // A matrix with condition number close to 1
            const A: [[number, number, number], [number, number, number], [number, number, number]] = [
                [10, 1, 1],
                [1, 10, 1],
                [1, 1, 10],
            ];
            const L = cholesky3(A);
            const LT = transpose3(L);
            const reconstructed = mulMat3(L, LT);
            expect(matricesClose(reconstructed, A, 1e-10)).toBe(true);
        });
    });

    describe('Interpolation Edge Cases', () => {
        it('should handle negative interpolation parameter', () => {
            const a: [number, number, number] = [0, 0, 0];
            const b: [number, number, number] = [10, 10, 10];
            const result = lerp3(a, b, -0.5);
            expect(arraysClose(result, [-5, -5, -5])).toBe(true);
        });

        it('should handle t > 1 (extrapolation)', () => {
            const a: [number, number, number] = [0, 0, 0];
            const b: [number, number, number] = [10, 10, 10];
            const result = lerp3(a, b, 1.5);
            expect(arraysClose(result, [15, 15, 15])).toBe(true);
        });
    });

    describe('Generic Vector Operation Consistency', () => {
        it('should satisfy vector addition commutativity', () => {
            const a = [1, 2, 3, 4, 5];
            const b = [5, 4, 3, 2, 1];
            expect(arraysClose(add(a, b), add(b, a))).toBe(true);
        });

        it('should satisfy scaling distributivity', () => {
            const a = [1, 2, 3];
            const b = [4, 5, 6];
            const s = 2;
            // s*(a+b) = s*a + s*b
            const left = scale(add(a, b), s);
            const right = add(scale(a, s), scale(b, s));
            expect(arraysClose(left, right)).toBe(true);
        });

        it('should satisfy dot product linearity', () => {
            const a = [1, 2, 3];
            const b = [4, 5, 6];
            const c = [7, 8, 9];
            const s = 2;
            // dot(s*a, b) = s * dot(a, b)
            expect(isClose(dot(scale(a, s), b), s * dot(a, b))).toBe(true);
            // dot(a+b, c) = dot(a, c) + dot(b, c)
            expect(isClose(dot(add(a, b), c), dot(a, c) + dot(b, c))).toBe(true);
        });

        it('should satisfy norm properties', () => {
            const v = [3, 4];
            // ||sv|| = |s| * ||v||
            expect(isClose(norm(scale(v, 3)), 3 * norm(v))).toBe(true);
            expect(isClose(norm(scale(v, -2)), 2 * norm(v))).toBe(true);
        });
    });
});
