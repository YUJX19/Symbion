/**
 * @module planning/minco
 * @description MINCO (Minimum Control) trajectory optimization algorithm.
 * 
 * Generates smooth polynomial trajectories by minimizing control effort
 * (e.g., jerk, snap) while passing through specified waypoints.
 * Uses banded linear systems for efficient computation.
 * 
 * Reference: Zhepei Wang et al., "Geometrically Constrained Trajectory Optimization
 * for Multicopters", IEEE Transactions on Robotics, 2022.
 */

import { zeros, zerosMatrix, dot, squaredNorm } from '../../numeric/math/linear-algebra';
import type { Vector3Array, PolynomialPiece, PiecewiseTrajectory } from '../trajectory/types';

// ==================== Banded System Solver ====================

/**
 * Efficient solver for banded linear systems Ax = b.
 * Uses LU factorization with O(N) complexity.
 */
export class BandedSystem {
    private N: number = 0;
    private lowerBw: number = 0;
    private upperBw: number = 0;
    private data: number[] = [];

    /**
     * Create a banded system of size n with given bandwidths
     * @param n Matrix size
     * @param p Lower bandwidth
     * @param q Upper bandwidth
     */
    create(n: number, p: number, q: number): void {
        this.N = n;
        this.lowerBw = p;
        this.upperBw = q;
        const actualSize = n * (p + q + 1);
        this.data = new Array(actualSize).fill(0);
    }

    /**
     * Reset the matrix to zero
     */
    reset(): void {
        this.data.fill(0);
    }

    /**
     * Get element at (i, j)
     */
    get(i: number, j: number): number {
        return this.data[(i - j + this.upperBw) * this.N + j];
    }

    /**
     * Set element at (i, j)
     */
    set(i: number, j: number, value: number): void {
        this.data[(i - j + this.upperBw) * this.N + j] = value;
    }

    /**
     * In-place LU factorization (no pivoting for efficiency)
     */
    factorizeLU(): void {
        for (let k = 0; k <= this.N - 2; k++) {
            const iM = Math.min(k + this.lowerBw, this.N - 1);
            const cVl = this.get(k, k);

            for (let i = k + 1; i <= iM; i++) {
                if (this.get(i, k) !== 0) {
                    this.set(i, k, this.get(i, k) / cVl);
                }
            }

            const jM = Math.min(k + this.upperBw, this.N - 1);
            for (let j = k + 1; j <= jM; j++) {
                const cVl2 = this.get(k, j);
                if (cVl2 !== 0) {
                    for (let i = k + 1; i <= iM; i++) {
                        if (this.get(i, k) !== 0) {
                            this.set(i, j, this.get(i, j) - this.get(i, k) * cVl2);
                        }
                    }
                }
            }
        }
    }

    /**
     * Solve Ax = b in place (b is replaced with solution)
     * @param b Right-hand side matrix (N x cols), modified in place
     */
    solve(b: number[][]): void {
        const cols = b[0].length;

        // Forward substitution
        for (let j = 0; j <= this.N - 1; j++) {
            const iM = Math.min(j + this.lowerBw, this.N - 1);
            for (let i = j + 1; i <= iM; i++) {
                const factor = this.get(i, j);
                if (factor !== 0) {
                    for (let c = 0; c < cols; c++) {
                        b[i][c] -= factor * b[j][c];
                    }
                }
            }
        }

        // Back substitution
        for (let j = this.N - 1; j >= 0; j--) {
            const diag = this.get(j, j);
            for (let c = 0; c < cols; c++) {
                b[j][c] /= diag;
            }

            const iM = Math.max(0, j - this.upperBw);
            for (let i = iM; i <= j - 1; i++) {
                const factor = this.get(i, j);
                if (factor !== 0) {
                    for (let c = 0; c < cols; c++) {
                        b[i][c] -= factor * b[j][c];
                    }
                }
            }
        }
    }

    /**
     * Solve A^T x = b in place (for adjoint/gradient computation)
     */
    solveAdj(b: number[][]): void {
        const cols = b[0].length;

        // Forward part of adjoint solve
        for (let j = 0; j <= this.N - 1; j++) {
            const diag = this.get(j, j);
            for (let c = 0; c < cols; c++) {
                b[j][c] /= diag;
            }

            const iM = Math.min(j + this.upperBw, this.N - 1);
            for (let i = j + 1; i <= iM; i++) {
                const factor = this.get(j, i);
                if (factor !== 0) {
                    for (let c = 0; c < cols; c++) {
                        b[i][c] -= factor * b[j][c];
                    }
                }
            }
        }

        // Back part of adjoint solve
        for (let j = this.N - 1; j >= 0; j--) {
            const iM = Math.max(0, j - this.lowerBw);
            for (let i = iM; i <= j - 1; i++) {
                const factor = this.get(j, i);
                if (factor !== 0) {
                    for (let c = 0; c < cols; c++) {
                        b[i][c] -= factor * b[j][c];
                    }
                }
            }
        }
    }
}

// ==================== MINCO S=2 (Minimum Jerk) ====================

/**
 * MINCO trajectory optimizer for s=2 (cubic polynomials, minimum jerk).
 * Suitable for velocity-constrained trajectories.
 */
export class MinimumJerkTrajectory {
    private N: number = 0;
    private headPV: number[][] = [[0, 0], [0, 0], [0, 0]]; // 3x2: pos, vel
    private tailPV: number[][] = [[0, 0], [0, 0], [0, 0]]; // 3x2: pos, vel
    private A: BandedSystem = new BandedSystem();
    private b: number[][] = [];
    private T1: number[] = [];
    private T2: number[] = [];
    private T3: number[] = [];

    /**
     * Set boundary conditions and number of trajectory pieces
     * @param headState Initial [position, velocity] for each dimension
     * @param tailState Final [position, velocity] for each dimension
     * @param pieceNum Number of polynomial pieces
     */
    setConditions(
        headState: { position: Vector3Array; velocity: Vector3Array },
        tailState: { position: Vector3Array; velocity: Vector3Array },
        pieceNum: number
    ): void {
        this.N = pieceNum;
        this.headPV = [
            [headState.position[0], headState.velocity[0]],
            [headState.position[1], headState.velocity[1]],
            [headState.position[2], headState.velocity[2]]
        ];
        this.tailPV = [
            [tailState.position[0], tailState.velocity[0]],
            [tailState.position[1], tailState.velocity[1]],
            [tailState.position[2], tailState.velocity[2]]
        ];

        this.A.create(4 * this.N, 4, 4);
        this.b = zerosMatrix(4 * this.N, 3);
        this.T1 = zeros(this.N);
        this.T2 = zeros(this.N);
        this.T3 = zeros(this.N);
    }

    /**
     * Compute trajectory given intermediate waypoints and time durations
     * @param waypoints Intermediate waypoints (N-1 points)
     * @param durations Time duration for each piece (N values)
     */
    setParameters(waypoints: Vector3Array[], durations: number[]): void {
        this.T1 = [...durations];
        this.T2 = this.T1.map((t, i) => t * t);
        this.T3 = this.T2.map((t2, i) => t2 * this.T1[i]);

        this.A.reset();
        this.b = zerosMatrix(4 * this.N, 3);

        // Boundary conditions at start
        this.A.set(0, 0, 1.0);
        this.A.set(1, 1, 1.0);
        this.b[0] = [this.headPV[0][0], this.headPV[1][0], this.headPV[2][0]];
        this.b[1] = [this.headPV[0][1], this.headPV[1][1], this.headPV[2][1]];

        // Continuity constraints at intermediate points
        for (let i = 0; i < this.N - 1; i++) {
            // Acceleration continuity
            this.A.set(4 * i + 2, 4 * i + 2, 2.0);
            this.A.set(4 * i + 2, 4 * i + 3, 6.0 * this.T1[i]);
            this.A.set(4 * i + 2, 4 * i + 6, -2.0);

            // Position at end = waypoint
            this.A.set(4 * i + 3, 4 * i, 1.0);
            this.A.set(4 * i + 3, 4 * i + 1, this.T1[i]);
            this.A.set(4 * i + 3, 4 * i + 2, this.T2[i]);
            this.A.set(4 * i + 3, 4 * i + 3, this.T3[i]);

            // Position continuity
            this.A.set(4 * i + 4, 4 * i, 1.0);
            this.A.set(4 * i + 4, 4 * i + 1, this.T1[i]);
            this.A.set(4 * i + 4, 4 * i + 2, this.T2[i]);
            this.A.set(4 * i + 4, 4 * i + 3, this.T3[i]);
            this.A.set(4 * i + 4, 4 * i + 4, -1.0);

            // Velocity continuity
            this.A.set(4 * i + 5, 4 * i + 1, 1.0);
            this.A.set(4 * i + 5, 4 * i + 2, 2.0 * this.T1[i]);
            this.A.set(4 * i + 5, 4 * i + 3, 3.0 * this.T2[i]);
            this.A.set(4 * i + 5, 4 * i + 5, -1.0);

            this.b[4 * i + 3] = [waypoints[i][0], waypoints[i][1], waypoints[i][2]];
        }

        // Boundary conditions at end
        const N = this.N;
        this.A.set(4 * N - 2, 4 * N - 4, 1.0);
        this.A.set(4 * N - 2, 4 * N - 3, this.T1[N - 1]);
        this.A.set(4 * N - 2, 4 * N - 2, this.T2[N - 1]);
        this.A.set(4 * N - 2, 4 * N - 1, this.T3[N - 1]);
        this.A.set(4 * N - 1, 4 * N - 3, 1.0);
        this.A.set(4 * N - 1, 4 * N - 2, 2 * this.T1[N - 1]);
        this.A.set(4 * N - 1, 4 * N - 1, 3 * this.T2[N - 1]);

        this.b[4 * N - 2] = [this.tailPV[0][0], this.tailPV[1][0], this.tailPV[2][0]];
        this.b[4 * N - 1] = [this.tailPV[0][1], this.tailPV[1][1], this.tailPV[2][1]];

        this.A.factorizeLU();
        this.A.solve(this.b);
    }

    /**
     * Get the computed trajectory
     */
    getTrajectory(): PiecewiseTrajectory {
        const pieces: PolynomialPiece[] = [];

        for (let i = 0; i < this.N; i++) {
            // Coefficients are in order [c0, c1, c2, c3] where p(t) = c0 + c1*t + c2*t^2 + c3*t^3
            // Reverse for standard polynomial form
            const coeffsX = [this.b[4 * i + 3][0], this.b[4 * i + 2][0], this.b[4 * i + 1][0], this.b[4 * i][0]];
            const coeffsY = [this.b[4 * i + 3][1], this.b[4 * i + 2][1], this.b[4 * i + 1][1], this.b[4 * i][1]];
            const coeffsZ = [this.b[4 * i + 3][2], this.b[4 * i + 2][2], this.b[4 * i + 1][2], this.b[4 * i][2]];

            pieces.push({
                duration: this.T1[i],
                coeffs: [coeffsX, coeffsY, coeffsZ]
            });
        }

        return { pieces };
    }

    /**
     * Get the total energy (integral of squared jerk)
     */
    getEnergy(): number {
        let energy = 0;
        for (let i = 0; i < this.N; i++) {
            const c2 = this.b[4 * i + 2];
            const c3 = this.b[4 * i + 3];

            energy += 4.0 * squaredNorm(c2) * this.T1[i] +
                12.0 * dot(c2, c3) * this.T2[i] +
                12.0 * squaredNorm(c3) * this.T3[i];
        }
        return energy;
    }

    /**
     * Get the raw coefficient matrix for gradient computation
     */
    getCoeffs(): number[][] {
        return this.b;
    }

    /**
     * Compute partial gradient of energy with respect to polynomial coefficients.
     * Used in gradient-based trajectory optimization.
     * 
     * @returns Matrix of gradients (4*N x 3)
     */
    getEnergyPartialGradByCoeffs(): number[][] {
        const gdC: number[][] = [];
        for (let i = 0; i < this.N; i++) {
            const c2 = this.b[4 * i + 2];
            const c3 = this.b[4 * i + 3];

            // Gradient w.r.t. c0, c1 are zero
            gdC.push([0, 0, 0]);
            gdC.push([0, 0, 0]);

            // Gradient w.r.t. c2: 8*c2*T1 + 12*c3*T2
            gdC.push([
                8.0 * c2[0] * this.T1[i] + 12.0 * c3[0] * this.T2[i],
                8.0 * c2[1] * this.T1[i] + 12.0 * c3[1] * this.T2[i],
                8.0 * c2[2] * this.T1[i] + 12.0 * c3[2] * this.T2[i]
            ]);

            // Gradient w.r.t. c3: 12*c2*T2 + 24*c3*T3
            gdC.push([
                12.0 * c2[0] * this.T2[i] + 24.0 * c3[0] * this.T3[i],
                12.0 * c2[1] * this.T2[i] + 24.0 * c3[1] * this.T3[i],
                12.0 * c2[2] * this.T2[i] + 24.0 * c3[2] * this.T3[i]
            ]);
        }
        return gdC;
    }

    /**
     * Compute partial gradient of energy with respect to time durations.
     * 
     * @returns Vector of gradients (N)
     */
    getEnergyPartialGradByTimes(): number[] {
        const gdT: number[] = [];
        for (let i = 0; i < this.N; i++) {
            const c2 = this.b[4 * i + 2];
            const c3 = this.b[4 * i + 3];

            gdT.push(
                4.0 * squaredNorm(c2) +
                24.0 * dot(c2, c3) * this.T1[i] +
                36.0 * squaredNorm(c3) * this.T2[i]
            );
        }
        return gdT;
    }

    /**
     * Propagate gradients from coefficient/time space to waypoint/time space.
     * This is the key function for gradient-based optimization.
     * 
     * @param partialGradByCoeffs Gradient of cost w.r.t. coefficients
     * @param partialGradByTimes Gradient of cost w.r.t. time durations
     * @returns Gradients w.r.t. intermediate waypoints and time durations
     */
    propagateGrad(
        partialGradByCoeffs: number[][],
        partialGradByTimes: number[]
    ): { gradByPoints: number[][]; gradByTimes: number[] } {
        const gradByPoints: number[][] = [];
        const gradByTimes: number[] = [...partialGradByTimes];

        // Solve adjoint system A^T * adjGrad = partialGradByCoeffs
        const adjGrad = partialGradByCoeffs.map(row => [...row]);
        this.A.solveAdj(adjGrad);

        // Extract gradient w.r.t. intermediate waypoints
        for (let i = 0; i < this.N - 1; i++) {
            gradByPoints.push([...adjGrad[4 * i + 3]]);
        }

        // Compute gradient w.r.t. time through chain rule
        for (let i = 0; i < this.N - 1; i++) {
            const c1 = this.b[4 * i + 1];
            const c2 = this.b[4 * i + 2];
            const c3 = this.b[4 * i + 3];

            // Negative jerk at end of piece
            const negJerk = [-6.0 * c3[0], -6.0 * c3[1], -6.0 * c3[2]];

            // Negative velocity at end of piece
            const negVel = [
                -(c1[0] + 2.0 * this.T1[i] * c2[0] + 3.0 * this.T2[i] * c3[0]),
                -(c1[1] + 2.0 * this.T1[i] * c2[1] + 3.0 * this.T2[i] * c3[1]),
                -(c1[2] + 2.0 * this.T1[i] * c2[2] + 3.0 * this.T2[i] * c3[2])
            ];

            // Negative acceleration at end of piece
            const negAcc = [
                -(2.0 * c2[0] + 6.0 * this.T1[i] * c3[0]),
                -(2.0 * c2[1] + 6.0 * this.T1[i] * c3[1]),
                -(2.0 * c2[2] + 6.0 * this.T1[i] * c3[2])
            ];

            // B matrix for this segment
            const B = [negJerk, negVel, negVel, negAcc];
            const adjBlock = adjGrad.slice(4 * i + 2, 4 * i + 6);

            let sum = 0;
            for (let r = 0; r < 4; r++) {
                for (let c = 0; c < 3; c++) {
                    sum += B[r][c] * adjBlock[r][c];
                }
            }
            gradByTimes[i] += sum;
        }

        // Handle last segment
        const n = this.N - 1;
        const c1 = this.b[4 * n + 1];
        const c2 = this.b[4 * n + 2];
        const c3 = this.b[4 * n + 3];

        const negVelLast = [
            -(c1[0] + 2.0 * this.T1[n] * c2[0] + 3.0 * this.T2[n] * c3[0]),
            -(c1[1] + 2.0 * this.T1[n] * c2[1] + 3.0 * this.T2[n] * c3[1]),
            -(c1[2] + 2.0 * this.T1[n] * c2[2] + 3.0 * this.T2[n] * c3[2])
        ];
        const negAccLast = [
            -(2.0 * c2[0] + 6.0 * this.T1[n] * c3[0]),
            -(2.0 * c2[1] + 6.0 * this.T1[n] * c3[1]),
            -(2.0 * c2[2] + 6.0 * this.T1[n] * c3[2])
        ];

        const B2 = [negVelLast, negAccLast];
        const adjBlock2 = adjGrad.slice(4 * this.N - 2, 4 * this.N);

        let sumLast = 0;
        for (let r = 0; r < 2; r++) {
            for (let c = 0; c < 3; c++) {
                sumLast += B2[r][c] * adjBlock2[r][c];
            }
        }
        gradByTimes[n] += sumLast;

        return { gradByPoints, gradByTimes };
    }
}

// ==================== MINCO S=3 (Minimum Snap) ====================

/**
 * MINCO trajectory optimizer for s=3 (quintic polynomials, minimum snap).
 * Standard for quadrotor trajectory optimization.
 */
export class MinimumSnapTrajectory {
    private N: number = 0;
    private headPVA: number[][] = []; // 3x3: pos, vel, acc
    private tailPVA: number[][] = [];
    private A: BandedSystem = new BandedSystem();
    private b: number[][] = [];
    private T1: number[] = [];
    private T2: number[] = [];
    private T3: number[] = [];
    private T4: number[] = [];
    private T5: number[] = [];

    /**
     * Set boundary conditions and number of trajectory pieces
     */
    setConditions(
        headState: { position: Vector3Array; velocity: Vector3Array; acceleration: Vector3Array },
        tailState: { position: Vector3Array; velocity: Vector3Array; acceleration: Vector3Array },
        pieceNum: number
    ): void {
        this.N = pieceNum;

        this.headPVA = [
            [headState.position[0], headState.velocity[0], headState.acceleration[0]],
            [headState.position[1], headState.velocity[1], headState.acceleration[1]],
            [headState.position[2], headState.velocity[2], headState.acceleration[2]]
        ];
        this.tailPVA = [
            [tailState.position[0], tailState.velocity[0], tailState.acceleration[0]],
            [tailState.position[1], tailState.velocity[1], tailState.acceleration[1]],
            [tailState.position[2], tailState.velocity[2], tailState.acceleration[2]]
        ];

        this.A.create(6 * this.N, 6, 6);
        this.b = zerosMatrix(6 * this.N, 3);
        this.T1 = zeros(this.N);
        this.T2 = zeros(this.N);
        this.T3 = zeros(this.N);
        this.T4 = zeros(this.N);
        this.T5 = zeros(this.N);
    }

    /**
     * Compute trajectory given intermediate waypoints and time durations
     */
    setParameters(waypoints: Vector3Array[], durations: number[]): void {
        this.T1 = [...durations];
        this.T2 = this.T1.map((t) => t * t);
        this.T3 = this.T2.map((t2, i) => t2 * this.T1[i]);
        this.T4 = this.T2.map((t2) => t2 * t2);
        this.T5 = this.T4.map((t4, i) => t4 * this.T1[i]);

        this.A.reset();
        this.b = zerosMatrix(6 * this.N, 3);

        // Boundary conditions at start
        this.A.set(0, 0, 1.0);
        this.A.set(1, 1, 1.0);
        this.A.set(2, 2, 2.0);
        this.b[0] = [this.headPVA[0][0], this.headPVA[1][0], this.headPVA[2][0]];
        this.b[1] = [this.headPVA[0][1], this.headPVA[1][1], this.headPVA[2][1]];
        this.b[2] = [this.headPVA[0][2], this.headPVA[1][2], this.headPVA[2][2]];

        // Continuity constraints
        for (let i = 0; i < this.N - 1; i++) {
            // Jerk continuity
            this.A.set(6 * i + 3, 6 * i + 3, 6.0);
            this.A.set(6 * i + 3, 6 * i + 4, 24.0 * this.T1[i]);
            this.A.set(6 * i + 3, 6 * i + 5, 60.0 * this.T2[i]);
            this.A.set(6 * i + 3, 6 * i + 9, -6.0);

            // Snap continuity
            this.A.set(6 * i + 4, 6 * i + 4, 24.0);
            this.A.set(6 * i + 4, 6 * i + 5, 120.0 * this.T1[i]);
            this.A.set(6 * i + 4, 6 * i + 10, -24.0);

            // Position at waypoint
            this.A.set(6 * i + 5, 6 * i, 1.0);
            this.A.set(6 * i + 5, 6 * i + 1, this.T1[i]);
            this.A.set(6 * i + 5, 6 * i + 2, this.T2[i]);
            this.A.set(6 * i + 5, 6 * i + 3, this.T3[i]);
            this.A.set(6 * i + 5, 6 * i + 4, this.T4[i]);
            this.A.set(6 * i + 5, 6 * i + 5, this.T5[i]);

            // Position continuity
            this.A.set(6 * i + 6, 6 * i, 1.0);
            this.A.set(6 * i + 6, 6 * i + 1, this.T1[i]);
            this.A.set(6 * i + 6, 6 * i + 2, this.T2[i]);
            this.A.set(6 * i + 6, 6 * i + 3, this.T3[i]);
            this.A.set(6 * i + 6, 6 * i + 4, this.T4[i]);
            this.A.set(6 * i + 6, 6 * i + 5, this.T5[i]);
            this.A.set(6 * i + 6, 6 * i + 6, -1.0);

            // Velocity continuity
            this.A.set(6 * i + 7, 6 * i + 1, 1.0);
            this.A.set(6 * i + 7, 6 * i + 2, 2 * this.T1[i]);
            this.A.set(6 * i + 7, 6 * i + 3, 3 * this.T2[i]);
            this.A.set(6 * i + 7, 6 * i + 4, 4 * this.T3[i]);
            this.A.set(6 * i + 7, 6 * i + 5, 5 * this.T4[i]);
            this.A.set(6 * i + 7, 6 * i + 7, -1.0);

            // Acceleration continuity
            this.A.set(6 * i + 8, 6 * i + 2, 2.0);
            this.A.set(6 * i + 8, 6 * i + 3, 6 * this.T1[i]);
            this.A.set(6 * i + 8, 6 * i + 4, 12 * this.T2[i]);
            this.A.set(6 * i + 8, 6 * i + 5, 20 * this.T3[i]);
            this.A.set(6 * i + 8, 6 * i + 8, -2.0);

            this.b[6 * i + 5] = [waypoints[i][0], waypoints[i][1], waypoints[i][2]];
        }

        // End boundary conditions
        const N = this.N;
        this.A.set(6 * N - 3, 6 * N - 6, 1.0);
        this.A.set(6 * N - 3, 6 * N - 5, this.T1[N - 1]);
        this.A.set(6 * N - 3, 6 * N - 4, this.T2[N - 1]);
        this.A.set(6 * N - 3, 6 * N - 3, this.T3[N - 1]);
        this.A.set(6 * N - 3, 6 * N - 2, this.T4[N - 1]);
        this.A.set(6 * N - 3, 6 * N - 1, this.T5[N - 1]);

        this.A.set(6 * N - 2, 6 * N - 5, 1.0);
        this.A.set(6 * N - 2, 6 * N - 4, 2 * this.T1[N - 1]);
        this.A.set(6 * N - 2, 6 * N - 3, 3 * this.T2[N - 1]);
        this.A.set(6 * N - 2, 6 * N - 2, 4 * this.T3[N - 1]);
        this.A.set(6 * N - 2, 6 * N - 1, 5 * this.T4[N - 1]);

        this.A.set(6 * N - 1, 6 * N - 4, 2);
        this.A.set(6 * N - 1, 6 * N - 3, 6 * this.T1[N - 1]);
        this.A.set(6 * N - 1, 6 * N - 2, 12 * this.T2[N - 1]);
        this.A.set(6 * N - 1, 6 * N - 1, 20 * this.T3[N - 1]);

        this.b[6 * N - 3] = [this.tailPVA[0][0], this.tailPVA[1][0], this.tailPVA[2][0]];
        this.b[6 * N - 2] = [this.tailPVA[0][1], this.tailPVA[1][1], this.tailPVA[2][1]];
        this.b[6 * N - 1] = [this.tailPVA[0][2], this.tailPVA[1][2], this.tailPVA[2][2]];

        this.A.factorizeLU();
        this.A.solve(this.b);
    }

    /**
     * Get the computed trajectory
     */
    getTrajectory(): PiecewiseTrajectory {
        const pieces: PolynomialPiece[] = [];

        for (let i = 0; i < this.N; i++) {
            // Reverse coefficients for standard polynomial form
            const coeffsX = [
                this.b[6 * i + 5][0], this.b[6 * i + 4][0], this.b[6 * i + 3][0],
                this.b[6 * i + 2][0], this.b[6 * i + 1][0], this.b[6 * i][0]
            ];
            const coeffsY = [
                this.b[6 * i + 5][1], this.b[6 * i + 4][1], this.b[6 * i + 3][1],
                this.b[6 * i + 2][1], this.b[6 * i + 1][1], this.b[6 * i][1]
            ];
            const coeffsZ = [
                this.b[6 * i + 5][2], this.b[6 * i + 4][2], this.b[6 * i + 3][2],
                this.b[6 * i + 2][2], this.b[6 * i + 1][2], this.b[6 * i][2]
            ];

            pieces.push({
                duration: this.T1[i],
                coeffs: [coeffsX, coeffsY, coeffsZ]
            });
        }

        return { pieces };
    }

    /**
     * Get the total energy (integral of squared snap)
     */
    getEnergy(): number {
        let energy = 0;
        for (let i = 0; i < this.N; i++) {
            const c3 = this.b[6 * i + 3];
            const c4 = this.b[6 * i + 4];
            const c5 = this.b[6 * i + 5];

            energy += 36.0 * squaredNorm(c3) * this.T1[i] +
                144.0 * dot(c4, c3) * this.T2[i] +
                192.0 * squaredNorm(c4) * this.T3[i] +
                240.0 * dot(c5, c3) * this.T3[i] +
                720.0 * dot(c5, c4) * this.T4[i] +
                720.0 * squaredNorm(c5) * this.T5[i];
        }
        return energy;
    }

    /**
     * Get the raw coefficient matrix for gradient computation
     */
    getCoeffs(): number[][] {
        return this.b;
    }

    /**
     * Compute partial gradient of energy with respect to polynomial coefficients.
     * Used in gradient-based trajectory optimization with L-BFGS.
     * 
     * @returns Matrix of gradients (6*N x 3)
     */
    getEnergyPartialGradByCoeffs(): number[][] {
        const gdC: number[][] = [];
        for (let i = 0; i < this.N; i++) {
            const c3 = this.b[6 * i + 3];
            const c4 = this.b[6 * i + 4];
            const c5 = this.b[6 * i + 5];

            // Gradient w.r.t. c0, c1, c2 are zero
            gdC.push([0, 0, 0]);
            gdC.push([0, 0, 0]);
            gdC.push([0, 0, 0]);

            // Gradient w.r.t. c3: 72*c3*T1 + 144*c4*T2 + 240*c5*T3
            gdC.push([
                72.0 * c3[0] * this.T1[i] + 144.0 * c4[0] * this.T2[i] + 240.0 * c5[0] * this.T3[i],
                72.0 * c3[1] * this.T1[i] + 144.0 * c4[1] * this.T2[i] + 240.0 * c5[1] * this.T3[i],
                72.0 * c3[2] * this.T1[i] + 144.0 * c4[2] * this.T2[i] + 240.0 * c5[2] * this.T3[i]
            ]);

            // Gradient w.r.t. c4: 144*c3*T2 + 384*c4*T3 + 720*c5*T4
            gdC.push([
                144.0 * c3[0] * this.T2[i] + 384.0 * c4[0] * this.T3[i] + 720.0 * c5[0] * this.T4[i],
                144.0 * c3[1] * this.T2[i] + 384.0 * c4[1] * this.T3[i] + 720.0 * c5[1] * this.T4[i],
                144.0 * c3[2] * this.T2[i] + 384.0 * c4[2] * this.T3[i] + 720.0 * c5[2] * this.T4[i]
            ]);

            // Gradient w.r.t. c5: 240*c3*T3 + 720*c4*T4 + 1440*c5*T5
            gdC.push([
                240.0 * c3[0] * this.T3[i] + 720.0 * c4[0] * this.T4[i] + 1440.0 * c5[0] * this.T5[i],
                240.0 * c3[1] * this.T3[i] + 720.0 * c4[1] * this.T4[i] + 1440.0 * c5[1] * this.T5[i],
                240.0 * c3[2] * this.T3[i] + 720.0 * c4[2] * this.T4[i] + 1440.0 * c5[2] * this.T5[i]
            ]);
        }
        return gdC;
    }

    /**
     * Compute partial gradient of energy with respect to time durations.
     * 
     * @returns Vector of gradients (N)
     */
    getEnergyPartialGradByTimes(): number[] {
        const gdT: number[] = [];
        for (let i = 0; i < this.N; i++) {
            const c3 = this.b[6 * i + 3];
            const c4 = this.b[6 * i + 4];
            const c5 = this.b[6 * i + 5];

            gdT.push(
                36.0 * squaredNorm(c3) +
                288.0 * dot(c4, c3) * this.T1[i] +
                576.0 * squaredNorm(c4) * this.T2[i] +
                720.0 * dot(c5, c3) * this.T2[i] +
                2880.0 * dot(c5, c4) * this.T3[i] +
                3600.0 * squaredNorm(c5) * this.T4[i]
            );
        }
        return gdT;
    }

    /**
     * Propagate gradients from coefficient/time space to waypoint/time space.
     * This is the key function for gradient-based optimization with L-BFGS.
     * 
     * @param partialGradByCoeffs Gradient of cost w.r.t. coefficients (6*N x 3)
     * @param partialGradByTimes Gradient of cost w.r.t. time durations (N)
     * @returns Gradients w.r.t. intermediate waypoints and time durations
     */
    propagateGrad(
        partialGradByCoeffs: number[][],
        partialGradByTimes: number[]
    ): { gradByPoints: number[][]; gradByTimes: number[] } {
        const gradByPoints: number[][] = [];
        const gradByTimes: number[] = [...partialGradByTimes];

        // Solve adjoint system A^T * adjGrad = partialGradByCoeffs
        const adjGrad = partialGradByCoeffs.map(row => [...row]);
        this.A.solveAdj(adjGrad);

        // Extract gradient w.r.t. intermediate waypoints
        for (let i = 0; i < this.N - 1; i++) {
            gradByPoints.push([...adjGrad[6 * i + 5]]);
        }

        // Compute gradient w.r.t. time through chain rule
        for (let i = 0; i < this.N - 1; i++) {
            const c1 = this.b[6 * i + 1];
            const c2 = this.b[6 * i + 2];
            const c3 = this.b[6 * i + 3];
            const c4 = this.b[6 * i + 4];
            const c5 = this.b[6 * i + 5];

            // Negative snap at end of piece
            const negSnap = [
                -(24.0 * c4[0] + 120.0 * this.T1[i] * c5[0]),
                -(24.0 * c4[1] + 120.0 * this.T1[i] * c5[1]),
                -(24.0 * c4[2] + 120.0 * this.T1[i] * c5[2])
            ];

            // Negative crackle
            const negCrackle = [-120.0 * c5[0], -120.0 * c5[1], -120.0 * c5[2]];

            // Negative velocity at end
            const negVel = [
                -(c1[0] + 2.0 * this.T1[i] * c2[0] + 3.0 * this.T2[i] * c3[0] +
                    4.0 * this.T3[i] * c4[0] + 5.0 * this.T4[i] * c5[0]),
                -(c1[1] + 2.0 * this.T1[i] * c2[1] + 3.0 * this.T2[i] * c3[1] +
                    4.0 * this.T3[i] * c4[1] + 5.0 * this.T4[i] * c5[1]),
                -(c1[2] + 2.0 * this.T1[i] * c2[2] + 3.0 * this.T2[i] * c3[2] +
                    4.0 * this.T3[i] * c4[2] + 5.0 * this.T4[i] * c5[2])
            ];

            // Negative acceleration at end
            const negAcc = [
                -(2.0 * c2[0] + 6.0 * this.T1[i] * c3[0] + 12.0 * this.T2[i] * c4[0] +
                    20.0 * this.T3[i] * c5[0]),
                -(2.0 * c2[1] + 6.0 * this.T1[i] * c3[1] + 12.0 * this.T2[i] * c4[1] +
                    20.0 * this.T3[i] * c5[1]),
                -(2.0 * c2[2] + 6.0 * this.T1[i] * c3[2] + 12.0 * this.T2[i] * c4[2] +
                    20.0 * this.T3[i] * c5[2])
            ];

            // Negative jerk at end
            const negJerk = [
                -(6.0 * c3[0] + 24.0 * this.T1[i] * c4[0] + 60.0 * this.T2[i] * c5[0]),
                -(6.0 * c3[1] + 24.0 * this.T1[i] * c4[1] + 60.0 * this.T2[i] * c5[1]),
                -(6.0 * c3[2] + 24.0 * this.T1[i] * c4[2] + 60.0 * this.T2[i] * c5[2])
            ];

            // B matrix for this segment
            const B = [negSnap, negCrackle, negVel, negVel, negAcc, negJerk];
            const adjBlock = adjGrad.slice(6 * i + 3, 6 * i + 9);

            let sum = 0;
            for (let r = 0; r < 6; r++) {
                for (let c = 0; c < 3; c++) {
                    sum += B[r][c] * adjBlock[r][c];
                }
            }
            gradByTimes[i] += sum;
        }

        // Handle last segment
        const n = this.N - 1;
        const c1 = this.b[6 * n + 1];
        const c2 = this.b[6 * n + 2];
        const c3 = this.b[6 * n + 3];
        const c4 = this.b[6 * n + 4];
        const c5 = this.b[6 * n + 5];

        const negVelLast = [
            -(c1[0] + 2.0 * this.T1[n] * c2[0] + 3.0 * this.T2[n] * c3[0] +
                4.0 * this.T3[n] * c4[0] + 5.0 * this.T4[n] * c5[0]),
            -(c1[1] + 2.0 * this.T1[n] * c2[1] + 3.0 * this.T2[n] * c3[1] +
                4.0 * this.T3[n] * c4[1] + 5.0 * this.T4[n] * c5[1]),
            -(c1[2] + 2.0 * this.T1[n] * c2[2] + 3.0 * this.T2[n] * c3[2] +
                4.0 * this.T3[n] * c4[2] + 5.0 * this.T4[n] * c5[2])
        ];
        const negAccLast = [
            -(2.0 * c2[0] + 6.0 * this.T1[n] * c3[0] + 12.0 * this.T2[n] * c4[0] +
                20.0 * this.T3[n] * c5[0]),
            -(2.0 * c2[1] + 6.0 * this.T1[n] * c3[1] + 12.0 * this.T2[n] * c4[1] +
                20.0 * this.T3[n] * c5[1]),
            -(2.0 * c2[2] + 6.0 * this.T1[n] * c3[2] + 12.0 * this.T2[n] * c4[2] +
                20.0 * this.T3[n] * c5[2])
        ];
        const negJerkLast = [
            -(6.0 * c3[0] + 24.0 * this.T1[n] * c4[0] + 60.0 * this.T2[n] * c5[0]),
            -(6.0 * c3[1] + 24.0 * this.T1[n] * c4[1] + 60.0 * this.T2[n] * c5[1]),
            -(6.0 * c3[2] + 24.0 * this.T1[n] * c4[2] + 60.0 * this.T2[n] * c5[2])
        ];

        const B2 = [negVelLast, negAccLast, negJerkLast];
        const adjBlock2 = adjGrad.slice(6 * this.N - 3, 6 * this.N);

        let sumLast = 0;
        for (let r = 0; r < 3; r++) {
            for (let c = 0; c < 3; c++) {
                sumLast += B2[r][c] * adjBlock2[r][c];
            }
        }
        gradByTimes[n] += sumLast;

        return { gradByPoints, gradByTimes };
    }
}

// ==================== Helper Functions ====================

/**
 * Generate a minimum-snap trajectory through waypoints
 */
export function generateMinimumSnapTrajectory(
    waypoints: Vector3Array[],
    totalTime: number,
    options?: {
        initialVelocity?: Vector3Array;
        finalVelocity?: Vector3Array;
        initialAcceleration?: Vector3Array;
        finalAcceleration?: Vector3Array;
    }
): PiecewiseTrajectory {
    if (waypoints.length < 2) {
        throw new Error('At least 2 waypoints required');
    }

    const numPieces = waypoints.length - 1;
    const opts = options || {};

    // Calculate segment distances for time allocation
    const distances: number[] = [];
    let totalDist = 0;
    for (let i = 0; i < numPieces; i++) {
        const dx = waypoints[i + 1][0] - waypoints[i][0];
        const dy = waypoints[i + 1][1] - waypoints[i][1];
        const dz = waypoints[i + 1][2] - waypoints[i][2];
        const dist = Math.sqrt(dx * dx + dy * dy + dz * dz);
        distances.push(dist);
        totalDist += dist;
    }

    // Allocate time proportionally to distance
    const durations = distances.map(d => (d / totalDist) * totalTime);

    // Create trajectory optimizer
    const traj = new MinimumSnapTrajectory();

    traj.setConditions(
        {
            position: waypoints[0],
            velocity: opts.initialVelocity || [0, 0, 0],
            acceleration: opts.initialAcceleration || [0, 0, 0]
        },
        {
            position: waypoints[waypoints.length - 1],
            velocity: opts.finalVelocity || [0, 0, 0],
            acceleration: opts.finalAcceleration || [0, 0, 0]
        },
        numPieces
    );

    // Intermediate waypoints (excluding start and end)
    const intermediateWaypoints = waypoints.slice(1, -1);

    traj.setParameters(intermediateWaypoints, durations);

    return traj.getTrajectory();
}

/**
 * Evaluate a polynomial trajectory at time t
 */
export function evaluateTrajectory(
    trajectory: PiecewiseTrajectory,
    t: number
): { position: Vector3Array; velocity: Vector3Array; acceleration: Vector3Array } {
    // Find the correct piece
    let accTime = 0;
    let pieceIdx = 0;

    for (let i = 0; i < trajectory.pieces.length; i++) {
        if (t <= accTime + trajectory.pieces[i].duration + 1e-6) {
            pieceIdx = i;
            break;
        }
        accTime += trajectory.pieces[i].duration;
        if (i === trajectory.pieces.length - 1) {
            pieceIdx = i;
        }
    }

    const piece = trajectory.pieces[pieceIdx];
    const localT = Math.max(0, Math.min(t - accTime, piece.duration));

    // Evaluate position, velocity, acceleration for each dimension
    const position: Vector3Array = [0, 0, 0];
    const velocity: Vector3Array = [0, 0, 0];
    const acceleration: Vector3Array = [0, 0, 0];

    for (let dim = 0; dim < 3; dim++) {
        const c = piece.coeffs[dim];
        const n = c.length;

        // Position: sum of c_i * t^(n-1-i)
        let tPower = 1;
        for (let i = n - 1; i >= 0; i--) {
            position[dim] += c[i] * tPower;
            tPower *= localT;
        }

        // Velocity: derivative
        tPower = 1;
        for (let i = n - 2; i >= 0; i--) {
            velocity[dim] += c[i] * (n - 1 - i) * tPower;
            tPower *= localT;
        }

        // Acceleration: second derivative
        tPower = 1;
        for (let i = n - 3; i >= 0; i--) {
            acceleration[dim] += c[i] * (n - 1 - i) * (n - 2 - i) * tPower;
            tPower *= localT;
        }
    }

    return { position, velocity, acceleration };
}
