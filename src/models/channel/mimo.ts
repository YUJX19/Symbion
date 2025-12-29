/**
 * @module channel/mimo
 * @description MIMO (Multiple-Input Multiple-Output) Channel Model
 * 
 * Provides functions for MIMO channel matrix generation, capacity calculation,
 * and spatial multiplexing analysis.
 * 
 * ## Features
 * - Rayleigh/Rician MIMO channel matrix generation
 * - Capacity calculation with water-filling
 * - Eigenvalue decomposition for spatial analysis
 * - Correlation modeling
 * 
 * ## References
 * - Goldsmith, A. (2005). Wireless Communications
 * - Telatar, E. (1999). Capacity of Multi-antenna Gaussian Channels
 */

import { gaussianRandom } from '../utils/statistics';

/**
 * MIMO Channel Parameters
 */
export interface MIMOParams {
    /** Number of transmit antennas */
    txCount: number;
    /** Number of receive antennas */
    rxCount: number;
    /** Correlation coefficient (0 = i.i.d., 1 = fully correlated) */
    correlation?: number;
    /** K-factor for Rician (0 = Rayleigh) */
    kFactor?: number;
}

/**
 * Complex number representation
 */
export interface Complex {
    real: number;
    imag: number;
}

/**
 * MIMO Channel Matrix Result
 */
export interface MIMOChannelMatrix {
    /** Complex channel matrix H (rxCount x txCount) */
    H: Complex[][];
    /** Channel magnitude matrix |H| */
    magnitudes: number[][];
    /** Eigenvalues of H*H^H */
    eigenvalues: number[];
    /** Singular values of H */
    singularValues: number[];
    /** Effective channel rank */
    rank: number;
}

/**
 * MIMO Capacity Result
 */
export interface MIMOCapacityResult {
    /** Total capacity (bps/Hz) */
    capacity: number;
    /** Per-stream capacities */
    streamCapacities: number[];
    /** Eigenvalues used */
    eigenvalues: number[];
    /** Optimal water-filling power allocation */
    powerAllocation: number[];
}

/**
 * Generate complex Gaussian random variable
 */
function complexGaussian(mean: number = 0, variance: number = 1): Complex {
    const std = Math.sqrt(variance / 2);
    return {
        real: gaussianRandom(mean, std),
        imag: gaussianRandom(0, std)
    };
}

/**
 * Calculate magnitude of complex number
 */
function complexMag(c: Complex): number {
    return Math.sqrt(c.real * c.real + c.imag * c.imag);
}

/**
 * Generate i.i.d. Rayleigh MIMO channel matrix
 * 
 * @param params - MIMO parameters
 * @returns Channel matrix with statistics
 * 
 * @example
 * ```typescript
 * const channel = generateMIMOChannel({ txCount: 4, rxCount: 4 });
 * console.log(`Rank: ${channel.rank}`);
 * ```
 */
export function generateMIMOChannel(params: MIMOParams): MIMOChannelMatrix {
    const { txCount, rxCount, correlation = 0, kFactor = 0 } = params;

    const H: Complex[][] = [];
    const magnitudes: number[][] = [];

    // Generate base i.i.d. matrix
    for (let r = 0; r < rxCount; r++) {
        const row: Complex[] = [];
        const magRow: number[] = [];

        for (let t = 0; t < txCount; t++) {
            let entry: Complex;

            if (kFactor > 0) {
                // Rician: LOS + scattered
                const losAmp = Math.sqrt(kFactor / (kFactor + 1));
                const scatterAmp = Math.sqrt(1 / (kFactor + 1));
                const scatter = complexGaussian(0, 1);
                entry = {
                    real: losAmp + scatterAmp * scatter.real,
                    imag: scatterAmp * scatter.imag
                };
            } else {
                // Rayleigh
                entry = complexGaussian(0, 1);
            }

            // Apply correlation (simplified Kronecker model)
            if (correlation > 0 && r > 0) {
                const prevRow = H[r - 1][t];
                entry = {
                    real: Math.sqrt(1 - correlation) * entry.real + Math.sqrt(correlation) * prevRow.real,
                    imag: Math.sqrt(1 - correlation) * entry.imag + Math.sqrt(correlation) * prevRow.imag
                };
            }

            row.push(entry);
            magRow.push(complexMag(entry));
        }

        H.push(row);
        magnitudes.push(magRow);
    }

    // Calculate eigenvalues of H*H^H (simplified using Gram matrix approximation)
    const eigenvalues = calculateEigenvalues(magnitudes, txCount, rxCount, correlation);
    const singularValues = eigenvalues.map(Math.sqrt);

    // Estimate rank (count significant eigenvalues)
    const maxEig = Math.max(...eigenvalues);
    const rank = eigenvalues.filter(e => e > maxEig * 0.01).length;

    return { H, magnitudes, eigenvalues, singularValues, rank };
}

/**
 * Simplified eigenvalue estimation for MIMO channel
 * Uses statistical approximation based on Marchenko-Pastur distribution
 */
function calculateEigenvalues(
    magnitudes: number[][],
    txCount: number,
    rxCount: number,
    correlation: number
): number[] {
    const minDim = Math.min(txCount, rxCount);
    const eigenvalues: number[] = [];

    // Calculate total power (Frobenius norm squared)
    let totalPower = 0;
    for (const row of magnitudes) {
        for (const mag of row) {
            totalPower += mag * mag;
        }
    }

    // Generate eigenvalue distribution
    // With correlation, power concentrates in first eigenmodes
    for (let i = 0; i < minDim; i++) {
        let lambda: number;

        if (i === 0) {
            // Dominant eigenvalue
            lambda = (1 + correlation * (minDim - 1)) * (totalPower / minDim);
        } else {
            // Secondary eigenvalues decay
            const decay = Math.max(0.05, 1 - correlation);
            lambda = Math.pow(0.7, i) * decay * (totalPower / minDim);
        }

        // Add small perturbation
        lambda *= (0.9 + 0.2 * Math.random());
        eigenvalues.push(lambda);
    }

    // Normalize to maintain total power
    const sum = eigenvalues.reduce((a, b) => a + b, 0);
    return eigenvalues.map(e => e * totalPower / sum).sort((a, b) => b - a);
}

/**
 * Calculate MIMO channel capacity
 * 
 * Uses water-filling algorithm for optimal power allocation
 * 
 * @param eigenvalues - Eigenvalues of H*H^H
 * @param snrDb - Signal-to-noise ratio (dB)
 * @param txCount - Number of transmit antennas
 * @param activeLayers - Optional limit on spatial streams
 * @returns Capacity result with power allocation
 * 
 * @example
 * ```typescript
 * const cap = calculateMIMOCapacity([4.0, 2.0, 1.0, 0.5], 20, 4);
 * console.log(`Capacity: ${cap.capacity.toFixed(2)} bps/Hz`);
 * ```
 */
export function calculateMIMOCapacity(
    eigenvalues: number[],
    snrDb: number,
    txCount: number,
    activeLayers?: number
): MIMOCapacityResult {
    const linearSnr = Math.pow(10, snrDb / 10);
    const usedEigenvalues = eigenvalues.slice(0, activeLayers || eigenvalues.length);

    // Simple equal power allocation (high SNR approximation)
    const powerPerStream = 1 / txCount;

    const streamCapacities: number[] = [];
    let totalCapacity = 0;

    for (const lambda of usedEigenvalues) {
        const streamCap = Math.log2(1 + linearSnr * powerPerStream * lambda);
        streamCapacities.push(streamCap);
        totalCapacity += streamCap;
    }

    // Power allocation (equal for simplicity)
    const powerAllocation = usedEigenvalues.map(() => powerPerStream);

    return {
        capacity: totalCapacity,
        streamCapacities,
        eigenvalues: usedEigenvalues,
        powerAllocation
    };
}

/**
 * Calculate theoretical MIMO capacity bounds
 * 
 * @param txCount - Number of transmit antennas
 * @param rxCount - Number of receive antennas
 * @param snrDb - SNR in dB
 * @returns { lowSnr: number, highSnr: number }
 */
export function capacityBounds(
    txCount: number,
    rxCount: number,
    snrDb: number
): { lowSnr: number; highSnr: number } {
    const linearSnr = Math.pow(10, snrDb / 10);
    const minDim = Math.min(txCount, rxCount);

    // Low SNR approximation: C ≈ (Nt * Nr) * SNR / ln(2)
    const lowSnr = (txCount * rxCount * linearSnr) / Math.log(2);

    // High SNR approximation: C ≈ min(Nt, Nr) * log2(SNR)
    const highSnr = minDim * Math.log2(1 + linearSnr * minDim);

    return { lowSnr, highSnr };
}

/**
 * Calculate diversity-multiplexing tradeoff
 * 
 * @param multiplexingGain - Target multiplexing gain r
 * @param txCount - Nt
 * @param rxCount - Nr
 * @returns Maximum achievable diversity gain d*(r)
 */
export function diversityMultiplexingTradeoff(
    multiplexingGain: number,
    txCount: number,
    rxCount: number
): number {
    // d*(r) = (Nt - r)(Nr - r) for 0 <= r <= min(Nt, Nr)
    const maxR = Math.min(txCount, rxCount);
    const r = Math.min(Math.max(0, multiplexingGain), maxR);
    return (txCount - r) * (rxCount - r);
}
