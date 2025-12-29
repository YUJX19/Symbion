/**
 * @module modulation/qam
 * @description QAM (Quadrature Amplitude Modulation)
 * 
 * ## Background
 * QAM modulates both the amplitude and the phase of the carrier signal 
 * to map data to constellation points in the complex plane.
 * M-QAM transmits log₂(M) bits per symbol.
 * 
 * ## References
 * - Proakis, J. G. (2008). Digital Communications
 */

import { gaussianRandom } from '../../utils/statistics';

/**
 * QAM Constellation Point (Complex Representation)
 */
export interface QAMSymbol {
    /** In-phase component (Real part) */
    i: number;
    /** Quadrature component (Imaginary part) */
    q: number;
}

/**
 * Generate M-QAM Constellation Points
 * 
 * @param order - QAM order (4, 16, 64, 256, etc.)
 * @returns Array of constellation points
 * 
 * @example
 * ```typescript
 * const constellation = generateConstellation(16);  // 16-QAM
 * // Returns 16 constellation points
 * ```
 */
export function generateConstellation(order: number): QAMSymbol[] {
    const sqrtM = Math.sqrt(order);
    if (!Number.isInteger(sqrtM)) {
        throw new Error('QAM order must be a perfect square (4, 16, 64, 256, ...)');
    }

    const points: QAMSymbol[] = [];
    const offset = (sqrtM - 1) / 2;  // Center the constellation around the origin

    for (let i = 0; i < sqrtM; i++) {
        for (let q = 0; q < sqrtM; q++) {
            points.push({
                i: i - offset,
                q: q - offset
            });
        }
    }

    return points;
}

/**
 * Normalize Constellation to Unit Average Energy
 * 
 * @param constellation - Raw constellation points
 * @returns Normalized constellation points
 */
export function normalizeConstellation(constellation: QAMSymbol[]): QAMSymbol[] {
    // Calculate average energy
    let avgEnergy = 0;
    for (const point of constellation) {
        avgEnergy += point.i * point.i + point.q * point.q;
    }
    avgEnergy /= constellation.length;

    // Normalization factor
    const factor = 1 / Math.sqrt(avgEnergy);

    return constellation.map(p => ({
        i: p.i * factor,
        q: p.q * factor
    }));
}

/**
 * Map Bits to Symbol Index (Gray Coding)
 * 
 * @param bits - Bit array
 * @param order - QAM order
 * @returns Symbol index
 */
export function bitsToSymbolIndex(bits: number[], order: number): number {
    const bitsPerSymbol = Math.log2(order);
    if (bits.length !== bitsPerSymbol) {
        throw new Error(`Expected ${bitsPerSymbol} bits for ${order}-QAM`);
    }

    // Gray code conversion logic
    let index = 0;
    for (let i = 0; i < bits.length; i++) {
        index = (index << 1) | bits[i];
    }

    return index;
}

/**
 * Add AWGN Noise to QAM Symbols
 * 
 * @param symbols - Array of symbols
 * @param snrDb - Signal-to-Noise Ratio (dB)
 * @returns Noisy symbols
 */
export function addNoise(symbols: QAMSymbol[], snrDb: number): QAMSymbol[] {
    // Calculate signal power
    let signalPower = 0;
    for (const s of symbols) {
        signalPower += s.i * s.i + s.q * s.q;
    }
    signalPower /= symbols.length;

    // Calculate noise standard deviation
    const snrLinear = Math.pow(10, snrDb / 10);
    const noisePower = signalPower / snrLinear;
    const noiseStd = Math.sqrt(noisePower / 2);  // Std dev per dimension (I/Q)

    return symbols.map(s => ({
        i: s.i + gaussianRandom(0, noiseStd),
        q: s.q + gaussianRandom(0, noiseStd)
    }));
}

/**
 * Hard Decision Demodulation (Nearest Neighbor)
 * 
 * @param received - Received symbols
 * @param constellation - Constellation set
 * @returns Array of decided symbol indices
 */
export function hardDecision(received: QAMSymbol[], constellation: QAMSymbol[]): number[] {
    return received.map(r => {
        let minDist = Infinity;
        let minIdx = 0;

        for (let i = 0; i < constellation.length; i++) {
            const di = r.i - constellation[i].i;
            const dq = r.q - constellation[i].q;
            const dist = di * di + dq * dq;

            if (dist < minDist) {
                minDist = dist;
                minIdx = i;
            }
        }

        return minIdx;
    });
}

/**
 * Calculate Theoretical Symbol Error Rate (SER) for M-QAM
 * 
 * Approximation for high SNR.
 * 
 * @param order - QAM order
 * @param snrDb - Eb/N0 (dB)
 * @returns Theoretical SER
 */
export function theoreticalSer(order: number, snrDb: number): number {
    const snrLinear = Math.pow(10, snrDb / 10);
    const sqrtM = Math.sqrt(order);
    const k = Math.log2(order);  // bits per symbol

    // P_s ≈ 4(1 - 1/√M) * Q(√(3*k*Eb/N0/(M-1)))
    const arg = Math.sqrt(3 * k * snrLinear / (order - 1));
    const qValue = 0.5 * erfc(arg / Math.sqrt(2));

    return 4 * (1 - 1 / sqrtM) * qValue;
}

/**
 * Calculate Spectral Efficiency
 * 
 * @param order - QAM order
 * @returns bits/symbol
 */
export function spectralEfficiency(order: number): number {
    return Math.log2(order);
}

/**
 * Complementary Error Function (Helper)
 */
function erfc(x: number): number {
    const a1 = 0.254829592;
    const a2 = -0.284496736;
    const a3 = 1.421413741;
    const a4 = -1.453152027;
    const a5 = 1.061405429;
    const p = 0.3275911;

    const sign = x < 0 ? -1 : 1;
    x = Math.abs(x);

    const t = 1.0 / (1.0 + p * x);
    const y = 1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * Math.exp(-x * x);

    return 1 - sign * y;
}

/**
 * Generate Random QAM Data (For testing/visualization)
 * 
 * @param options - Configuration options
 * @returns Transmitted and received symbol data
 */
export function generateQAMData(options: {
    order: number;
    count: number;
    snrDb: number;
}): {
    tx: QAMSymbol[];
    rx: QAMSymbol[];
    constellation: QAMSymbol[];
} {
    const { order, count, snrDb } = options;

    const constellation = normalizeConstellation(generateConstellation(order));

    // Randomly select symbols
    const tx: QAMSymbol[] = [];
    for (let i = 0; i < count; i++) {
        const idx = Math.floor(Math.random() * constellation.length);
        tx.push({ ...constellation[idx] });
    }

    // Add noise
    const rx = addNoise(tx, snrDb);

    return { tx, rx, constellation };
}

