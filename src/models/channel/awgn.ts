/**
 * @module channel/awgn
 * @description AWGN (Additive White Gaussian Noise) Channel Model
 * 
 * The AWGN channel is the most basic physical channel model, considering only 
 * the effect of additive white Gaussian noise.
 * 
 * ## Theoretical Background
 * In an AWGN channel, the received signal y = x + n, where:
 * - x is the transmitted signal
 * - n is the noise following Gaussian distribution with mean 0 and variance N₀/2
 * 
 * ## References
 * - Shannon, C. E. (1948). A Mathematical Theory of Communication
 * - Proakis, J. G. (2008). Digital Communications
 * 
 * @see https://en.wikipedia.org/wiki/Additive_white_Gaussian_noise
 */

import { dbToLinear, linearToDb } from '../utils/conversion';
import { gaussianRandom, gaussianPdf as _gaussianPdf } from '../utils/statistics';

/**
 * Thermal Noise Power Spectral Density at Room Temperature (dBm/Hz)
 * At 290K (approx 17°C), kT = -174 dBm/Hz
 */
export const THERMAL_NOISE_DENSITY = -174;

/**
 * AWGN Channel Parameters Interface
 */
export interface AWGNParams {
    /** Signal-to-Noise Ratio (dB) */
    snrDb: number;
    /** Channel Bandwidth (Hz) */
    bandwidth: number;
}

/**
 * AWGN Channel Calculation Result Interface
 */
export interface AWGNResult {
    /** Signal-to-Noise Ratio (Linear) */
    snrLinear: number;
    /** Signal-to-Noise Ratio (dB) */
    snrDb: number;
    /** Shannon Channel Capacity (bps) */
    capacity: number;
    /** Noise Power (dBm) */
    noisePowerDbm: number;
    /** Noise Standard Deviation (assuming normalized signal power = 1) */
    noiseStd: number;
}

/**
 * Calculate AWGN Channel Parameters
 * 
 * @param params - AWGN channel parameters
 * @returns Calculation result
 * 
 * @example
 * ```typescript
 * const result = calculate({ snrDb: 10, bandwidth: 20e6 });
 * console.log(`Capacity: ${result.capacity / 1e6} Mbps`);
 * // Capacity: 69.23 Mbps
 * ```
 */
export function calculate(params: AWGNParams): AWGNResult {
    const { snrDb, bandwidth } = params;

    const snrLinear = dbToLinear(snrDb);
    const noisePowerDbm = THERMAL_NOISE_DENSITY + linearToDb(bandwidth);
    const noiseStd = 1 / Math.sqrt(Math.max(0.01, snrLinear));
    const capacity = shannonCapacity(snrDb, bandwidth);

    return {
        snrLinear,
        snrDb,
        capacity,
        noisePowerDbm,
        noiseStd
    };
}

/**
 * Calculate Shannon Channel Capacity
 * 
 * Formula: C = B × log₂(1 + SNR)
 * 
 * @param snrDb - Signal-to-Noise Ratio (dB)
 * @param bandwidth - Bandwidth (Hz)
 * @returns Channel Capacity (bps)
 * 
 * @example
 * ```typescript
 * const capacity = shannonCapacity(10, 20e6);  // 10dB SNR, 20MHz BW
 * console.log(`${capacity / 1e6} Mbps`);       // Approx 69.23 Mbps
 * ```
 */
export function shannonCapacity(snrDb: number, bandwidth: number): number {
    const snrLinear = dbToLinear(snrDb);
    return bandwidth * Math.log2(1 + snrLinear);
}

/**
 * Calculate Required SNR for Target Capacity
 * 
 * Inverted from C = B × log₂(1 + SNR) -> SNR = 2^(C/B) - 1
 * 
 * @param capacity - Target Capacity (bps)
 * @param bandwidth - Bandwidth (Hz)
 * @returns Required SNR (dB)
 * 
 * @example
 * ```typescript
 * const snr = requiredSnr(100e6, 20e6);  // 100 Mbps, 20 MHz
 * console.log(`Required SNR: ${snr.toFixed(2)} dB`);
 * ```
 */
export function requiredSnr(capacity: number, bandwidth: number): number {
    const snrLinear = Math.pow(2, capacity / bandwidth) - 1;
    return linearToDb(snrLinear);
}

/**
 * Calculate Required Signal Power
 * 
 * @param snrDb - Target SNR (dB)
 * @param bandwidth - Bandwidth (Hz)
 * @returns Required Signal Power (dBm)
 */
export function requiredSignalPower(snrDb: number, bandwidth: number): number {
    const noisePowerDbm = THERMAL_NOISE_DENSITY + linearToDb(bandwidth);
    return snrDb + noisePowerDbm;
}

/**
 * Add AWGN Noise to Signal
 * 
 * @param signal - Input signal (real number array)
 * @param snrDb - SNR (dB)
 * @returns Signal with added noise
 * 
 * @example
 * ```typescript
 * const cleanSignal = [1, 1, -1, -1, 1];
 * const noisySignal = addNoise(cleanSignal, 10);  // Add 10dB SNR noise
 * ```
 */
export function addNoise(signal: number[], snrDb: number): number[] {
    const signalPower = signal.reduce((sum, x) => sum + x * x, 0) / signal.length;
    const snrLinear = dbToLinear(snrDb);
    const noisePower = signalPower / snrLinear;
    const noiseStd = Math.sqrt(noisePower);

    return signal.map(x => x + gaussianRandom(0, noiseStd));
}

/**
 * Add Complex AWGN Noise to Signal
 * 
 * @param realPart - Real part array
 * @param imagPart - Imaginary part array
 * @param snrDb - SNR (dB)
 * @returns [Noisy Real, Noisy Imag]
 */
export function addComplexNoise(
    realPart: number[],
    imagPart: number[],
    snrDb: number
): [number[], number[]] {
    if (realPart.length !== imagPart.length) {
        throw new Error('Real and imaginary parts must have the same length');
    }

    // Calculate complex signal power
    let signalPower = 0;
    for (let i = 0; i < realPart.length; i++) {
        signalPower += realPart[i] * realPart[i] + imagPart[i] * imagPart[i];
    }
    signalPower /= realPart.length;

    const snrLinear = dbToLinear(snrDb);
    const noisePower = signalPower / snrLinear;
    const noiseStd = Math.sqrt(noisePower / 2); // Standard deviation per component

    const noisyReal = realPart.map(x => x + gaussianRandom(0, noiseStd));
    const noisyImag = imagPart.map(x => x + gaussianRandom(0, noiseStd));

    return [noisyReal, noisyImag];
}

/**
 * Generate Noise Distribution PDF Data (For Visualization)
 * 
 * @param snrDb - SNR (dB)
 * @param numPoints - Number of points (default 100)
 * @returns { x: number[], y: number[] } PDF Data
 */
export function noisePdfData(snrDb: number, numPoints: number = 100): { x: number[], y: number[] } {
    const snrLinear = dbToLinear(snrDb);
    const noiseStd = 1 / Math.sqrt(Math.max(0.01, snrLinear));

    const range = Math.max(3, noiseStd * 4);
    const x: number[] = [];
    const y: number[] = [];

    for (let i = 0; i <= numPoints; i++) {
        const xi = -range + (2 * range * i) / numPoints;
        x.push(xi);
        y.push(_gaussianPdf(xi, 0, noiseStd));
    }

    return { x, y };
}

