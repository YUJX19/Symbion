/**
 * @module channel/fading
 * @description Rayleigh and Rician Fading Channel Models
 * 
 * ## Rayleigh Fading
 * For Non-Line-of-Sight (NLOS) scenarios, the channel envelope follows a Rayleigh distribution.
 * 
 * ## Rician Fading
 * For Line-of-Sight (LOS) scenarios, the K-factor describes the power ratio between 
 * the LOS component and the scattered components.
 * 
 * ## References
 * - Jakes, W. C. (1994). Microwave Mobile Communications
 * - Goldsmith, A. (2005). Wireless Communications
 */

import { gaussianRandom } from '../utils/statistics';

/**
 * Rayleigh Fading Channel Coefficient
 * 
 * Generates a channel amplitude following a Rayleigh distribution.
 * 
 * @returns Complex channel coefficient { real, imag, magnitude, phase }
 * 
 * @example
 * ```typescript
 * const h = rayleighFading();
 * console.log(`|h| = ${h.magnitude}, ∠h = ${h.phase * 180 / Math.PI}°`);
 * ```
 */
export function rayleighFading(): { real: number; imag: number; magnitude: number; phase: number } {
    const real = gaussianRandom(0, 1 / Math.sqrt(2));
    const imag = gaussianRandom(0, 1 / Math.sqrt(2));
    const magnitude = Math.sqrt(real * real + imag * imag);
    const phase = Math.atan2(imag, real);

    return { real, imag, magnitude, phase };
}

/**
 * Generate Batch of Rayleigh Fading Coefficients
 * 
 * @param count - Number of samples
 * @returns Arrays of real, imag, and magnitude
 */
export function rayleighFadingBatch(count: number): { real: number[]; imag: number[]; magnitude: number[] } {
    const real: number[] = [];
    const imag: number[] = [];
    const magnitude: number[] = [];

    for (let i = 0; i < count; i++) {
        const h = rayleighFading();
        real.push(h.real);
        imag.push(h.imag);
        magnitude.push(h.magnitude);
    }

    return { real, imag, magnitude };
}

/**
 * PDF of Rayleigh Distribution
 * 
 * f(r) = (r/σ²) * exp(-r²/2σ²)
 * 
 * @param r - Amplitude value
 * @param sigma - Scale parameter (default 1/√2)
 * @returns PDF value
 */
export function rayleighPdf(r: number, sigma: number = 1 / Math.sqrt(2)): number {
    if (r < 0) return 0;
    const sigma2 = sigma * sigma;
    return (r / sigma2) * Math.exp(-r * r / (2 * sigma2));
}

/**
 * CDF of Rayleigh Distribution
 * 
 * F(r) = 1 - exp(-r²/2σ²)
 * 
 * @param r - Amplitude value
 * @param sigma - Scale parameter
 * @returns CDF value
 */
export function rayleighCdf(r: number, sigma: number = 1 / Math.sqrt(2)): number {
    if (r < 0) return 0;
    return 1 - Math.exp(-r * r / (2 * sigma * sigma));
}

/**
 * Rician Fading Parameters Interface
 */
export interface RicianParams {
    /** K-factor (Linear, not dB) */
    kFactor: number;
    /** Phase of the LOS component (rad), default 0 */
    losPhase?: number;
}

/**
 * Rician Fading Channel Coefficient
 * 
 * h = sqrt(K/(K+1)) * exp(j*θ_los) + sqrt(1/(K+1)) * h_rayleigh
 * 
 * @param params - Rician parameters
 * @returns Complex channel coefficient
 * 
 * @example
 * ```typescript
 * // K = 10 dB
 * const K = Math.pow(10, 10 / 10);  // Linear value = 10
 * const h = ricianFading({ kFactor: K });
 * ```
 */
export function ricianFading(params: RicianParams): { real: number; imag: number; magnitude: number; phase: number } {
    const { kFactor } = params;
    const losPhase = params.losPhase ?? 0;

    // LOS Component
    const losAmplitude = Math.sqrt(kFactor / (kFactor + 1));
    const losReal = losAmplitude * Math.cos(losPhase);
    const losImag = losAmplitude * Math.sin(losPhase);

    // Scattered Component (Rayleigh)
    const scatterAmplitude = Math.sqrt(1 / (kFactor + 1));
    const scatter = rayleighFading();

    // Composite Channel
    const real = losReal + scatterAmplitude * scatter.real;
    const imag = losImag + scatterAmplitude * scatter.imag;
    const magnitude = Math.sqrt(real * real + imag * imag);
    const phase = Math.atan2(imag, real);

    return { real, imag, magnitude, phase };
}

/**
 * Convert K-factor from dB to Linear
 * 
 * @param kFactorDb - K-factor (dB)
 * @returns K-factor (Linear)
 */
export function kFactorFromDb(kFactorDb: number): number {
    return Math.pow(10, kFactorDb / 10);
}

/**
 * Convert K-factor from Linear to dB
 * 
 * @param kFactor - K-factor (Linear)
 * @returns K-factor (dB)
 */
export function kFactorToDb(kFactor: number): number {
    return 10 * Math.log10(kFactor);
}

/**
 * Calculate Doppler Shift
 * 
 * f_d = (v/c) * f_c * cos(θ)
 * 
 * @param velocity - Velocity (m/s)
 * @param carrierFreq - Carrier Frequency (Hz)
 * @param angle - Angle between motion and wave propagation (rad)
 * @returns Doppler Shift (Hz)
 */
export function dopplerShift(velocity: number, carrierFreq: number, angle: number = 0): number {
    const c = 3e8; // Speed of light
    return (velocity / c) * carrierFreq * Math.cos(angle);
}

/**
 * Calculate Maximum Doppler Shift
 * 
 * @param velocity - Velocity (m/s)
 * @param carrierFreq - Carrier Frequency (Hz)
 * @returns Max Doppler Shift (Hz)
 */
export function maxDopplerShift(velocity: number, carrierFreq: number): number {
    const c = 3e8;
    return (velocity / c) * carrierFreq;
}

/**
 * Calculate Coherence Time
 * 
 * T_c ≈ 1 / f_d_max
 * 
 * @param maxDoppler - Max Doppler Shift (Hz)
 * @returns Coherence Time (s)
 */
export function coherenceTime(maxDoppler: number): number {
    return 1 / maxDoppler;
}

/**
 * Calculate Coherence Bandwidth
 * 
 * B_c ≈ 1 / (5 * σ_τ)  (for 0.5 correlation)
 * 
 * @param delaySpread - RMS Delay Spread (s)
 * @returns Coherence Bandwidth (Hz)
 */
export function coherenceBandwidth(delaySpread: number): number {
    return 1 / (5 * delaySpread);
}

