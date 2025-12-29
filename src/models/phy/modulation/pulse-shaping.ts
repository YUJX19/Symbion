/**
 * @module modulation/pulse-shaping
 * @description Pulse Shaping Filters (Raised Cosine, Root Raised Cosine)
 * 
 * ## Theoretical Background
 * Pulse shaping filters are used to limit signal bandwidth and reduce Inter-Symbol Interference (ISI).
 * Raised Cosine filter satisfies Nyquist ISI criterion (zero ISI at sampling points).
 * 
 * ## References
 * - Proakis, J. G. (2008). Digital Communications
 */

/**
 * Raised Cosine Pulse
 * 
 * h(t) = sinc(t/T) * cos(π*α*t/T) / (1 - (2*α*t/T)²)
 * 
 * @param t - Time
 * @param symbolPeriod - Symbol period T
 * @param rolloff - Rolloff factor α (0 ≤ α ≤ 1)
 * @returns Pulse value
 */
export function raisedCosine(t: number, symbolPeriod: number, rolloff: number): number {
    const T = symbolPeriod;
    const alpha = rolloff;

    // Handle singularity points
    if (Math.abs(t) < 1e-10) {
        return 1;
    }

    // Handle case where denominator is zero (1 - (2αt/T)² = 0)
    const denominator = 1 - Math.pow(2 * alpha * t / T, 2);
    if (Math.abs(denominator) < 1e-10) {
        return (alpha / 2) * Math.sin(Math.PI / (2 * alpha));
    }

    const sincPart = Math.sin(Math.PI * t / T) / (Math.PI * t / T);
    const cosPart = Math.cos(Math.PI * alpha * t / T);

    return sincPart * cosPart / denominator;
}

/**
 * Root Raised Cosine Pulse
 * 
 * @param t - Time
 * @param symbolPeriod - Symbol period T
 * @param rolloff - Rolloff factor α
 * @returns Pulse value
 */
export function rootRaisedCosine(t: number, symbolPeriod: number, rolloff: number): number {
    const T = symbolPeriod;
    const alpha = rolloff;

    // Special case t = 0
    if (Math.abs(t) < 1e-10) {
        return (1 / T) * (1 + alpha * (4 / Math.PI - 1));
    }

    // Special case t = ±T/(4α)
    if (Math.abs(Math.abs(t) - T / (4 * alpha)) < 1e-10) {
        const a = (1 + 2 / Math.PI) * Math.sin(Math.PI / (4 * alpha));
        const b = (1 - 2 / Math.PI) * Math.cos(Math.PI / (4 * alpha));
        return (alpha / (T * Math.sqrt(2))) * (a + b);
    }

    const piT = Math.PI * t / T;
    const fourAlphaT = 4 * alpha * t / T;

    const numerator = Math.sin(piT * (1 - alpha)) + fourAlphaT * Math.cos(piT * (1 + alpha));
    const denominator = piT * (1 - fourAlphaT * fourAlphaT);

    return numerator / denominator;
}

/**
 * Generate Raised Cosine Filter Coefficients
 * 
 * @param symbolPeriod - Symbol period (s)
 * @param rolloff - Rolloff factor α
 * @param sampleRate - Sample rate (Hz)
 * @param numSymbols - Number of symbols spanned by filter (one-sided)
 * @returns Filter coefficients
 */
export function raisedCosineFilter(
    symbolPeriod: number,
    rolloff: number,
    sampleRate: number,
    numSymbols: number = 6
): number[] {
    const samplesPerSymbol = Math.round(symbolPeriod * sampleRate);
    const halfLength = numSymbols * samplesPerSymbol;
    const length = 2 * halfLength + 1;

    const filter = new Array(length);

    for (let i = 0; i < length; i++) {
        const n = i - halfLength;
        const t = n / sampleRate;
        filter[i] = raisedCosine(t, symbolPeriod, rolloff);
    }

    // Normalization
    const sum = filter.reduce((a, b) => a + b, 0);
    return filter.map(x => x / sum * samplesPerSymbol);
}

/**
 * Generate Root Raised Cosine Filter Coefficients
 * 
 * @param symbolPeriod - Symbol period (s)
 * @param rolloff - Rolloff factor α
 * @param sampleRate - Sample rate (Hz)
 * @param numSymbols - Number of symbols spanned by filter (one-sided)
 * @returns Filter coefficients
 */
export function rootRaisedCosineFilter(
    symbolPeriod: number,
    rolloff: number,
    sampleRate: number,
    numSymbols: number = 6
): number[] {
    const samplesPerSymbol = Math.round(symbolPeriod * sampleRate);
    const halfLength = numSymbols * samplesPerSymbol;
    const length = 2 * halfLength + 1;

    const filter = new Array(length);

    for (let i = 0; i < length; i++) {
        const n = i - halfLength;
        const t = n / sampleRate;
        filter[i] = rootRaisedCosine(t, symbolPeriod, rolloff);
    }

    // Energy normalization
    const energy = filter.reduce((a, b) => a + b * b, 0);
    const factor = 1 / Math.sqrt(energy);
    return filter.map(x => x * factor);
}

/**
 * Calculate Raised Cosine Signal Bandwidth
 * 
 * B = (1 + α) / (2T)
 * 
 * @param symbolPeriod - Symbol period (s)
 * @param rolloff - Rolloff factor α
 * @returns Bandwidth (Hz)
 */
export function raisedCosineBandwidth(symbolPeriod: number, rolloff: number): number {
    return (1 + rolloff) / (2 * symbolPeriod);
}

/**
 * Calculate Maximum Symbol Rate from Bandwidth
 * 
 * For raised cosine pulse shaping, the relationship between symbol rate and bandwidth is:
 * - Single-sided bandwidth: B = R_s(1 + α)/2
 * - Solving for R_s: R_s = 2B/(1 + α)
 * 
 * **Note**: This function interprets the input `bandwidth` as **single-sided bandwidth**.
 * If you have the total (double-sided) bandwidth B_total, use: 
 * `maxSymbolRate(B_total / 2, rolloff)` or simply `B_total / (1 + rolloff)`.
 * 
 * @param bandwidth - Single-sided bandwidth (Hz)
 * @param rolloff - Rolloff factor α (0 ≤ α ≤ 1)
 * @returns Maximum symbol rate R_s (symbols/s)
 * 
 * @example
 * ```typescript
 * // Single-sided bandwidth = 500 kHz, rolloff = 0.25
 * const symbolRate = maxSymbolRate(500e3, 0.25);
 * // Returns: 800,000 symbols/s
 * 
 * // If you have total bandwidth = 1000 kHz
 * const symbolRate2 = 1000e3 / (1 + 0.25);
 * // Returns: 800,000 symbols/s (alternative calculation)
 * ```
 */
export function maxSymbolRate(bandwidth: number, rolloff: number): number {
    return 2 * bandwidth / (1 + rolloff);
}

/**
 * sinc function
 */
export function sinc(x: number): number {
    if (Math.abs(x) < 1e-10) return 1;
    return Math.sin(Math.PI * x) / (Math.PI * x);
}

/**
 * Rectangular Pulse (NRZ)
 * 
 * @param t - Time
 * @param symbolPeriod - Symbol period T
 * @returns Pulse value
 */
export function rectangularPulse(t: number, symbolPeriod: number): number {
    return Math.abs(t) <= symbolPeriod / 2 ? 1 : 0;
}
