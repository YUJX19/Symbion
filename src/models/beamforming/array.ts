/**
 * @module beamforming/array
 * @description Antenna Array and Beamforming Algorithms
 * 
 * ## Supported Array Types
 * - ULA (Uniform Linear Array)
 * 
 * ## References
 * - Balanis, C. A. (2016). Antenna Theory
 * - Van Trees, H. L. (2002). Optimum Array Processing
 */

/**
 * Array Configuration Parameters
 */
export interface ArrayConfig {
    /** Number of antennas */
    numAntennas: number;
    /** Antenna spacing (in wavelengths) */
    spacing: number;
    /** Beam steering angle (rad) */
    steeringAngle?: number;
}

/**
 * Calculate Array Factor for ULA
 * 
 * AF(θ) = sin(N*ψ/2) / (N*sin(ψ/2))
 * where ψ = k*d*sin(θ) + β
 * 
 * @param theta - Observation angle (rad)
 * @param config - Array configuration
 * @returns Normalized array factor
 * 
 * @example
 * ```typescript
 * const af = arrayFactor(0, { numAntennas: 8, spacing: 0.5 });
 * // af ≈ 1 (Main lobe direction)
 * ```
 */
export function arrayFactor(theta: number, config: ArrayConfig): number {
    const { numAntennas: N, spacing: d } = config;
    const steeringAngle = config.steeringAngle ?? 0;

    const k = 2 * Math.PI;  // Wavenumber (spacing is normalized to wavelength)
    const beta = -k * d * Math.sin(steeringAngle);  // Phase excitation
    const psi = k * d * Math.sin(theta) + beta;

    // Avoid division by zero
    if (Math.abs(Math.sin(psi / 2)) < 1e-10) {
        return 1;
    }

    const af = Math.sin(N * psi / 2) / (N * Math.sin(psi / 2));
    return Math.abs(af);
}

/**
 * Calculate Array Factor in dB
 * 
 * @param theta - Observation angle (rad)
 * @param config - Array configuration
 * @returns Array factor (dB)
 */
export function arrayFactorDb(theta: number, config: ArrayConfig): number {
    const af = arrayFactor(theta, config);
    return 20 * Math.log10(Math.max(af, 1e-10));
}

/**
 * Generate Array Pattern Data
 * 
 * @param config - Array configuration
 * @param numPoints - Number of angular sample points
 * @returns Pattern data { angles, pattern }
 */
export function arrayPattern(
    config: ArrayConfig,
    numPoints: number = 361
): { angles: number[]; pattern: number[]; patternDb: number[] } {
    const angles: number[] = [];
    const pattern: number[] = [];
    const patternDb: number[] = [];

    for (let i = 0; i < numPoints; i++) {
        const theta = -Math.PI / 2 + (Math.PI * i) / (numPoints - 1);
        angles.push(theta);
        const af = arrayFactor(theta, config);
        pattern.push(af);
        patternDb.push(20 * Math.log10(Math.max(af, 1e-10)));
    }

    return { angles, pattern, patternDb };
}

/**
 * Calculate Half Power Beamwidth (HPBW)
 * 
 * HPBW ≈ 0.886 * λ / (N * d)
 * 
 * @param config - Array configuration
 * @returns HPBW (rad)
 */
export function halfPowerBeamwidth(config: ArrayConfig): number {
    const { numAntennas: N, spacing: d } = config;
    const steeringAngle = config.steeringAngle ?? 0;

    // Exact formula for broadside array
    if (Math.abs(steeringAngle) < 1e-6) {
        return 0.886 / (N * d);
    }

    // Scanned array approximation (broadening factor)
    return 0.886 / (N * d * Math.cos(steeringAngle));
}

/**
 * Calculate First Null Beamwidth (FNBW)
 * 
 * FNBW ≈ 2 * λ / (N * d)
 * 
 * @param config - Array configuration
 * @returns FNBW (rad)
 */
export function firstNullBeamwidth(config: ArrayConfig): number {
    const { numAntennas: N, spacing: d } = config;
    return 2 / (N * d);
}

/**
 * Calculate Array Gain (Directivity) in Linear Scale
 * 
 * For ULA with d=0.5λ: G ≈ N
 * 
 * **Note**: This function returns **linear gain**. For dB, use `arrayGainDb()`.
 * 
 * @param config - Array configuration
 * @returns Linear gain (unitless)
 * 
 * @example
 * ```typescript
 * const config = { numAntennas: 8, spacing: 0.5 };
 * const gainLinear = arrayGain(config);  // Returns: 8
 * const gainDb = arrayGainDb(config);     // Returns: 9.03 dB
 * ```
 */
export function arrayGain(config: ArrayConfig): number {
    // Simplified approximation for standard spacing
    return config.numAntennas;
}

/**
 * Calculate Array Gain (Directivity) in dB
 * 
 * G_dB = 10 * log₁₀(N)
 * 
 * @param config - Array configuration
 * @returns Gain in dB
 * 
 * @example
 * ```typescript
 * const config = { numAntennas: 8, spacing: 0.5 };
 * const gainDb = arrayGainDb(config);  // Returns: 9.03 dB
 * ```
 */
export function arrayGainDb(config: ArrayConfig): number {
    return 10 * Math.log10(config.numAntennas);
}

/**
 * Calculate Steering Phases (for Phase Shifters)
 * 
 * @param targetAngle - Target beam angle (rad)
 * @param config - Array configuration
 * @returns Phase shifts for each element (rad)
 */
export function steeringPhases(targetAngle: number, config: ArrayConfig): number[] {
    const { numAntennas: N, spacing: d } = config;
    const k = 2 * Math.PI;
    const beta = -k * d * Math.sin(targetAngle);

    const phases: number[] = [];
    for (let n = 0; n < N; n++) {
        // Phase shift: β_n = n * β
        let phi = n * beta;
        // Wrap to [-π, π]
        phi = ((phi + Math.PI) % (2 * Math.PI)) - Math.PI;
        phases.push(phi);
    }
    return phases;
}

/**
 * Check for grating lobes in antenna array
 * 
 * @param spacing - Antenna spacing (in wavelengths)
 * @param maxScanAngle - Maximum scan angle (rad)
 * @returns Whether grating lobes exist
 */
export function hasGratingLobes(spacing: number, maxScanAngle: number): boolean {
    const maxSpacing = 1 / (1 + Math.abs(Math.sin(maxScanAngle)));
    return spacing > maxSpacing;
}

