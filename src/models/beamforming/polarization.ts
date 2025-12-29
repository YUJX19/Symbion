/**
 * @module beamforming/polarization
 * @description Antenna Polarization and Polarization Matching
 * 
 * ## Background
 * Polarization describes the orientation of the electric field vector. 
 * Mismatch between Tx and Rx antenna polarization leads to power loss.
 * 
 * ## References
 * - Balanis, C. A. (2016). Antenna Theory
 */

/**
 * Polarization Types
 */
export type PolarizationType = 'V' | 'H' | 'LHCP' | 'RHCP' | '45' | '-45';

/**
 * Calculate Polarization Loss Factor (PLF)
 * 
 * PLF = |ρ_t · ρ_r*|²
 * 
 * @param txPol - Tx antenna polarization angle (rad)
 * @param rxPol - Rx antenna polarization angle (rad)
 * @returns PLF (0 to 1)
 * 
 * @example
 * ```typescript
 * const plf = polarizationLossFactor(0, 0);        // Co-polarized: 1
 * const plf = polarizationLossFactor(0, Math.PI/2); // Cross-polarized: 0
 * ```
 */
export function polarizationLossFactor(txPol: number, rxPol: number): number {
    return Math.pow(Math.cos(txPol - rxPol), 2);
}

/**
 * Calculate Polarization Mismatch Loss in dB
 * 
 * @param txPol - Tx antenna polarization angle (rad)
 * @param rxPol - Rx antenna polarization angle (rad)
 * @returns Loss (dB)
 */
export function polarizationLossDb(txPol: number, rxPol: number): number {
    const plf = polarizationLossFactor(txPol, rxPol);
    if (plf < 1e-10) return -Infinity;  // Complete cross-polarization
    return 10 * Math.log10(plf);
}

/**
 * Convert Polarization Type to Angle
 * 
 * @param type - Polarization type
 * @returns Angle (rad)
 */
export function polarizationTypeToAngle(type: PolarizationType): number {
    switch (type) {
        case 'V': return 0;
        case 'H': return Math.PI / 2;
        case '45': return Math.PI / 4;
        case '-45': return -Math.PI / 4;
        case 'LHCP':
        case 'RHCP':
            return 0;  // Circular polarization needs special handling (Jones vectors)
        default:
            return 0;
    }
}

/**
 * Calculate PLF between two polarization types
 * 
 * @param txType - Tx polarization type
 * @param rxType - Rx polarization type
 * @returns PLF
 */
export function polarizationMatch(txType: PolarizationType, rxType: PolarizationType): number {
    // Special handling for linear vs circular
    const circPols: PolarizationType[] = ['LHCP', 'RHCP'];
    const linPols: PolarizationType[] = ['V', 'H', '45', '-45'];

    if (circPols.includes(txType) && circPols.includes(rxType)) {
        // Same circular direction: PLF = 1
        // Opposite circular direction: PLF = 0
        return txType === rxType ? 1 : 0;
    }

    if (circPols.includes(txType) && linPols.includes(rxType)) {
        // Circular vs Linear: PLF = 0.5 (-3 dB)
        return 0.5;
    }

    if (linPols.includes(txType) && circPols.includes(rxType)) {
        return 0.5;
    }

    // Two linear polarizations
    const txAngle = polarizationTypeToAngle(txType);
    const rxAngle = polarizationTypeToAngle(rxType);
    return polarizationLossFactor(txAngle, rxAngle);
}

/**
 * Jones Vector Representation
 */
export interface JonesVector {
    /** Horizontal component (Real part) */
    hReal: number;
    /** Horizontal component (Imaginary part) */
    hImag: number;
    /** Vertical component (Real part) */
    vReal: number;
    /** Vertical component (Imaginary part) */
    vImag: number;
}

/**
 * Generate Jones Vector for Polarization Type
 * 
 * @param type - Polarization type
 * @returns Jones Vector
 */
export function jonesVector(type: PolarizationType): JonesVector {
    switch (type) {
        case 'V':
            return { hReal: 0, hImag: 0, vReal: 1, vImag: 0 };
        case 'H':
            return { hReal: 1, hImag: 0, vReal: 0, vImag: 0 };
        case '45':
            return { hReal: 1 / Math.sqrt(2), hImag: 0, vReal: 1 / Math.sqrt(2), vImag: 0 };
        case '-45':
            return { hReal: 1 / Math.sqrt(2), hImag: 0, vReal: -1 / Math.sqrt(2), vImag: 0 };
        case 'LHCP':
            return { hReal: 1 / Math.sqrt(2), hImag: 0, vReal: 0, vImag: 1 / Math.sqrt(2) };
        case 'RHCP':
            return { hReal: 1 / Math.sqrt(2), hImag: 0, vReal: 0, vImag: -1 / Math.sqrt(2) };
    }
}

/**
 * Calculate Inner Product of Jones Vectors (Polarization Match)
 * 
 * @param j1 - Jones Vector 1
 * @param j2 - Jones Vector 2
 * @returns Squared magnitude of complex inner product
 */
export function jonesInnerProduct(j1: JonesVector, j2: JonesVector): number {
    // <j1, j2*> = h1*h2_conj + v1*v2_conj
    const hReal = j1.hReal * j2.hReal + j1.hImag * j2.hImag;
    const hImag = j1.hImag * j2.hReal - j1.hReal * j2.hImag;
    const vReal = j1.vReal * j2.vReal + j1.vImag * j2.vImag;
    const vImag = j1.vImag * j2.vReal - j1.vReal * j2.vImag;

    const realSum = hReal + vReal;
    const imagSum = hImag + vImag;

    return realSum * realSum + imagSum * imagSum;
}

/**
 * Cross-Polarization Discrimination (XPD)
 * 
 * @param coPolarization - Co-polarization power
 * @param crossPolarization - Cross-polarization power
 * @returns XPD (dB)
 */
export function crossPolarizationDiscrimination(
    coPolarization: number,
    crossPolarization: number
): number {
    return 10 * Math.log10(coPolarization / crossPolarization);
}

