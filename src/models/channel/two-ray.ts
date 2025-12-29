/**
 * @module channel/two-ray
 * @description Two-Ray Ground Reflection Model
 * 
 * Considers the superposition of the direct path and the ground reflected path,
 * applicable for near-ground propagation scenarios.
 * 
 * ## Theoretical Background
 * The Two-Ray model assumes the signal reaches the receiver via two paths:
 * 1. Line-of-Sight (LOS) path
 * 2. Ground reflected path
 * 
 * The phase difference between the two paths leads to constructive or destructive interference.
 * 
 * ## References
 * - Rappaport, T. S. (2002). Wireless Communications
 */

/**
 * Two-Ray Model Parameters Interface
 */
export interface TwoRayParams {
    /** Transmitter height (m) */
    txHeight: number;
    /** Receiver height (m) */
    rxHeight: number;
    /** Horizontal distance (m) */
    distance: number;
    /** Wavelength (m) */
    wavelength: number;
    /** Transmit power (W), default 1 */
    txPower?: number;
    /** Ground reflection coefficient, default -1 (perfect conductor) */
    reflectionCoeff?: number;
}

/**
 * Two-Ray Model Calculation Result
 */
export interface TwoRayResult {
    /** LOS path length (m) */
    dLos: number;
    /** Reflected path length (m) */
    dRef: number;
    /** Path difference (m) */
    pathDiff: number;
    /** Phase difference (rad) */
    phaseDiff: number;
    /** Received power (Normalized) */
    receivedPower: number;
    /** Received power (dB) */
    receivedPowerDb: number;
}

/**
 * Calculate Breakpoint Distance
 * 
 * Beyond the breakpoint distance d_break, path loss decays as d^-4 instead of d^-2.
 * 
 * @param txHeight - Transmitter height (m)
 * @param rxHeight - Receiver height (m)
 * @param wavelength - Wavelength (m)
 * @returns Breakpoint distance (m)
 * 
 * @example
 * ```typescript
 * const dBreak = breakpointDistance(50, 2, 0.125);  // 50m BS, 2m UE, 2.4GHz
 * // dBreak = 3200m
 * ```
 */
export function breakpointDistance(
    txHeight: number,
    rxHeight: number,
    wavelength: number
): number {
    return (4 * txHeight * rxHeight) / wavelength;
}

/**
 * Calculate LOS Path Length
 */
export function losDistance(distance: number, txHeight: number, rxHeight: number): number {
    return Math.sqrt(distance * distance + Math.pow(txHeight - rxHeight, 2));
}

/**
 * Calculate Reflected Path Length
 */
export function reflectedDistance(distance: number, txHeight: number, rxHeight: number): number {
    return Math.sqrt(distance * distance + Math.pow(txHeight + rxHeight, 2));
}

/**
 * Calculate Full Two-Ray Model
 * 
 * @param params - Two-Ray model parameters
 * @returns Calculation result
 * 
 * @example
 * ```typescript
 * const result = calculate({
 *     txHeight: 50,
 *     rxHeight: 2,
 *     distance: 1000,
 *     wavelength: 0.125  // 2.4 GHz
 * });
 * console.log(`Received Power: ${result.receivedPowerDb.toFixed(2)} dB`);
 * ```
 */
export function calculate(params: TwoRayParams): TwoRayResult {
    const { txHeight, rxHeight, distance, wavelength } = params;
    const txPower = params.txPower ?? 1;
    const reflectionCoeff = params.reflectionCoeff ?? -1;

    // Path lengths
    const dLos = losDistance(distance, txHeight, rxHeight);
    const dRef = reflectedDistance(distance, txHeight, rxHeight);
    const pathDiff = dRef - dLos;

    // Phase difference
    const phaseDiff = (2 * Math.PI * pathDiff) / wavelength;

    // Receive Field Strength (Normalized)
    // E_total = E_los + Γ * E_ref * exp(j*phaseDiff)
    const eLos = 1 / dLos;
    const eRef = reflectionCoeff / dRef;

    // Power = |E_los + E_ref * exp(j*phaseDiff)|^2
    const realPart = eLos + eRef * Math.cos(phaseDiff);
    const imagPart = eRef * Math.sin(phaseDiff);
    const receivedPower = txPower * (realPart * realPart + imagPart * imagPart);
    const receivedPowerDb = 10 * Math.log10(receivedPower);

    return {
        dLos,
        dRef,
        pathDiff,
        phaseDiff,
        receivedPower,
        receivedPowerDb
    };
}

/**
 * Generate Power vs Distance Curve Data
 * 
 * @param txHeight - Transmitter height (m)
 * @param rxHeight - Receiver height (m)
 * @param wavelength - Wavelength (m)
 * @param distances - Array of distances (m)
 * @returns Array of received power (dB)
 */
export function powerVsDistance(
    txHeight: number,
    rxHeight: number,
    wavelength: number,
    distances: number[]
): number[] {
    return distances.map(d => {
        const result = calculate({
            txHeight,
            rxHeight,
            distance: d,
            wavelength
        });
        return result.receivedPowerDb;
    });
}

/**
 * Calculate Free Space Path Loss (For Comparison)
 * 
 * @param distance - Distance (m)
 * @param wavelength - Wavelength (m)
 * @returns Path Loss (dB)
 */
export function freeSpacePathLoss(distance: number, wavelength: number): number {
    return 20 * Math.log10((4 * Math.PI * distance) / wavelength);
}

