/**
 * @module tasks/isac-trajectory/task1/uav-comm-model
 * @description UAV-to-UAV Communication Model based on ITU-R
 * 
 * Implements paper equations 27-30:
 * - Path Loss (LoS/NLoS) for air-to-air links
 * - LoS Probability for UAV-UAV communication
 * 
 * This is a copy of the parent module for task1 encapsulation.
 */

/**
 * ITU parameters for urban/suburban environments (Table III from paper)
 */
export interface ITUParameters {
    a1: number;  // Building density
    a2: number;  // Average building height
    a3: number;  // Building height std deviation
}

export const ITU_PARAMS = {
    urban: { a1: 0.3, a2: 500, a3: 20 },
    suburban: { a1: 0.1, a2: 750, a3: 8 },
    rural: { a1: 0.002, a2: 300, a3: 4 }
};

/**
 * UAV-UAV LoS Probability (Eq. 29-30)
 * 
 * p^L_xu(r) = Π_{j=0}^M [1 - exp(-(h_x - (j+0.5)(h_x-h_u)/(k+1))² / (2*a3²))]
 * 
 * @param dist3D - 3D distance between UAVs (meters)
 * @param height1 - Height of first UAV (meters)
 * @param height2 - Height of second UAV (meters)
 * @param params - ITU environment parameters
 */
export function calculateLosProbabilityUAV(
    dist3D: number,
    height1: number,
    height2: number,
    params: ITUParameters = ITU_PARAMS.suburban
): number {
    const { a1, a2, a3 } = params;

    // Ensure heights are positive
    const h_x = Math.max(height1, height2);
    const h_u = Math.min(height1, height2);

    // M and k from paper (Eq. 29)
    const factor = (dist3D * Math.sqrt(a1 * a2)) / 1000;
    const M = Math.floor(factor - 1);
    const k = Math.floor(factor + 2);

    // For very close UAVs or high altitudes, LoS is guaranteed
    if (M < 0) return 1.0;

    let probability = 1.0;
    for (let j = 0; j <= M; j++) {
        const term = h_x - ((j + 0.5) * (h_x - h_u)) / (k + 1);
        const exponent = -(term * term) / (2 * a3 * a3);
        probability *= (1 - Math.exp(exponent));
    }

    return Math.max(0, Math.min(1, probability));
}

/**
 * UAV-UAV Path Loss - LoS (Eq. 27)
 * 
 * PL^L_k = τ_L + 22*log10(r_k)
 * where τ_L = 28.0 + 20*log10(f_c)
 * 
 * @param dist - Distance in meters
 * @param fcGHz - Carrier frequency in GHz
 */
export function calculatePathLossLoS(dist: number, fcGHz: number): number {
    const tau_L = 28.0 + 20 * Math.log10(fcGHz);
    return tau_L + 22 * Math.log10(Math.max(1, dist));
}

/**
 * UAV-UAV Path Loss - NLoS (Eq. 28)
 * 
 * PL^N_k = τ_N + α_N*log10(r_k)
 * where τ_N = -17.5 + 20*log10(40*π*f_c/3)
 *       α_N = 46 - 7*log10(h_UT)
 * 
 * @param dist - Distance in meters
 * @param fcGHz - Carrier frequency in GHz
 * @param hUT - Height of user terminal (UAV) in meters
 */
export function calculatePathLossNLoS(dist: number, fcGHz: number, hUT: number): number {
    const tau_N = -17.5 + 20 * Math.log10((40 * Math.PI * fcGHz) / 3);
    const alpha_N = 46 - 7 * Math.log10(Math.max(1, hUT));
    return tau_N + alpha_N * Math.log10(Math.max(1, dist));
}

/**
 * Expected Path Loss combining LoS and NLoS (Eq. 27-30)
 * 
 * E[PL] = p_LoS * PL_LoS + (1 - p_LoS) * PL_NLoS
 */
export function calculateExpectedPathLoss(
    dist3D: number,
    height1: number,
    height2: number,
    fcGHz: number,
    params: ITUParameters = ITU_PARAMS.suburban
): { expected: number; pLoS: number; plLoS: number; plNLoS: number } {
    const pLoS = calculateLosProbabilityUAV(dist3D, height1, height2, params);
    const plLoS = calculatePathLossLoS(dist3D, fcGHz);
    const plNLoS = calculatePathLossNLoS(dist3D, fcGHz, Math.min(height1, height2));

    const expected = pLoS * plLoS + (1 - pLoS) * plNLoS;

    return { expected, pLoS, plLoS, plNLoS };
}

/**
 * SINR calculation
 */
export function calculateSINR(
    txPowerDbm: number,
    pathLossDb: number,
    noisePowerDbm: number
): number {
    const rxPowerDbm = txPowerDbm - pathLossDb;
    return rxPowerDbm - noisePowerDbm;
}

/**
 * Shannon Capacity
 */
export function calculateCapacity(bandwidthHz: number, sinrDb: number): number {
    const sinrLinear = Math.pow(10, sinrDb / 10);
    return bandwidthHz * Math.log2(1 + Math.max(0, sinrLinear));
}

/**
 * LoS Geometric Constraint (Eq. 31)
 * 
 * LoS_k = { x ∈ R³ | <x - z_{I,k}, z_{J,k}> ≤ θ_k }
 * 
 * Checks if angle between (x - z_I) and z_J is within θ_k
 */
export function checkLoSConeConstraint(
    uavPos: [number, number, number],
    targetPos: [number, number, number],
    coneAxis: [number, number, number],
    halfAngleRad: number
): { satisfied: boolean; violation: number } {
    // Vector from target to UAV
    const dx = uavPos[0] - targetPos[0];
    const dy = uavPos[1] - targetPos[1];
    const dz = uavPos[2] - targetPos[2];
    const d = Math.sqrt(dx * dx + dy * dy + dz * dz);

    if (d < 1e-6) return { satisfied: true, violation: 0 };

    // Normalize
    const nx = dx / d;
    const ny = dy / d;
    const nz = dz / d;

    // Dot product with cone axis
    const aMag = Math.sqrt(coneAxis[0] ** 2 + coneAxis[1] ** 2 + coneAxis[2] ** 2);
    const dot = (nx * coneAxis[0] + ny * coneAxis[1] + nz * coneAxis[2]) / aMag;

    // Angle between vectors
    const angle = Math.acos(Math.max(-1, Math.min(1, dot)));

    if (angle <= halfAngleRad) {
        return { satisfied: true, violation: 0 };
    } else {
        return { satisfied: false, violation: angle - halfAngleRad };
    }
}
