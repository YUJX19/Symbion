/**
 * @module beamforming/mimo
 * @description Massive MIMO Precoding Algorithms
 * 
 * ## Supported Precoding Schemes
 * - MRT (Maximum Ratio Transmission)
 * - ZF (Zero-Forcing)
 * 
 * ## References
 * - Marzetta, T. L. (2010). Noncooperative Cellular Wireless with Unlimited Numbers of Base Station Antennas
 * - Rusek, F. et al. (2013). Scaling Up MIMO
 */

import { gaussianRandom } from '../utils/statistics';

/**
 * MIMO System Configuration
 */
export interface MIMOConfig {
    /** Number of Base Station Antennas (M) */
    numAntennas: number;
    /** Number of Users (K) */
    numUsers: number;
    /** Signal-to-Noise Ratio (Linear) */
    snrLinear: number;
}

/**
 * Complex Matrix Representation
 */
export interface ComplexMatrix {
    real: number[][];
    imag: number[][];
}

/**
 * Generate Random Rayleigh Fading Channel Matrix
 * 
 * H is an M x K complex matrix, elements ~ CN(0, 1)
 * 
 * @param numAntennas - BS antennas M
 * @param numUsers - Users K
 * @returns Channel Matrix H
 */
export function generateChannelMatrix(numAntennas: number, numUsers: number): ComplexMatrix {
    const real: number[][] = [];
    const imag: number[][] = [];

    for (let m = 0; m < numAntennas; m++) {
        const rowReal: number[] = [];
        const rowImag: number[] = [];

        for (let k = 0; k < numUsers; k++) {
            rowReal.push(gaussianRandom(0, 1 / Math.sqrt(2)));
            rowImag.push(gaussianRandom(0, 1 / Math.sqrt(2)));
        }

        real.push(rowReal);
        imag.push(rowImag);
    }

    return { real, imag };
}

/**
 * Calculate Hermitian Transpose H^H
 * 
 * @param H - Channel Matrix (M x K)
 * @returns H^H (K x M)
 */
export function hermitianTranspose(H: ComplexMatrix): ComplexMatrix {
    const M = H.real.length;
    const K = H.real[0].length;

    const real: number[][] = [];
    const imag: number[][] = [];

    for (let k = 0; k < K; k++) {
        const rowReal: number[] = [];
        const rowImag: number[] = [];

        for (let m = 0; m < M; m++) {
            rowReal.push(H.real[m][k]);
            rowImag.push(-H.imag[m][k]);  // Conjugate
        }

        real.push(rowReal);
        imag.push(rowImag);
    }

    return { real, imag };
}

/**
 * Calculate SINR for MRT Precoding
 * 
 * SINR_k = M * SNR / (K - 1 + 1/SNR)  (Large M approximation)
 * 
 * @param config - MIMO Configuration
 * @returns SINR for each user (Linear)
 */
export function mrtSinr(config: MIMOConfig): number {
    const { numAntennas: M, numUsers: K, snrLinear } = config;

    // Simplified formula when M >> K
    return M * snrLinear / (K - 1 + 1 / snrLinear + 1);
}

/**
 * Calculate SINR for Zero-Forcing (ZF) Precoding
 * 
 * SINR_k = (M - K + 1) * SNR  (Perfect CSI)
 * 
 * @param config - MIMO Configuration
 * @returns SINR for each user (Linear)
 */
export function zfSinr(config: MIMOConfig): number {
    const { numAntennas: M, numUsers: K, snrLinear } = config;

    // ZF eliminates inter-user interference but loses M spatial degrees of freedom
    const effectiveAntennas = Math.max(M - K + 1, 1);
    return effectiveAntennas * snrLinear;
}

/**
 * Calculate Capacity for Precoding Scheme
 * 
 * C = K * log2(1 + SINR)  (Sum capacity)
 * 
 * @param precodingType - Precoding type
 * @param config - MIMO Configuration
 * @returns Sum Capacity (bps/Hz)
 */
export function capacity(precodingType: 'MRT' | 'ZF', config: MIMOConfig): number {
    const sinr = precodingType === 'MRT' ? mrtSinr(config) : zfSinr(config);
    return config.numUsers * Math.log2(1 + sinr);
}

/**
 * Calculate Capacity Scaling with Antennas
 * 
 * @param precodingType - Precoding type
 * @param baseConfig - Base configuration
 * @param antennaRange - Range [min, max]
 * @returns Capacity vs Antennas data
 */
export function capacityVsAntennas(
    precodingType: 'MRT' | 'ZF',
    baseConfig: Omit<MIMOConfig, 'numAntennas'>,
    antennaRange: [number, number]
): { antennas: number[]; capacity: number[] } {
    const antennas: number[] = [];
    const capacities: number[] = [];

    for (let M = antennaRange[0]; M <= antennaRange[1]; M++) {
        antennas.push(M);
        capacities.push(capacity(precodingType, { ...baseConfig, numAntennas: M }));
    }

    return { antennas, capacity: capacities };
}

/**
 * Calculate Crossover Point (Where ZF outperforms MRT)
 * 
 * @param numUsers - Number of users
 * @param snrDb - SNR (dB)
 * @returns Number of antennas M
 */
export function mrtZfCrossover(numUsers: number, snrDb: number): number {
    const snrLinear = Math.pow(10, snrDb / 10);

    // Solve MRT SINR = ZF SINR
    // M*SNR/(K-1+1/SNR+1) = (M-K+1)*SNR
    // Approx: When SNR is high, M ≈ K^2 / (K-1)

    for (let M = numUsers; M <= 1000; M++) {
        const mrtS = mrtSinr({ numAntennas: M, numUsers, snrLinear });
        const zfS = zfSinr({ numAntennas: M, numUsers, snrLinear });

        if (zfS >= mrtS) {
            return M;
        }
    }

    return numUsers;  // Default
}

/**
 * Equal Power Allocation
 * 
 * Power per user = P_total / K
 * 
 * @param totalPower - Total Tx Power (W)
 * @param numUsers - Number of Users
 * @returns Power per user (W)
 */
export function equalPowerAllocation(totalPower: number, numUsers: number): number {
    return totalPower / numUsers;
}

/**
 * Calculate Asymptotic Efficiency
 * 
 * As M → ∞, ZF Capacity approaches: K * log2(1 + M * SNR)
 * 
 * @param config - MIMO Configuration
 * @returns Efficiency ratio (relative to interference-free upper bound)
 */
export function asymptoticEfficiency(config: MIMOConfig): number {
    const { numAntennas: M, numUsers: K, snrLinear } = config;

    // Interference-free upper bound
    const upperBound = K * Math.log2(1 + M * snrLinear);

    // Actual ZF Capacity
    const actualCapacity = capacity('ZF', config);

    return actualCapacity / upperBound;
}
