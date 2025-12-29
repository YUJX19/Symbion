/**
 * @module modulation/types
 * @description Type definitions for modulation algorithms
 */

/**
 * QAM modulation configuration
 */
export interface QAMConfig {
    /** Modulation order (4, 16, 64, 256, etc.) */
    order: number;
    /** Symbol rate in Baud */
    symbolRate?: number;
}

/**
 * PSK modulation configuration
 */
export interface PSKConfig {
    /** Modulation order (2 for BPSK, 4 for QPSK, 8 for 8-PSK, etc.) */
    order: number;
}

/**
 * OTFS modulation configuration
 */
export interface OTFSConfig {
    /** Number of Doppler bins */
    numDoppler: number;
    /** Number of delay bins */
    numDelay: number;
    /** Subcarrier spacing in Hz */
    subcarrierSpacing: number;
    /** Symbol duration in seconds */
    symbolDuration: number;
}

/**
 * Complex number representation
 */
export interface Complex {
    real: number;
    imag: number;
}

/**
 * Constellation point
 */
export interface ConstellationPoint {
    /** In-phase component */
    i: number;
    /** Quadrature component */
    q: number;
    /** Symbol label (bit string) */
    label?: string;
}
