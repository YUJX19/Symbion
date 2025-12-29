/**
 * @module channel/types
 * @description Type definitions for channel modeling
 */

/**
 * AWGN channel configuration
 */
export interface AWGNConfig {
    /** Signal-to-noise ratio in dB */
    snrDb: number;
    /** Channel bandwidth in Hz */
    bandwidth?: number;
}

/**
 * Fading channel configuration
 */
export interface FadingConfig {
    /** Type of fading: 'rayleigh' | 'rician' */
    type: 'rayleigh' | 'rician';
    /** K-factor for Rician fading */
    kFactor?: number;
    /** Maximum Doppler frequency in Hz */
    maxDoppler?: number;
    /** Number of paths for multipath */
    numPaths?: number;
}

/**
 * Channel impulse response
 */
export interface ChannelResponse {
    /** Complex tap coefficients */
    taps: { real: number; imag: number }[];
    /** Tap delays in seconds */
    delays: number[];
    /** Tap powers in linear scale */
    powers: number[];
}
