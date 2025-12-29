/**
 * @module src/extras/phy-coding
 * @description Advanced Physical Layer Coding (Experimental)
 *
 * High-maintenance coding implementations moved from core.
 *
 * Contains:
 * - LDPC: Low-Density Parity-Check codes
 * - Turbo: Turbo codes with interleaver
 * - Polar: Polar codes
 *
 * Note: For most ISAC research, use link abstraction (BLER curves, effective SINR mapping)
 * instead of full coding simulation. These are provided for specialized research.
 *
 * Core coding (CRC, Hamming, Viterbi) remains in models/phy.
 */

// Re-export from phy/coding module
export * from '../../models/phy/coding';
