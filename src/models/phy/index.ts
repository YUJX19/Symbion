/**
 * @module src/models/phy
 * @description Physical Layer Signal Processing
 *
 * Merged from: modulation + coding + spreading
 *
 * Contains:
 * - Modulation: BPSK, QPSK, QAM, OFDM, OTFS
 * - Coding: CRC, Hamming, Viterbi, Convolutional
 * - Spreading: Gold, Zadoff-Chu sequences
 */

import * as modulation from './modulation';
import * as coding from './coding';
import * as spreading from './spreading';

// Re-export as namespaces
export { modulation, coding, spreading };

// Direct exports for convenience
export * from './modulation';
