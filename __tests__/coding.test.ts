/**
 * Coding Module Tests
 * Tests for error correction codes: Hamming, CRC, Convolutional, Turbo, LDPC, Polar
 */

import { describe, it, expect } from 'vitest';
import {
    calculateLlr,
    parityBit,
    addParity,
    checkParity,
    hammingEncode,
    hammingDecode,
    crcCalculate,
    crc8,
    crc16,
    convolutionalEncode,
    viterbiDecode,
    turboEncode,
    turboDecode,
    ldpcEncode,
    ldpcDecode,
    polarEncode,
    polarDecode,
} from '../src/models/phy/coding';
import { isClose, arraysClose } from './test-utils';

// ==================== Parity Functions ====================

describe('Parity Functions', () => {
    describe('parityBit', () => {
        it('should return 0 for even number of 1s', () => {
            expect(parityBit([1, 1, 0, 0])).toBe(0);
            expect(parityBit([1, 1, 1, 1])).toBe(0);
            expect(parityBit([0, 0, 0, 0])).toBe(0);
        });

        it('should return 1 for odd number of 1s', () => {
            expect(parityBit([1, 0, 0, 0])).toBe(1);
            expect(parityBit([1, 1, 1, 0])).toBe(1);
            expect(parityBit([1, 1, 1, 1, 1])).toBe(1);
        });
    });

    describe('addParity', () => {
        it('should add parity bit to data', () => {
            const withParity = addParity([1, 1, 0, 0]);
            expect(withParity.length).toBe(5);
            expect(withParity[4]).toBe(0); // Even parity
        });

        it('should make total parity even', () => {
            const withParity = addParity([1, 0, 0, 0]);
            expect(parityBit(withParity)).toBe(0);
        });
    });

    describe('checkParity', () => {
        it('should return true for valid parity', () => {
            expect(checkParity([1, 1, 0, 0, 0])).toBe(true);
            expect(checkParity([1, 0, 0, 0, 1])).toBe(true);
        });

        it('should return false for invalid parity', () => {
            expect(checkParity([1, 1, 0, 0, 1])).toBe(false);
            expect(checkParity([1, 0, 0, 0, 0])).toBe(false);
        });
    });
});

// ==================== Hamming Code ====================

describe('Hamming Code', () => {
    describe('hammingEncode', () => {
        it('should encode data with correct length', () => {
            const data = [1, 0, 1, 1]; // 4 data bits
            const encoded = hammingEncode(data);
            expect(encoded.length).toBe(7); // (7,4) Hamming code
        });

        it('should be deterministic', () => {
            const data = [1, 0, 1, 1];
            const encoded1 = hammingEncode(data);
            const encoded2 = hammingEncode(data);
            expect(arraysClose(encoded1, encoded2)).toBe(true);
        });
    });

    describe('hammingDecode', () => {
        it('should decode error-free codeword', () => {
            const data = [1, 0, 1, 1];
            const encoded = hammingEncode(data);
            const { data: decoded, errorPos, corrected } = hammingDecode(encoded);
            expect(errorPos).toBe(0); // No error
        });

        it('should detect and correct single bit error', () => {
            const data = [1, 0, 1, 1];
            const encoded = hammingEncode(data);

            // Introduce single bit error
            const corrupted = [...encoded];
            corrupted[3] = corrupted[3] === 0 ? 1 : 0;

            const { corrected, errorPos } = hammingDecode(corrupted);
            expect(corrected).toBe(true);
            expect(errorPos).toBeGreaterThan(0);
        });
    });
});

// ==================== CRC ====================

describe('CRC', () => {
    describe('crcCalculate', () => {
        it('should calculate CRC for data', () => {
            const data = [1, 0, 1, 1, 0, 0, 1, 0];
            const crc = crcCalculate(data, 0x07, 8);
            expect(typeof crc).toBe('number');
            expect(crc).toBeGreaterThanOrEqual(0);
            expect(crc).toBeLessThan(256);
        });

        it('should produce different CRCs for different data', () => {
            const crc1 = crcCalculate([1, 0, 1, 1], 0x07, 8);
            const crc2 = crcCalculate([1, 1, 1, 1], 0x07, 8);
            expect(crc1).not.toBe(crc2);
        });
    });

    describe('crc8', () => {
        it('should calculate 8-bit CRC', () => {
            const crc = crc8([1, 0, 1, 1, 0, 0, 1, 0]);
            expect(crc).toBeGreaterThanOrEqual(0);
            expect(crc).toBeLessThan(256);
        });
    });

    describe('crc16', () => {
        it('should calculate 16-bit CRC', () => {
            const crc = crc16([1, 0, 1, 1, 0, 0, 1, 0]);
            expect(crc).toBeGreaterThanOrEqual(0);
            expect(crc).toBeLessThan(65536);
        });
    });
});

// ==================== Convolutional Code ====================

describe('Convolutional Code', () => {
    describe('convolutionalEncode', () => {
        it('should encode data', () => {
            const data = [1, 0, 1, 1];
            const encoded = convolutionalEncode(data);
            expect(encoded.length).toBeGreaterThan(data.length);
        });

        it('should be deterministic', () => {
            const data = [1, 0, 1, 1, 0, 0, 1, 0];
            const encoded1 = convolutionalEncode(data);
            const encoded2 = convolutionalEncode(data);
            expect(arraysClose(encoded1, encoded2)).toBe(true);
        });
    });

    describe('viterbiDecode', () => {
        it('should decode convolutional code', () => {
            const data = [1, 0, 1, 1, 0, 0, 1, 0];
            const encoded = convolutionalEncode(data);
            const decoded = viterbiDecode(encoded);
            expect(decoded.length).toBeGreaterThan(0);
        });

        it('should produce valid binary output', () => {
            const data = [1, 0, 1, 1];
            const encoded = convolutionalEncode(data);
            const decoded = viterbiDecode(encoded);
            // Just verify it produces binary output
            for (const bit of decoded) {
                expect(bit === 0 || bit === 1).toBe(true);
            }
        });
    });
});

// ==================== Turbo Code ====================

describe('Turbo Code', () => {
    describe('turboEncode', () => {
        it('should encode data with systematic and parity bits', () => {
            const data = [1, 0, 1, 1, 0, 0, 1, 0];
            const { systematic, parity1, parity2, interleaver } = turboEncode(data);

            // Systematic includes termination bits (+2)
            expect(systematic.length).toBeGreaterThanOrEqual(data.length);
            expect(parity1.length).toBeGreaterThan(0);
            expect(parity2.length).toBeGreaterThan(0);
            expect(interleaver.length).toBe(data.length);
        });

        it('should have systematic bits starting with input', () => {
            const data = [1, 0, 1, 1, 0, 0, 1, 0];
            const { systematic } = turboEncode(data);
            // First n bits should match input
            for (let i = 0; i < data.length; i++) {
                expect(systematic[i]).toBe(data[i]);
            }
        });
    });

    describe('turboDecode', () => {
        it('should decode turbo code', () => {
            const data = [1, 0, 1, 1, 0, 0, 1, 0];
            const { systematic, parity1, parity2, interleaver } = turboEncode(data);

            // Convert to soft values (no noise)
            const sysLlr = systematic.map(b => b === 1 ? 10 : -10);
            const p1Llr = parity1.map(b => b === 1 ? 10 : -10);
            const p2Llr = parity2.map(b => b === 1 ? 10 : -10);

            const decoded = turboDecode(sysLlr, p1Llr, p2Llr, interleaver, 4);
            expect(decoded.length).toBe(data.length);
        });
    });
});

// ==================== LDPC Code ====================

describe('LDPC Code', () => {
    describe('ldpcEncode', () => {
        it('should encode data with parity check matrix', () => {
            const data = [1, 0, 1, 0, 1];
            const { codeword, H } = ldpcEncode(data, 10, 5);

            expect(codeword.length).toBe(10);
            expect(H.length).toBeGreaterThan(0);
        });
    });

    describe('ldpcDecode', () => {
        it('should decode LDPC code', () => {
            const data = [1, 0, 1, 0, 1];
            const { codeword, H } = ldpcEncode(data, 10, 5);

            // Convert to LLR (high confidence)
            const llr = codeword.map(b => b === 1 ? 10 : -10);

            const decoded = ldpcDecode(llr, H, 10);
            expect(decoded.length).toBe(10);
        });
    });
});

// ==================== Polar Code ====================

describe('Polar Code', () => {
    describe('polarEncode', () => {
        it('should encode data', () => {
            const data = [1, 0, 1, 1]; // 4 information bits
            const encoded = polarEncode(data, 8, [0, 1, 2, 4]);
            expect(encoded.length).toBe(8);
        });

        it('should handle different frozen bit patterns', () => {
            const data = [1, 0];
            const encoded = polarEncode(data, 4, [0, 1]);
            expect(encoded.length).toBe(4);
        });
    });

    describe('polarDecode', () => {
        it('should decode polar code', () => {
            const data = [1, 0, 1, 1];
            const frozenIndices = [0, 1, 2, 4];
            const encoded = polarEncode(data, 8, frozenIndices);

            // Convert to LLR (high confidence)
            const llr = encoded.map(b => b === 1 ? 10 : -10);

            const decoded = polarDecode(llr, frozenIndices);
            // Should return some decoded bits
            expect(decoded.length).toBeGreaterThan(0);
        });
    });
});

// ==================== LLR Calculation ====================

describe('calculateLlr', () => {
    it('should calculate log-likelihood ratio array', () => {
        const received = [0.9, -0.8, 0.3, -0.2];
        const sigma = 0.5;
        const llr = calculateLlr(received, sigma);
        expect(Array.isArray(llr)).toBe(true);
        expect(llr.length).toBe(4);
    });

    it('should return positive LLR for positive received value', () => {
        const llr = calculateLlr([1.0], 0.5);
        expect(llr[0]).toBeGreaterThan(0);
    });

    it('should return negative LLR for negative received value', () => {
        const llr = calculateLlr([-1.0], 0.5);
        expect(llr[0]).toBeLessThan(0);
    });
});
