/**
 * Spreading Module Tests
 * Tests for spreading sequences: m-sequence, Gold, Walsh, DSSS, FHSS
 */

import { describe, it, expect } from 'vitest';
import {
    mSequence,
    pnSequence,
    goldSequence,
    hadamardMatrix,
    walshCode,
    walshCorrelation,
    dsssSpread,
    dsssDesread,
    processingGain,
    frequencyHoppingPattern,
    fhssBandwidth,
    fhssJammingMargin,
    kasamiSequence,
} from '../src/models/phy/spreading';
import { isClose, arraysClose } from './test-utils';

// ==================== M-Sequence ====================

describe('mSequence', () => {
    it('should generate sequence of correct length', () => {
        const seq = mSequence([3, 1], 7);
        expect(seq.length).toBe(7);
    });

    it('should generate binary sequence', () => {
        const seq = mSequence([3, 1], 15);
        for (const bit of seq) {
            expect(bit === 0 || bit === 1 || bit === -1).toBe(true);
        }
    });

    it('should be deterministic with same taps', () => {
        const seq1 = mSequence([3, 1], 7);
        const seq2 = mSequence([3, 1], 7);
        expect(arraysClose(seq1, seq2)).toBe(true);
    });
});

// ==================== PN Sequence ====================

describe('pnSequence', () => {
    it('should generate PN sequence', () => {
        const seq = pnSequence(3);
        expect(seq.length).toBe(7); // 2^3 - 1
    });

    it('should have maximal length', () => {
        const seq = pnSequence(4);
        expect(seq.length).toBe(15); // 2^4 - 1
    });
});

// ==================== Gold Sequence ====================

describe('goldSequence', () => {
    it('should generate Gold sequence', () => {
        const seq = goldSequence(5, 0);
        expect(seq.length).toBe(31); // 2^5 - 1
    });

    it('should generate different sequences for different shifts', () => {
        const seq1 = goldSequence(5, 0);
        const seq2 = goldSequence(5, 5);
        // Sequences should differ
        let differences = 0;
        for (let i = 0; i < seq1.length; i++) {
            if (seq1[i] !== seq2[i]) differences++;
        }
        expect(differences).toBeGreaterThan(0);
    });
});

// ==================== Hadamard Matrix ====================

describe('hadamardMatrix', () => {
    it('should generate 2x2 Hadamard matrix', () => {
        const H = hadamardMatrix(2);
        expect(H.length).toBe(2);
        expect(H[0].length).toBe(2);
    });

    it('should generate orthogonal rows', () => {
        const H = hadamardMatrix(4);
        // Check orthogonality: H * H^T = n * I
        for (let i = 0; i < 4; i++) {
            for (let j = 0; j < 4; j++) {
                let dot = 0;
                for (let k = 0; k < 4; k++) {
                    dot += H[i][k] * H[j][k];
                }
                if (i === j) {
                    expect(dot).toBe(4);
                } else {
                    expect(dot).toBe(0);
                }
            }
        }
    });

    it('should contain only +1 and -1', () => {
        const H = hadamardMatrix(8);
        for (let i = 0; i < 8; i++) {
            for (let j = 0; j < 8; j++) {
                expect(H[i][j] === 1 || H[i][j] === -1).toBe(true);
            }
        }
    });
});

// ==================== Walsh Code ====================

describe('walshCode', () => {
    it('should generate Walsh code of correct length', () => {
        const code = walshCode(3, 8);
        expect(code.length).toBe(8);
    });

    it('should contain only +1 and -1', () => {
        const code = walshCode(5, 16);
        for (const chip of code) {
            expect(chip === 1 || chip === -1).toBe(true);
        }
    });

    it('should generate orthogonal codes', () => {
        const code0 = walshCode(0, 8);
        const code1 = walshCode(1, 8);
        const correlation = walshCorrelation(code0, code1);
        expect(correlation).toBe(0);
    });
});

describe('walshCorrelation', () => {
    it('should return normalized autocorrelation', () => {
        const code = walshCode(3, 8);
        const corr = walshCorrelation(code, code);
        // Implementation normalizes by length, so autocorrelation = 1
        expect(isClose(corr, 1)).toBe(true);
    });

    it('should return 0 for different orthogonal codes', () => {
        const code1 = walshCode(2, 8);
        const code2 = walshCode(5, 8);
        const corr = walshCorrelation(code1, code2);
        expect(corr).toBe(0);
    });
});

// ==================== DSSS ====================

describe('DSSS Spreading', () => {
    describe('dsssSpread', () => {
        it('should spread data by spreading factor', () => {
            const data = [1, -1, 1];
            const code = [1, 1, -1, 1];
            const spread = dsssSpread(data, code);
            expect(spread.length).toBe(data.length * code.length);
        });

        it('should multiply data by spreading code', () => {
            const data = [1];
            const code = [1, -1, 1, -1];
            const spread = dsssSpread(data, code);
            expect(arraysClose(spread, [1, -1, 1, -1])).toBe(true);
        });
    });

    describe('dsssDesread', () => {
        it('should recover original data', () => {
            const data = [1, -1, 1, -1];
            const code = [1, 1, -1, 1];
            const spread = dsssSpread(data, code);
            const despread = dsssDesread(spread, code);

            expect(despread.length).toBe(data.length);
            for (let i = 0; i < data.length; i++) {
                // Sign should match
                expect(Math.sign(despread[i])).toBe(data[i]);
            }
        });
    });

    describe('processingGain', () => {
        it('should calculate processing gain correctly', () => {
            const chipRate = 10e6; // 10 Mcps
            const symbolRate = 1e6; // 1 Msps
            const gain = processingGain(chipRate, symbolRate);
            expect(isClose(gain, 10)).toBe(true);
        });
    });
});

// ==================== FHSS ====================

describe('FHSS', () => {
    describe('frequencyHoppingPattern', () => {
        it('should generate hopping pattern', () => {
            // Signature is (numChannels, hopCount, seed)
            const pattern = frequencyHoppingPattern(100, 10, 1);
            expect(pattern.length).toBe(10);
        });

        it('should produce valid channel indices', () => {
            const numChannels = 50;
            const pattern = frequencyHoppingPattern(numChannels, 20, 1);

            for (const channel of pattern) {
                expect(channel).toBeGreaterThanOrEqual(0);
                expect(channel).toBeLessThan(numChannels);
            }
        });
    });

    describe('fhssBandwidth', () => {
        it('should calculate total bandwidth', () => {
            const hopBw = 1e6; // 1 MHz per hop
            const numChannels = 79; // Bluetooth-like
            const totalBw = fhssBandwidth(hopBw, numChannels);
            expect(totalBw).toBe(79e6);
        });
    });

    describe('fhssJammingMargin', () => {
        it('should calculate jamming margin', () => {
            const totalBw = 79e6;
            const signalBw = 1e6;
            const margin = fhssJammingMargin(totalBw, signalBw);
            // Returns linear ratio
            expect(margin).toBeGreaterThan(1);
        });
    });
});

// ==================== Kasami Sequence ====================

describe('kasamiSequence', () => {
    it('should generate Kasami sequence', () => {
        const seq = kasamiSequence(6, 0);
        expect(seq.length).toBe(63); // 2^6 - 1
    });

    it('should generate different sequences for different shifts', () => {
        const seq1 = kasamiSequence(6, 0);
        const seq2 = kasamiSequence(6, 3);
        let differences = 0;
        for (let i = 0; i < seq1.length; i++) {
            if (seq1[i] !== seq2[i]) differences++;
        }
        expect(differences).toBeGreaterThan(0);
    });
});
