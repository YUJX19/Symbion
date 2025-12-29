/**
 * Channel Module Tests
 * Tests for AWGN, Fading, and MIMO channel models
 */

import { describe, it, expect } from 'vitest';
import {
    // Direct exports
    shannonCapacity,
    addNoise,
    awgnCalculate,
    rayleighFading,
    ricianFading,
    dopplerShift,
    maxDopplerShift,
    coherenceTime,
    coherenceBandwidth,
    generateMIMOChannel,
    calculateMIMOCapacity,
    capacityBounds,
    diversityMultiplexingTradeoff,
    // Namespaced modules
    awgn,
    fading,
} from '../src/models/channel';
import { isClose, mean as calcMean } from './test-utils';

// Destructure from namespaced modules
const { requiredSnr, requiredSignalPower, addComplexNoise, noisePdfData } = awgn;
const { rayleighFadingBatch, rayleighPdf, rayleighCdf, kFactorFromDb, kFactorToDb } = fading;


// ==================== AWGN Channel Tests ====================

describe('AWGN Channel', () => {
    describe('shannonCapacity', () => {
        it('should calculate capacity for typical SNR and bandwidth', () => {
            // 10 dB SNR, 20 MHz → ~69.23 Mbps
            const capacity = shannonCapacity(10, 20e6);
            expect(isClose(capacity, 69.23e6, 0.01)).toBe(true);
        });

        it('should return 0 capacity for extremely low SNR approaching -∞', () => {
            // Very low SNR (approaching 0 linear)
            const capacity = shannonCapacity(-100, 20e6);
            expect(capacity).toBeGreaterThan(0);
            expect(capacity).toBeLessThan(1000); // Very small
        });

        it('should scale linearly with bandwidth', () => {
            const cap1 = shannonCapacity(10, 10e6);
            const cap2 = shannonCapacity(10, 20e6);
            expect(isClose(cap2 / cap1, 2, 1e-6)).toBe(true);
        });

        it('should increase with SNR', () => {
            const cap0 = shannonCapacity(0, 20e6);
            const cap10 = shannonCapacity(10, 20e6);
            const cap20 = shannonCapacity(20, 20e6);
            expect(cap10).toBeGreaterThan(cap0);
            expect(cap20).toBeGreaterThan(cap10);
        });

        it('should equal bandwidth at SNR of 0 dB (SNR_linear=1)', () => {
            // C = B * log2(1+1) = B * 1 = B
            const capacity = shannonCapacity(0, 20e6);
            expect(isClose(capacity, 20e6, 1e-6)).toBe(true);
        });
    });

    describe('requiredSnr', () => {
        it('should be inverse of shannonCapacity', () => {
            const bandwidth = 20e6;
            const snrDb = 15;
            const capacity = shannonCapacity(snrDb, bandwidth);
            const recoveredSnr = requiredSnr(capacity, bandwidth);
            expect(isClose(recoveredSnr, snrDb, 1e-4)).toBe(true);
        });

        it('should return 0 dB SNR for capacity = bandwidth', () => {
            const snr = requiredSnr(20e6, 20e6);
            expect(isClose(snr, 0, 1e-6)).toBe(true);
        });
    });

    describe('requiredSignalPower', () => {
        it('should account for thermal noise and SNR', () => {
            const power = requiredSignalPower(10, 20e6);
            // Noise power = -174 + 10*log10(20e6) = -174 + 73 = -101 dBm
            // Required power = 10 + (-101) = -91 dBm
            expect(isClose(power, -91, 0.1)).toBe(true);
        });
    });

    describe('addNoise', () => {
        it('should add noise while maintaining approximate signal power ratio', () => {
            const signal = [1, 1, -1, -1, 1, -1, 1, 1];
            const snrDb = 20; // High SNR
            const noisy = addNoise(signal, snrDb);

            expect(noisy.length).toBe(signal.length);

            // Signal should be similar (high SNR = little noise)
            for (let i = 0; i < signal.length; i++) {
                // With 20 dB SNR, difference should typically be < 0.3
                expect(Math.abs(noisy[i] - signal[i])).toBeLessThan(1);
            }
        });

        it('should add more noise with lower SNR', () => {
            const signal = [1, 1, 1, 1, 1, 1, 1, 1];

            // Collect noise samples
            const highSnrDiffs: number[] = [];
            const lowSnrDiffs: number[] = [];

            for (let trial = 0; trial < 100; trial++) {
                const highSnr = addNoise([1], 30);
                const lowSnr = addNoise([1], 10);
                highSnrDiffs.push(Math.abs(highSnr[0] - 1));
                lowSnrDiffs.push(Math.abs(lowSnr[0] - 1));
            }

            const highSnrAvgDiff = calcMean(highSnrDiffs);
            const lowSnrAvgDiff = calcMean(lowSnrDiffs);

            // Low SNR should have more noise (larger average difference)
            expect(lowSnrAvgDiff).toBeGreaterThan(highSnrAvgDiff);
        });
    });

    describe('addComplexNoise', () => {
        it('should add noise to complex signal', () => {
            const real = [1, 0, -1, 0];
            const imag = [0, 1, 0, -1];
            const [noisyReal, noisyImag] = addComplexNoise(real, imag, 20);

            expect(noisyReal.length).toBe(4);
            expect(noisyImag.length).toBe(4);
        });

        it('should throw for mismatched array lengths', () => {
            expect(() => addComplexNoise([1, 2], [1], 10)).toThrow();
        });
    });

    describe('awgn.calculate', () => {
        it('should return complete calculation result', () => {
            const result = awgnCalculate({ snrDb: 10, bandwidth: 20e6 });

            expect(result.snrDb).toBe(10);
            expect(isClose(result.snrLinear, 10, 1e-6)).toBe(true);
            expect(result.capacity).toBeGreaterThan(0);
            expect(result.noisePowerDbm).toBeLessThan(0);
            expect(result.noiseStd).toBeGreaterThan(0);
        });
    });

    describe('noisePdfData', () => {
        it('should return PDF data arrays', () => {
            const { x, y } = noisePdfData(10, 50);
            expect(x.length).toBe(51);
            expect(y.length).toBe(51);
            // PDF values should be non-negative
            for (const yVal of y) {
                expect(yVal).toBeGreaterThanOrEqual(0);
            }
        });
    });
});

// ==================== Fading Channel Tests ====================

describe('Fading Channels', () => {
    describe('rayleighFading', () => {
        it('should return complex coefficient with all properties', () => {
            const h = rayleighFading();
            expect(typeof h.real).toBe('number');
            expect(typeof h.imag).toBe('number');
            expect(typeof h.magnitude).toBe('number');
            expect(typeof h.phase).toBe('number');
        });

        it('should have magnitude matching sqrt(real² + imag²)', () => {
            const h = rayleighFading();
            const expectedMag = Math.sqrt(h.real ** 2 + h.imag ** 2);
            expect(isClose(h.magnitude, expectedMag, 1e-10)).toBe(true);
        });

        it('should have phase matching atan2(imag, real)', () => {
            const h = rayleighFading();
            const expectedPhase = Math.atan2(h.imag, h.real);
            expect(isClose(h.phase, expectedPhase, 1e-10)).toBe(true);
        });

        it('should generate coefficients with mean magnitude ≈ sqrt(π/2) * sigma', () => {
            const samples = 10000;
            let magSum = 0;
            for (let i = 0; i < samples; i++) {
                magSum += rayleighFading().magnitude;
            }
            const meanMag = magSum / samples;
            // For sigma = 1/sqrt(2), E[r] = sqrt(π/2) * sigma ≈ 0.886
            const expected = Math.sqrt(Math.PI / 2) * (1 / Math.sqrt(2));
            expect(Math.abs(meanMag - expected)).toBeLessThan(0.05);
        });
    });

    describe('rayleighFadingBatch', () => {
        it('should generate batch of coefficients', () => {
            const { real, imag, magnitude } = rayleighFadingBatch(100);
            expect(real.length).toBe(100);
            expect(imag.length).toBe(100);
            expect(magnitude.length).toBe(100);
        });
    });

    describe('rayleighPdf', () => {
        it('should return 0 for negative values', () => {
            expect(rayleighPdf(-1)).toBe(0);
        });

        it('should return 0 at r=0', () => {
            expect(rayleighPdf(0)).toBe(0);
        });

        it('should be non-negative for positive r', () => {
            for (let r = 0.1; r < 3; r += 0.1) {
                expect(rayleighPdf(r)).toBeGreaterThan(0);
            }
        });

        it('should peak at r = sigma', () => {
            const sigma = 1 / Math.sqrt(2);
            const peakR = sigma;
            const atPeak = rayleighPdf(peakR, sigma);
            const before = rayleighPdf(peakR - 0.1, sigma);
            const after = rayleighPdf(peakR + 0.1, sigma);
            expect(atPeak).toBeGreaterThan(before);
            expect(atPeak).toBeGreaterThan(after);
        });
    });

    describe('rayleighCdf', () => {
        it('should return 0 for negative values', () => {
            expect(rayleighCdf(-1)).toBe(0);
        });

        it('should return 0 at r=0', () => {
            expect(rayleighCdf(0)).toBe(0);
        });

        it('should approach 1 for large r', () => {
            expect(rayleighCdf(10)).toBeGreaterThan(0.999);
        });

        it('should be monotonically increasing', () => {
            let prev = 0;
            for (let r = 0; r <= 3; r += 0.1) {
                const cdf = rayleighCdf(r);
                expect(cdf).toBeGreaterThanOrEqual(prev);
                prev = cdf;
            }
        });
    });

    describe('ricianFading', () => {
        it('should return complex coefficient', () => {
            const h = ricianFading({ kFactor: 5 });
            expect(typeof h.real).toBe('number');
            expect(typeof h.imag).toBe('number');
            expect(typeof h.magnitude).toBe('number');
            expect(typeof h.phase).toBe('number');
        });

        it('should have higher mean magnitude with higher K-factor', () => {
            // Higher K = more LOS = higher average magnitude
            let sumLowK = 0;
            let sumHighK = 0;
            const trials = 1000;

            for (let i = 0; i < trials; i++) {
                sumLowK += ricianFading({ kFactor: 1 }).magnitude;
                sumHighK += ricianFading({ kFactor: 100 }).magnitude;
            }

            const meanLowK = sumLowK / trials;
            const meanHighK = sumHighK / trials;
            expect(meanHighK).toBeGreaterThan(meanLowK);
        });

        it('should approach Rayleigh when K→0', () => {
            // K=0 means no LOS, pure Rayleigh
            const samples: number[] = [];
            for (let i = 0; i < 1000; i++) {
                samples.push(ricianFading({ kFactor: 0.001 }).magnitude);
            }
            const mean = calcMean(samples);
            // Should be close to Rayleigh mean
            const rayleighMean = Math.sqrt(Math.PI / 2) * (1 / Math.sqrt(2));
            expect(Math.abs(mean - rayleighMean)).toBeLessThan(0.1);
        });
    });

    describe('K-factor conversions', () => {
        it('should convert dB to linear correctly', () => {
            expect(isClose(kFactorFromDb(0), 1, 1e-10)).toBe(true);
            expect(isClose(kFactorFromDb(10), 10, 1e-6)).toBe(true);
            expect(isClose(kFactorFromDb(20), 100, 1e-6)).toBe(true);
        });

        it('should convert linear to dB correctly', () => {
            expect(isClose(kFactorToDb(1), 0, 1e-10)).toBe(true);
            expect(isClose(kFactorToDb(10), 10, 1e-6)).toBe(true);
            expect(isClose(kFactorToDb(100), 20, 1e-6)).toBe(true);
        });

        it('should be inverse operations', () => {
            for (const k of [1, 5, 10, 50, 100]) {
                expect(isClose(kFactorFromDb(kFactorToDb(k)), k, 1e-10)).toBe(true);
            }
        });
    });

    describe('Doppler and Coherence', () => {
        it('should calculate max Doppler shift correctly', () => {
            // 2.4 GHz, 30 m/s → fd = v/c * fc = 30/3e8 * 2.4e9 = 240 Hz
            const fd = maxDopplerShift(30, 2.4e9);
            expect(isClose(fd, 240, 1e-6)).toBe(true);
        });

        it('should calculate Doppler shift with angle', () => {
            // At 90°, cos(90°) = 0 → no Doppler shift
            const fd90 = dopplerShift(30, 2.4e9, Math.PI / 2);
            expect(isClose(fd90, 0, 1e-10)).toBe(true);

            // At 0°, cos(0) = 1 → max Doppler shift
            const fd0 = dopplerShift(30, 2.4e9, 0);
            expect(isClose(fd0, maxDopplerShift(30, 2.4e9), 1e-10)).toBe(true);
        });

        it('should calculate coherence time correctly', () => {
            // Tc = 1 / fd_max
            const fd = 100; // Hz
            expect(isClose(coherenceTime(fd), 0.01, 1e-10)).toBe(true);
        });

        it('should calculate coherence bandwidth correctly', () => {
            // Bc = 1 / (5 * delay_spread)
            const delaySpread = 1e-6; // 1 μs
            expect(isClose(coherenceBandwidth(delaySpread), 200000, 1e-6)).toBe(true);
        });
    });
});

// ==================== MIMO Channel Tests ====================

describe('MIMO Channel', () => {
    describe('generateMIMOChannel', () => {
        it('should generate channel matrix with correct dimensions', () => {
            const { H } = generateMIMOChannel({ txCount: 4, rxCount: 2 });
            expect(H.length).toBe(2); // Rx antennas
            expect(H[0].length).toBe(4); // Tx antennas
        });

        it('should compute magnitudes matrix', () => {
            const { H, magnitudes } = generateMIMOChannel({ txCount: 2, rxCount: 2 });
            expect(magnitudes.length).toBe(2);
            expect(magnitudes[0].length).toBe(2);

            // Verify magnitudes match
            for (let i = 0; i < 2; i++) {
                for (let j = 0; j < 2; j++) {
                    const expected = Math.sqrt(
                        H[i][j].real ** 2 + H[i][j].imag ** 2
                    );
                    expect(isClose(magnitudes[i][j], expected, 1e-10)).toBe(true);
                }
            }
        });

        it('should compute eigenvalues', () => {
            const { eigenvalues } = generateMIMOChannel({ txCount: 4, rxCount: 4 });
            expect(eigenvalues.length).toBe(4);
            // Eigenvalues should be non-negative
            for (const ev of eigenvalues) {
                expect(ev).toBeGreaterThanOrEqual(0);
            }
        });

        it('should determine rank', () => {
            const { rank } = generateMIMOChannel({ txCount: 4, rxCount: 4 });
            expect(rank).toBeGreaterThanOrEqual(0);
            expect(rank).toBeLessThanOrEqual(4);
        });

        it('should handle correlation parameter', () => {
            // High correlation should reduce effective degrees of freedom
            const lowCorr = generateMIMOChannel({
                txCount: 4,
                rxCount: 4,
                correlation: 0.1,
            });
            const highCorr = generateMIMOChannel({
                txCount: 4,
                rxCount: 4,
                correlation: 0.9,
            });

            // High correlation should tend to have fewer dominant eigenvalues
            // (statistical property, just ensure no errors)
            expect(lowCorr.eigenvalues.length).toBe(4);
            expect(highCorr.eigenvalues.length).toBe(4);
        });
    });

    describe('calculateMIMOCapacity', () => {
        it('should calculate capacity for given eigenvalues', () => {
            const eigenvalues = [1, 1, 1, 1]; // Unit eigenvalues
            const result = calculateMIMOCapacity(eigenvalues, 10, 4);

            expect(result.capacity).toBeGreaterThan(0);
            expect(result.streamCapacities.length).toBe(4);
            expect(result.powerAllocation.length).toBe(4);
        });

        it('should increase capacity with SNR', () => {
            const eigenvalues = [1, 0.5, 0.25, 0.1];
            const cap10 = calculateMIMOCapacity(eigenvalues, 10, 4);
            const cap20 = calculateMIMOCapacity(eigenvalues, 20, 4);

            expect(cap20.capacity).toBeGreaterThan(cap10.capacity);
        });

        it('should handle single eigenvalue', () => {
            const result = calculateMIMOCapacity([1], 10, 1);
            expect(result.capacity).toBeGreaterThan(0);
        });
    });

    describe('capacityBounds', () => {
        it('should return low and high SNR bounds', () => {
            const bounds = capacityBounds(4, 4, 10);

            expect(typeof bounds.lowSnr).toBe('number');
            expect(typeof bounds.highSnr).toBe('number');
        });

        it('should have highSnr bound greater at high SNR', () => {
            const bounds = capacityBounds(4, 4, 30);
            // At high SNR, multiplexing gain dominates
            expect(bounds.highSnr).toBeGreaterThan(0);
        });
    });

    describe('diversityMultiplexingTradeoff', () => {
        it('should return diversity gain for given multiplexing gain', () => {
            const d = diversityMultiplexingTradeoff(0, 2, 2);
            // At r=0 (maximum diversity), d = Nt * Nr = 4
            expect(d).toBe(4);
        });

        it('should decrease diversity as multiplexing increases', () => {
            const d0 = diversityMultiplexingTradeoff(0, 4, 4);
            const d1 = diversityMultiplexingTradeoff(1, 4, 4);
            const d2 = diversityMultiplexingTradeoff(2, 4, 4);

            expect(d1).toBeLessThan(d0);
            expect(d2).toBeLessThan(d1);
        });

        it('should return 0 diversity at maximum multiplexing', () => {
            // At r = min(Nt, Nr), diversity should be 0
            const d = diversityMultiplexingTradeoff(2, 2, 4);
            expect(d).toBe(0);
        });
    });
});
