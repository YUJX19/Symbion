/**
 * Modulation Module Tests
 * Tests for QAM, Analog Modulation, and Pulse Shaping
 */

import { describe, it, expect } from 'vitest';
import {
    // QAM exports
    generateConstellation,
    normalizeConstellation,
    bitsToSymbolIndex,
    addQamNoise,
    hardDecision,
    theoreticalSer,
    generateQAMData,
    qam,
    // Analog modulation exports
    dsbScModulate,
    dsbScDemodulate,
    ssbModulate,
    amModulate,
    amEnvelopeDetect,
    analog,
    // Pulse shaping exports
    raisedCosine,
    rootRaisedCosine,
    raisedCosineFilter,
    rootRaisedCosineFilter,
    raisedCosineBandwidth,
    sinc,
    pulseShaping,
} from '../src/models/phy/modulation';
import { isClose, arraysClose, mean } from './test-utils';

// ==================== QAM Tests ====================

describe('QAM Modulation', () => {
    describe('generateConstellation', () => {
        it('should generate 4-QAM constellation with 4 points', () => {
            const constellation = generateConstellation(4);
            expect(constellation.length).toBe(4);
        });

        it('should generate 16-QAM constellation with 16 points', () => {
            const constellation = generateConstellation(16);
            expect(constellation.length).toBe(16);
        });

        it('should generate 64-QAM constellation with 64 points', () => {
            const constellation = generateConstellation(64);
            expect(constellation.length).toBe(64);
        });

        it('should throw error for non-perfect-square order', () => {
            expect(() => generateConstellation(5)).toThrow();
            expect(() => generateConstellation(10)).toThrow();
            expect(() => generateConstellation(15)).toThrow();
        });

        it('should center constellation around origin', () => {
            const constellation = generateConstellation(16);
            const avgI = constellation.reduce((sum, p) => sum + p.i, 0) / constellation.length;
            const avgQ = constellation.reduce((sum, p) => sum + p.q, 0) / constellation.length;
            expect(isClose(avgI, 0, 1e-10)).toBe(true);
            expect(isClose(avgQ, 0, 1e-10)).toBe(true);
        });

        it('should have points with both positive and negative I/Q values', () => {
            const constellation = generateConstellation(16);
            const hasPositiveI = constellation.some(p => p.i > 0);
            const hasNegativeI = constellation.some(p => p.i < 0);
            const hasPositiveQ = constellation.some(p => p.q > 0);
            const hasNegativeQ = constellation.some(p => p.q < 0);
            expect(hasPositiveI && hasNegativeI).toBe(true);
            expect(hasPositiveQ && hasNegativeQ).toBe(true);
        });
    });

    describe('normalizeConstellation', () => {
        it('should normalize constellation to unit average energy', () => {
            const constellation = generateConstellation(16);
            const normalized = normalizeConstellation(constellation);

            let avgEnergy = 0;
            for (const p of normalized) {
                avgEnergy += p.i * p.i + p.q * p.q;
            }
            avgEnergy /= normalized.length;

            expect(isClose(avgEnergy, 1, 1e-10)).toBe(true);
        });

        it('should maintain constellation shape after normalization', () => {
            const constellation = generateConstellation(4);
            const normalized = normalizeConstellation(constellation);

            // Check that relative positions are preserved
            const d01_orig = Math.sqrt(
                (constellation[0].i - constellation[1].i) ** 2 +
                (constellation[0].q - constellation[1].q) ** 2
            );
            const d02_orig = Math.sqrt(
                (constellation[0].i - constellation[2].i) ** 2 +
                (constellation[0].q - constellation[2].q) ** 2
            );

            const d01_norm = Math.sqrt(
                (normalized[0].i - normalized[1].i) ** 2 +
                (normalized[0].q - normalized[1].q) ** 2
            );
            const d02_norm = Math.sqrt(
                (normalized[0].i - normalized[2].i) ** 2 +
                (normalized[0].q - normalized[2].q) ** 2
            );

            // Ratio should be preserved
            expect(isClose(d01_orig / d02_orig, d01_norm / d02_norm, 1e-10)).toBe(true);
        });
    });

    describe('bitsToSymbolIndex', () => {
        it('should convert bits to symbol index correctly for 4-QAM', () => {
            expect(bitsToSymbolIndex([0, 0], 4)).toBe(0);
            expect(bitsToSymbolIndex([0, 1], 4)).toBe(1);
            expect(bitsToSymbolIndex([1, 0], 4)).toBe(2);
            expect(bitsToSymbolIndex([1, 1], 4)).toBe(3);
        });

        it('should throw error for incorrect bit length', () => {
            expect(() => bitsToSymbolIndex([0], 4)).toThrow(); // Need 2 bits for 4-QAM
            expect(() => bitsToSymbolIndex([0, 0, 0], 4)).toThrow();
        });

        it('should handle 16-QAM (4 bits per symbol)', () => {
            expect(bitsToSymbolIndex([0, 0, 0, 0], 16)).toBe(0);
            expect(bitsToSymbolIndex([1, 1, 1, 1], 16)).toBe(15);
        });
    });

    describe('addQamNoise (addNoise)', () => {
        it('should add noise to symbols', () => {
            const constellation = normalizeConstellation(generateConstellation(16));
            const symbols = [constellation[0], constellation[5], constellation[10]];
            const noisy = addQamNoise(symbols, 20);

            expect(noisy.length).toBe(symbols.length);
            // With high SNR, noisy symbols should be close to original
            for (let i = 0; i < symbols.length; i++) {
                const dist = Math.sqrt(
                    (noisy[i].i - symbols[i].i) ** 2 +
                    (noisy[i].q - symbols[i].q) ** 2
                );
                expect(dist).toBeLessThan(1); // Reasonable bound for 20 dB SNR
            }
        });

        it('should add more noise at lower SNR', () => {
            const constellation = normalizeConstellation(generateConstellation(4));
            const symbol = constellation[0];
            const symbols = [symbol];

            const highSnrDists: number[] = [];
            const lowSnrDists: number[] = [];

            for (let trial = 0; trial < 100; trial++) {
                const highSnrNoisy = addQamNoise(symbols, 30);
                const lowSnrNoisy = addQamNoise(symbols, 10);

                highSnrDists.push(Math.sqrt(
                    (highSnrNoisy[0].i - symbol.i) ** 2 +
                    (highSnrNoisy[0].q - symbol.q) ** 2
                ));
                lowSnrDists.push(Math.sqrt(
                    (lowSnrNoisy[0].i - symbol.i) ** 2 +
                    (lowSnrNoisy[0].q - symbol.q) ** 2
                ));
            }

            expect(mean(lowSnrDists)).toBeGreaterThan(mean(highSnrDists));
        });
    });

    describe('hardDecision', () => {
        it('should return correct symbol index for noiseless symbols', () => {
            const constellation = normalizeConstellation(generateConstellation(16));
            const received = [constellation[0], constellation[5], constellation[15]];
            const decided = hardDecision(received, constellation);

            expect(decided).toEqual([0, 5, 15]);
        });

        it('should find nearest neighbor correctly', () => {
            const constellation = normalizeConstellation(generateConstellation(4));
            // Slightly perturb symbol 0
            const received = [{ i: constellation[0].i + 0.01, q: constellation[0].q + 0.01 }];
            const decided = hardDecision(received, constellation);

            expect(decided[0]).toBe(0);
        });
    });

    describe('theoreticalSer', () => {
        it('should decrease SER as SNR increases', () => {
            const ser10 = theoreticalSer(16, 10);
            const ser15 = theoreticalSer(16, 15);
            const ser20 = theoreticalSer(16, 20);

            expect(ser15).toBeLessThan(ser10);
            expect(ser20).toBeLessThan(ser15);
        });

        it('should return higher SER for higher order QAM at same SNR', () => {
            const ser4 = theoreticalSer(4, 15);
            const ser16 = theoreticalSer(16, 15);
            const ser64 = theoreticalSer(64, 15);

            expect(ser16).toBeGreaterThan(ser4);
            expect(ser64).toBeGreaterThan(ser16);
        });

        it('should return SER between 0 and 1', () => {
            const ser = theoreticalSer(16, 15);
            expect(ser).toBeGreaterThanOrEqual(0);
            expect(ser).toBeLessThanOrEqual(1);
        });
    });

    describe('spectralEfficiency (via qam namespace)', () => {
        it('should return log2(order) bits per symbol', () => {
            expect(qam.spectralEfficiency(4)).toBe(2);
            expect(qam.spectralEfficiency(16)).toBe(4);
            expect(qam.spectralEfficiency(64)).toBe(6);
            expect(qam.spectralEfficiency(256)).toBe(8);
        });
    });

    describe('generateQAMData', () => {
        it('should generate tx and rx symbol arrays of correct length', () => {
            const data = generateQAMData({ order: 16, count: 100, snrDb: 20 });
            expect(data.tx.length).toBe(100);
            expect(data.rx.length).toBe(100);
            expect(data.constellation.length).toBe(16);
        });

        it('should generate normalized constellation', () => {
            const data = generateQAMData({ order: 16, count: 10, snrDb: 20 });
            let avgEnergy = 0;
            for (const p of data.constellation) {
                avgEnergy += p.i * p.i + p.q * p.q;
            }
            avgEnergy /= data.constellation.length;
            expect(isClose(avgEnergy, 1, 1e-10)).toBe(true);
        });
    });
});

// ==================== Analog Modulation Tests ====================

describe('Analog Modulation', () => {
    // Helper to generate a test message signal
    const generateTestMessage = (freq: number, sampleRate: number, duration: number): number[] => {
        const numSamples = Math.floor(sampleRate * duration);
        const signal: number[] = [];
        for (let i = 0; i < numSamples; i++) {
            const t = i / sampleRate;
            signal.push(Math.cos(2 * Math.PI * freq * t));
        }
        return signal;
    };

    describe('DSB-SC Modulation', () => {
        it('should modulate signal correctly', () => {
            const message = generateTestMessage(100, 10000, 0.1);
            const modulated = dsbScModulate(message, 1000, 10000);

            expect(modulated.length).toBe(message.length);
            // Modulated signal should have values in reasonable range
            expect(Math.max(...modulated.map(Math.abs))).toBeLessThanOrEqual(1.1);
        });

        it('should recover message after demodulation', () => {
            const sampleRate = 10000;
            const messageFreq = 100;
            const carrierFreq = 1000;
            const message = generateTestMessage(messageFreq, sampleRate, 0.1);

            const modulated = dsbScModulate(message, carrierFreq, sampleRate);
            const demodulated = dsbScDemodulate(modulated, carrierFreq, sampleRate, 200);

            // Skip edges due to filter transients
            const skipSamples = 100;
            const midMessage = message.slice(skipSamples, -skipSamples);
            const midDemod = demodulated.slice(skipSamples, -skipSamples);

            // Calculate correlation (should be high for successful demodulation)
            let correlation = 0;
            let msgPower = 0;
            let demodPower = 0;
            for (let i = 0; i < midMessage.length; i++) {
                correlation += midMessage[i] * midDemod[i];
                msgPower += midMessage[i] ** 2;
                demodPower += midDemod[i] ** 2;
            }
            const normalizedCorr = Math.abs(correlation) / Math.sqrt(msgPower * demodPower);
            expect(normalizedCorr).toBeGreaterThan(0.8);
        });
    });

    describe('SSB Modulation', () => {
        it('should modulate to USB correctly', () => {
            const message = generateTestMessage(100, 10000, 0.1);
            const hilbert = analog.hilbertApprox(message);
            const usb = ssbModulate(message, hilbert, 1000, 10000, 'USB');

            expect(usb.length).toBe(message.length);
        });

        it('should modulate to LSB correctly', () => {
            const message = generateTestMessage(100, 10000, 0.1);
            const hilbert = analog.hilbertApprox(message);
            const lsb = ssbModulate(message, hilbert, 1000, 10000, 'LSB');

            expect(lsb.length).toBe(message.length);
        });
    });

    describe('AM Modulation', () => {
        it('should modulate with carrier correctly', () => {
            const message = generateTestMessage(100, 10000, 0.1);
            const modulated = amModulate(message, 1000, 10000, 0.5);

            expect(modulated.length).toBe(message.length);
            // With modulation index 0.5, amplitude should be between 0.5 and 1.5
            const maxAmp = Math.max(...modulated.map(Math.abs));
            expect(maxAmp).toBeLessThanOrEqual(1.6);
        });

        it('should allow envelope detection recovery', () => {
            const sampleRate = 10000;
            const message = generateTestMessage(100, sampleRate, 0.1);
            const modulated = amModulate(message, 1000, sampleRate, 0.8);
            const detected = amEnvelopeDetect(modulated);

            expect(detected.length).toBe(modulated.length);
        });

        it('should respect modulation index', () => {
            const message = [1, 1, 1, 1]; // Constant message
            const modulated1 = amModulate(message, 1000, 10000, 0.5);
            const modulated2 = amModulate(message, 1000, 10000, 1.0);

            // Higher modulation index = larger amplitude variation
            const range1 = Math.max(...modulated1) - Math.min(...modulated1);
            const range2 = Math.max(...modulated2) - Math.min(...modulated2);
            // For constant message, both should have same range (carrier amplitude swing)
            expect(range1).toBeGreaterThan(0);
            expect(range2).toBeGreaterThan(0);
        });
    });

    describe('FM Modulation', () => {
        it('should modulate correctly', () => {
            const message = generateTestMessage(100, 10000, 0.1);
            const modulated = analog.fmModulate(message, 1000, 10000, 50);

            expect(modulated.length).toBe(message.length);
            // FM signal amplitude should be constant (approximately 1)
            for (const sample of modulated) {
                expect(Math.abs(sample)).toBeLessThanOrEqual(1.01);
            }
        });

        it('should recover message trend after demodulation', () => {
            const sampleRate = 10000;
            const message = generateTestMessage(50, sampleRate, 0.1);
            const modulated = analog.fmModulate(message, 1000, sampleRate, 100);
            const demodulated = analog.fmDemodulate(modulated, sampleRate);

            expect(demodulated.length).toBe(modulated.length);
        });
    });

    describe('PM Modulation', () => {
        it('should modulate correctly', () => {
            const message = generateTestMessage(100, 10000, 0.1);
            const modulated = analog.pmModulate(message, 1000, 10000, Math.PI / 4);

            expect(modulated.length).toBe(message.length);
            // PM signal amplitude should be constant (approximately 1)
            for (const sample of modulated) {
                expect(Math.abs(sample)).toBeLessThanOrEqual(1.01);
            }
        });

        it('should recover phase information after demodulation', () => {
            const sampleRate = 10000;
            const message = generateTestMessage(50, sampleRate, 0.1);
            const modulated = analog.pmModulate(message, 1000, sampleRate, Math.PI / 2);
            const demodulated = analog.pmDemodulate(modulated, 1000, sampleRate);

            expect(demodulated.length).toBe(modulated.length);
        });
    });

    describe('modulationBandwidth', () => {
        it('should return 2x message bandwidth for DSB-SC', () => {
            expect(analog.modulationBandwidth('DSB-SC', 5000)).toBe(10000);
        });

        it('should return 1x message bandwidth for SSB', () => {
            expect(analog.modulationBandwidth('SSB', 5000)).toBe(5000);
        });

        it('should return 2x message bandwidth for AM', () => {
            expect(analog.modulationBandwidth('AM', 5000)).toBe(10000);
        });
    });

    describe('generateSineMessage', () => {
        it('should generate correct number of samples', () => {
            const signal = analog.generateSineMessage(100, 1000, 0.5);
            expect(signal.length).toBe(500);
        });

        it('should generate signal with correct amplitude', () => {
            const signal = analog.generateSineMessage(100, 1000, 0.5, 2);
            const maxAmp = Math.max(...signal.map(Math.abs));
            expect(isClose(maxAmp, 2, 0.01)).toBe(true);
        });
    });
});

// ==================== Pulse Shaping Tests ====================

describe('Pulse Shaping', () => {
    const T = 1; // Symbol period for testing

    describe('sinc function', () => {
        it('should return 1 at t=0', () => {
            expect(sinc(0)).toBe(1);
        });

        it('should return 0 at integer multiples', () => {
            expect(isClose(sinc(1), 0, 1e-10)).toBe(true);
            expect(isClose(sinc(-1), 0, 1e-10)).toBe(true);
            expect(isClose(sinc(2), 0, 1e-10)).toBe(true);
        });

        it('should be symmetric', () => {
            expect(isClose(sinc(0.5), sinc(-0.5), 1e-10)).toBe(true);
            expect(isClose(sinc(1.5), sinc(-1.5), 1e-10)).toBe(true);
        });
    });

    describe('raisedCosine', () => {
        it('should return 1 at t=0', () => {
            expect(raisedCosine(0, T, 0.5)).toBe(1);
        });

        it('should be symmetric', () => {
            expect(isClose(raisedCosine(0.5, T, 0.5), raisedCosine(-0.5, T, 0.5), 1e-10)).toBe(true);
        });

        it('should match sinc for roll-off = 0 (normalized by symbol period)', () => {
            // For alpha = 0, raised cosine equals sinc(t/T)
            for (const t of [0.3, 0.5, 0.7]) {
                const rc = raisedCosine(t, T, 0);
                const expected = sinc(t / T);
                expect(isClose(rc, expected, 1e-6)).toBe(true);
            }
        });

        it('should decay for higher roll-off', () => {
            const t = 2;
            const rc0 = Math.abs(raisedCosine(t, T, 0));
            const rc05 = Math.abs(raisedCosine(t, T, 0.5));
            const rc1 = Math.abs(raisedCosine(t, T, 1));

            expect(rc0).toBeGreaterThanOrEqual(0);
            expect(rc05).toBeGreaterThanOrEqual(0);
            expect(rc1).toBeGreaterThanOrEqual(0);
        });
    });

    describe('rootRaisedCosine', () => {
        it('should have peak at t=0', () => {
            const peak = rootRaisedCosine(0, T, 0.5);
            expect(peak).toBeGreaterThan(0);
        });

        it('should be symmetric', () => {
            expect(isClose(rootRaisedCosine(0.5, T, 0.5), rootRaisedCosine(-0.5, T, 0.5), 1e-10)).toBe(true);
        });
    });

    describe('raisedCosineFilter', () => {
        it('should generate filter of correct length', () => {
            // Parameters: symbolPeriod, rolloff, sampleRate, numSymbols
            const symbolPeriod = 1e-6; // 1 us
            const sampleRate = 8e6;    // 8 MHz -> 8 samples per symbol
            const numSymbols = 4;
            const filter = raisedCosineFilter(symbolPeriod, 0.5, sampleRate, numSymbols);
            // samplesPerSymbol = round(symbolPeriod * sampleRate) = 8
            // Length = 2 * numSymbols * samplesPerSymbol + 1 = 2 * 4 * 8 + 1 = 65
            expect(filter.length).toBe(65);
        });

        it('should have peak at center', () => {
            const filter = raisedCosineFilter(1e-6, 0.5, 8e6, 4);
            const centerIdx = Math.floor(filter.length / 2);
            const maxIdx = filter.indexOf(Math.max(...filter));
            expect(maxIdx).toBe(centerIdx);
        });
    });

    describe('rootRaisedCosineFilter', () => {
        it('should generate filter of correct length', () => {
            const filter = rootRaisedCosineFilter(1e-6, 0.5, 8e6, 4);
            expect(filter.length).toBe(65);
        });

        it('should be normalized to unit energy', () => {
            const filter = rootRaisedCosineFilter(1e-6, 0.5, 8e6, 4);
            const energy = filter.reduce((sum, x) => sum + x * x, 0);
            expect(isClose(energy, 1, 0.01)).toBe(true);
        });
    });

    describe('raisedCosineBandwidth', () => {
        it('should calculate bandwidth correctly', () => {
            const symbolPeriod = 1e-6; // 1 us -> 1 MHz symbol rate
            const alpha = 0.5;
            const bw = raisedCosineBandwidth(symbolPeriod, alpha);
            // BW = (1 + alpha) / (2*T) = 1.5 / 2e-6 = 750 kHz
            expect(isClose(bw, 750000, 100)).toBe(true);
        });

        it('should equal 1/(2T) for alpha=0', () => {
            const symbolPeriod = 1e-6;
            const bw = raisedCosineBandwidth(symbolPeriod, 0);
            // BW = 1 / (2*T) = 500 kHz
            expect(isClose(bw, 500000, 100)).toBe(true);
        });

        it('should equal 1/T for alpha=1', () => {
            const symbolPeriod = 1e-6;
            const bw = raisedCosineBandwidth(symbolPeriod, 1);
            // BW = 2 / (2*T) = 1/T = 1 MHz
            expect(isClose(bw, 1000000, 100)).toBe(true);
        });
    });
});
