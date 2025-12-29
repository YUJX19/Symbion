/**
 * Utils Module Tests
 * Tests for dB conversion, statistics, and coordinate transforms
 */

import { describe, it, expect } from 'vitest';
import {
    linearToDb,
    dbToLinear,
    wattsToDbm,
    dbmToWatts,
    frequencyToWavelength,
    wavelengthToFrequency,
    gaussianPdf,
    gaussianRandom,
    complexGaussianRandom,
    mean,
    variance,
    standardDeviation,
    qFunction,
    erfc,
} from '../src/models/utils';
import { isClose, arraysClose } from './test-utils';

// ==================== dB Conversion Tests ====================

describe('dB Conversion Functions', () => {
    describe('linearToDb', () => {
        it('should convert linear to dB correctly', () => {
            expect(isClose(linearToDb(10), 10)).toBe(true);
            expect(isClose(linearToDb(100), 20)).toBe(true);
            expect(isClose(linearToDb(1000), 30)).toBe(true);
        });

        it('should handle fractional values', () => {
            expect(isClose(linearToDb(0.5), -3.0103, 1e-3)).toBe(true);
            expect(isClose(linearToDb(0.1), -10)).toBe(true);
            expect(isClose(linearToDb(0.01), -20)).toBe(true);
        });

        it('should handle 1 (0 dB)', () => {
            expect(linearToDb(1)).toBe(0);
        });

        it('should throw for non-positive values', () => {
            expect(() => linearToDb(0)).toThrow();
            expect(() => linearToDb(-1)).toThrow();
        });
    });

    describe('dbToLinear', () => {
        it('should convert dB to linear correctly', () => {
            expect(isClose(dbToLinear(10), 10)).toBe(true);
            expect(isClose(dbToLinear(20), 100)).toBe(true);
            expect(isClose(dbToLinear(30), 1000)).toBe(true);
        });

        it('should handle negative dB', () => {
            expect(isClose(dbToLinear(-10), 0.1)).toBe(true);
            expect(isClose(dbToLinear(-20), 0.01)).toBe(true);
        });

        it('should handle 0 dB (returns 1)', () => {
            expect(dbToLinear(0)).toBe(1);
        });
    });

    describe('round-trip conversion', () => {
        it('should be inverse functions', () => {
            const values = [0.1, 1, 10, 100, 1000];
            for (const v of values) {
                const roundTrip = dbToLinear(linearToDb(v));
                expect(isClose(roundTrip, v, 1e-10)).toBe(true);
            }
        });
    });
});

// ==================== Power Conversion Tests ====================

describe('Power Conversion Functions', () => {
    describe('wattsToDbm', () => {
        it('should convert watts to dBm correctly', () => {
            expect(isClose(wattsToDbm(1), 30)).toBe(true);       // 1W = 30 dBm
            expect(isClose(wattsToDbm(0.001), 0)).toBe(true);    // 1mW = 0 dBm
            expect(isClose(wattsToDbm(0.1), 20)).toBe(true);     // 100mW = 20 dBm
        });

        it('should throw for non-positive values', () => {
            expect(() => wattsToDbm(0)).toThrow();
            expect(() => wattsToDbm(-1)).toThrow();
        });
    });

    describe('dbmToWatts', () => {
        it('should convert dBm to watts correctly', () => {
            expect(isClose(dbmToWatts(30), 1)).toBe(true);       // 30 dBm = 1W
            expect(isClose(dbmToWatts(0), 0.001)).toBe(true);    // 0 dBm = 1mW
            expect(isClose(dbmToWatts(20), 0.1)).toBe(true);     // 20 dBm = 100mW
        });

        it('should handle negative dBm', () => {
            expect(isClose(dbmToWatts(-30), 0.000001)).toBe(true); // -30 dBm = 1μW
        });
    });

    describe('round-trip power conversion', () => {
        it('should be inverse functions', () => {
            const dbmValues = [-30, -10, 0, 10, 20, 30];
            for (const dbm of dbmValues) {
                const roundTrip = wattsToDbm(dbmToWatts(dbm));
                expect(isClose(roundTrip, dbm, 1e-10)).toBe(true);
            }
        });
    });
});

// ==================== Frequency/Wavelength Conversion Tests ====================

describe('Frequency/Wavelength Conversion', () => {
    const SPEED_OF_LIGHT = 299792458;

    describe('frequencyToWavelength', () => {
        it('should convert frequency to wavelength', () => {
            // 1 GHz → ~0.3m
            expect(isClose(frequencyToWavelength(1e9), SPEED_OF_LIGHT / 1e9)).toBe(true);

            // 2.4 GHz → ~0.125m
            expect(isClose(frequencyToWavelength(2.4e9), SPEED_OF_LIGHT / 2.4e9)).toBe(true);
        });
    });

    describe('wavelengthToFrequency', () => {
        it('should convert wavelength to frequency', () => {
            const lambda = 0.1; // 10cm
            expect(isClose(wavelengthToFrequency(lambda), SPEED_OF_LIGHT / lambda)).toBe(true);
        });
    });

    describe('round-trip conversion', () => {
        it('should be inverse functions', () => {
            const frequencies = [1e9, 2.4e9, 5e9, 28e9, 60e9];
            for (const f of frequencies) {
                const roundTrip = wavelengthToFrequency(frequencyToWavelength(f));
                expect(isClose(roundTrip, f, 1e-6)).toBe(true);
            }
        });
    });
});

// ==================== Statistics Tests ====================

describe('Statistics Functions', () => {
    describe('gaussianPdf', () => {
        it('should compute PDF for standard normal distribution', () => {
            // Peak at mean
            const peak = gaussianPdf(0, 0, 1);
            expect(isClose(peak, 1 / Math.sqrt(2 * Math.PI), 1e-10)).toBe(true);
        });

        it('should be symmetric around mean', () => {
            expect(isClose(gaussianPdf(-1, 0, 1), gaussianPdf(1, 0, 1))).toBe(true);
            expect(isClose(gaussianPdf(-2, 0, 1), gaussianPdf(2, 0, 1))).toBe(true);
        });

        it('should throw for non-positive std', () => {
            expect(() => gaussianPdf(0, 0, 0)).toThrow();
            expect(() => gaussianPdf(0, 0, -1)).toThrow();
        });
    });

    describe('gaussianRandom', () => {
        it('should generate random numbers with correct mean (statistical)', () => {
            const samples: number[] = [];
            for (let i = 0; i < 10000; i++) {
                samples.push(gaussianRandom(5, 1));
            }
            const sampleMean = samples.reduce((a, b) => a + b, 0) / samples.length;
            // With 10000 samples, mean should be within ~0.05 of 5
            expect(Math.abs(sampleMean - 5)).toBeLessThan(0.1);
        });

        it('should generate random numbers with correct std (statistical)', () => {
            const samples: number[] = [];
            for (let i = 0; i < 10000; i++) {
                samples.push(gaussianRandom(0, 2));
            }
            const sampleMean = samples.reduce((a, b) => a + b, 0) / samples.length;
            const sampleVar = samples.reduce((a, b) => a + (b - sampleMean) ** 2, 0) / samples.length;
            const sampleStd = Math.sqrt(sampleVar);
            // With 10000 samples, std should be within ~0.1 of 2
            expect(Math.abs(sampleStd - 2)).toBeLessThan(0.15);
        });
    });

    describe('complexGaussianRandom', () => {
        it('should return an array of length 2', () => {
            const [real, imag] = complexGaussianRandom();
            expect(typeof real).toBe('number');
            expect(typeof imag).toBe('number');
        });
    });

    describe('mean', () => {
        it('should compute mean correctly', () => {
            expect(mean([1, 2, 3, 4, 5])).toBe(3);
            expect(mean([10])).toBe(10);
            expect(mean([-5, 5])).toBe(0);
        });

        it('should throw for empty array', () => {
            expect(() => mean([])).toThrow();
        });
    });

    describe('variance', () => {
        it('should compute sample variance correctly', () => {
            // [1, 2, 3] mean = 2, deviations = [-1, 0, 1], squared = [1, 0, 1]
            // Sample variance = (1+0+1) / (3-1) = 1
            expect(variance([1, 2, 3])).toBe(1);
        });

        it('should compute population variance when specified', () => {
            // Population variance = (1+0+1) / 3 = 2/3
            expect(isClose(variance([1, 2, 3], false), 2 / 3)).toBe(true);
        });

        it('should throw for empty array', () => {
            expect(() => variance([])).toThrow();
        });
    });

    describe('standardDeviation', () => {
        it('should be square root of variance', () => {
            const data = [1, 2, 3, 4, 5];
            const v = variance(data);
            const s = standardDeviation(data);
            expect(isClose(s, Math.sqrt(v))).toBe(true);
        });
    });

    describe('qFunction', () => {
        it('should return 0.5 at x=0', () => {
            expect(isClose(qFunction(0), 0.5, 1e-4)).toBe(true);
        });

        it('should approach 0 for large x', () => {
            expect(qFunction(5)).toBeLessThan(1e-6);
        });

        it('should approach 1 for large negative x', () => {
            expect(isClose(qFunction(-5), 1, 1e-4)).toBe(true);
        });
    });

    describe('erfc', () => {
        it('should return 1 at x=0', () => {
            expect(isClose(erfc(0), 1)).toBe(true);
        });

        it('should approach 0 for large x', () => {
            expect(erfc(5)).toBeLessThan(1e-10);
        });

        it('should approach 2 for large negative x', () => {
            expect(isClose(erfc(-5), 2, 1e-4)).toBe(true);
        });
    });
});
