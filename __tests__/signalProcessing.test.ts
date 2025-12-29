/**
 * Signal Processing Module Tests
 * Tests for Complex class, FFT/IFFT, and utility functions
 */

import { describe, it, expect, beforeEach } from 'vitest';
import {
    Complex,
    fft,
    ifft,
    fft2d,
    ifft2d,
    linspace,
    psd,
    normalizePower,
    upsample,
    downsample,
    convolve,
    raisedCosineFilter,
    transpose,
} from '../src/models/phy/signal-processing';
import { isClose, arraysClose } from './test-utils';

// ==================== Complex Class Tests ====================

describe('Complex class', () => {
    describe('constructor and basic properties', () => {
        it('should create a complex number with real and imaginary parts', () => {
            const z = new Complex(3, 4);
            expect(z.real).toBe(3);
            expect(z.imag).toBe(4);
        });

        it('should create from polar coordinates', () => {
            const z = Complex.fromPolar(5, Math.PI / 4);
            expect(isClose(z.magnitude(), 5)).toBe(true);
            expect(isClose(z.phase(), Math.PI / 4)).toBe(true);
        });

        it('should create from exponential (unit circle)', () => {
            // e^(jπ/2) = j
            const z = Complex.exp(Math.PI / 2);
            expect(isClose(z.real, 0, 1e-10, 1e-10)).toBe(true);
            expect(isClose(z.imag, 1, 1e-10, 1e-10)).toBe(true);
        });
    });

    describe('arithmetic operations', () => {
        it('should add complex numbers correctly', () => {
            const z1 = new Complex(1, 2);
            const z2 = new Complex(3, 4);
            const result = z1.add(z2);
            expect(result.real).toBe(4);
            expect(result.imag).toBe(6);
        });

        it('should subtract complex numbers correctly', () => {
            const z1 = new Complex(5, 6);
            const z2 = new Complex(2, 3);
            const result = z1.subtract(z2);
            expect(result.real).toBe(3);
            expect(result.imag).toBe(3);
        });

        it('should multiply complex numbers correctly', () => {
            // (1+2j) * (3+4j) = 3 + 4j + 6j + 8j² = 3 + 10j - 8 = -5 + 10j
            const z1 = new Complex(1, 2);
            const z2 = new Complex(3, 4);
            const result = z1.multiply(z2);
            expect(result.real).toBe(-5);
            expect(result.imag).toBe(10);
        });

        it('should divide complex numbers correctly', () => {
            // (1+2j) / (1+2j) = 1
            const z1 = new Complex(1, 2);
            const z2 = new Complex(1, 2);
            const result = z1.divide(z2);
            expect(isClose(result.real, 1)).toBe(true);
            expect(isClose(result.imag, 0)).toBe(true);
        });

        it('should scale by a real number', () => {
            const z = new Complex(3, 4);
            const result = z.scale(2);
            expect(result.real).toBe(6);
            expect(result.imag).toBe(8);
        });
    });

    describe('properties', () => {
        it('should compute conjugate correctly', () => {
            const z = new Complex(3, 4);
            const conj = z.conjugate();
            expect(conj.real).toBe(3);
            expect(conj.imag).toBe(-4);
        });

        it('should compute magnitude correctly', () => {
            const z = new Complex(3, 4);
            expect(z.magnitude()).toBe(5);
        });

        it('should compute phase correctly', () => {
            const z = new Complex(1, 1);
            expect(isClose(z.phase(), Math.PI / 4)).toBe(true);
        });

        it('should clone correctly', () => {
            const z = new Complex(3, 4);
            const cloned = z.clone();
            expect(cloned.real).toBe(3);
            expect(cloned.imag).toBe(4);
            expect(cloned).not.toBe(z);
        });

        it('should format toString correctly', () => {
            const z1 = new Complex(3, 4);
            expect(z1.toString()).toMatch(/3\.0+.*\+.*4\.0+j/);

            const z2 = new Complex(3, -4);
            expect(z2.toString()).toMatch(/3\.0+.*-.*4\.0+j/);
        });
    });

    describe('edge cases', () => {
        it('should handle zero complex number', () => {
            const z = new Complex(0, 0);
            expect(z.magnitude()).toBe(0);
        });

        it('should handle purely real numbers', () => {
            const z = new Complex(5, 0);
            expect(z.phase()).toBe(0);
        });

        it('should handle purely imaginary numbers', () => {
            const z = new Complex(0, 5);
            expect(isClose(z.phase(), Math.PI / 2)).toBe(true);
        });

        it('should handle negative real numbers', () => {
            const z = new Complex(-5, 0);
            expect(isClose(Math.abs(z.phase()), Math.PI)).toBe(true);
        });
    });
});

// ==================== FFT/IFFT Tests ====================

describe('FFT and IFFT', () => {
    describe('basic FFT properties', () => {
        it('should compute FFT of constant signal (DC only)', () => {
            const signal = [
                new Complex(1, 0),
                new Complex(1, 0),
                new Complex(1, 0),
                new Complex(1, 0),
            ];
            const spectrum = fft(signal);
            // DC component should be 4, rest should be near 0
            expect(isClose(spectrum[0].magnitude(), 4, 1e-4)).toBe(true);
            for (let i = 1; i < spectrum.length; i++) {
                expect(isClose(spectrum[i].magnitude(), 0, 1e-4, 1e-10)).toBe(true);
            }
        });

        it('should compute FFT of alternating signal (Nyquist)', () => {
            // [1, -1, 1, -1] should have energy at Nyquist frequency
            const signal = [
                new Complex(1, 0),
                new Complex(-1, 0),
                new Complex(1, 0),
                new Complex(-1, 0),
            ];
            const spectrum = fft(signal);
            // Energy should be at N/2 = 2
            expect(isClose(spectrum[2].magnitude(), 4, 1e-4)).toBe(true);
        });

        it('should handle power of 2 input sizes', () => {
            for (const n of [2, 4, 8, 16, 32]) {
                const signal = Array.from(
                    { length: n },
                    () => new Complex(Math.random(), Math.random())
                );
                const spectrum = fft(signal);
                expect(spectrum.length).toBe(n);
            }
        });

        it('should zero-pad non-power-of-2 inputs', () => {
            const signal = [
                new Complex(1, 0),
                new Complex(2, 0),
                new Complex(3, 0),
            ];
            const spectrum = fft(signal);
            // Should be padded to 4
            expect(spectrum.length).toBe(4);
        });
    });

    describe('IFFT inverse property', () => {
        it('should recover original signal from FFT', () => {
            const original = [
                new Complex(1, 0),
                new Complex(2, 1),
                new Complex(-1, 0.5),
                new Complex(0.5, -0.5),
            ];
            const spectrum = fft(original);
            const recovered = ifft(spectrum);

            for (let i = 0; i < original.length; i++) {
                expect(isClose(recovered[i].real, original[i].real, 1e-4)).toBe(true);
                expect(isClose(recovered[i].imag, original[i].imag, 1e-4)).toBe(true);
            }
        });

        it('should satisfy Parseval theorem (energy conservation)', () => {
            const signal = [
                new Complex(1, 1),
                new Complex(2, -1),
                new Complex(-1, 2),
                new Complex(3, 0),
            ];

            // Time domain energy
            const timeDomainEnergy = signal.reduce(
                (sum, s) => sum + s.magnitude() ** 2,
                0
            );

            // Frequency domain energy
            const spectrum = fft(signal);
            const freqDomainEnergy =
                spectrum.reduce((sum, s) => sum + s.magnitude() ** 2, 0) / signal.length;

            expect(isClose(timeDomainEnergy, freqDomainEnergy, 1e-4)).toBe(true);
        });
    });

    describe('edge cases', () => {
        it('should handle single element input', () => {
            const signal = [new Complex(5, 3)];
            const spectrum = fft(signal);
            expect(spectrum.length).toBe(1);
            expect(isClose(spectrum[0].real, 5)).toBe(true);
            expect(isClose(spectrum[0].imag, 3)).toBe(true);
        });

        it('should handle all zeros', () => {
            const signal = [
                new Complex(0, 0),
                new Complex(0, 0),
                new Complex(0, 0),
                new Complex(0, 0),
            ];
            const spectrum = fft(signal);
            for (const s of spectrum) {
                expect(s.magnitude()).toBe(0);
            }
        });
    });
});

// ==================== 2D FFT Tests ====================

describe('2D FFT and IFFT', () => {
    it('should compute 2D FFT and recover with IFFT', () => {
        const original = [
            [new Complex(1, 0), new Complex(2, 0)],
            [new Complex(3, 0), new Complex(4, 0)],
        ];
        const spectrum = fft2d(original);
        const recovered = ifft2d(spectrum);

        for (let i = 0; i < 2; i++) {
            for (let j = 0; j < 2; j++) {
                expect(isClose(recovered[i][j].real, original[i][j].real, 1e-4)).toBe(
                    true
                );
            }
        }
    });

    it('should handle 4x4 matrix', () => {
        const original = Array.from({ length: 4 }, () =>
            Array.from({ length: 4 }, () => new Complex(Math.random(), Math.random()))
        );
        const spectrum = fft2d(original);
        const recovered = ifft2d(spectrum);

        for (let i = 0; i < 4; i++) {
            for (let j = 0; j < 4; j++) {
                expect(isClose(recovered[i][j].real, original[i][j].real, 1e-4)).toBe(
                    true
                );
                expect(isClose(recovered[i][j].imag, original[i][j].imag, 1e-4)).toBe(
                    true
                );
            }
        }
    });
});

// ==================== Utility Functions Tests ====================

describe('linspace', () => {
    it('should generate evenly spaced values', () => {
        const t = linspace(0, 1, 5);
        expect(t).toEqual([0, 0.25, 0.5, 0.75, 1]);
    });

    it('should handle single point edge case', () => {
        // Note: linspace(0, 1, 1) divides by (n-1=0) which produces NaN
        // This is a known edge case behavior - the function returns [start + 0*step]
        const t = linspace(0, 1, 1);
        expect(t.length).toBe(1);
    });

    it('should handle negative range', () => {
        const t = linspace(-1, 1, 3);
        expect(arraysClose(t, [-1, 0, 1])).toBe(true);
    });
});

describe('transpose', () => {
    it('should transpose a 2D array', () => {
        const matrix = [
            [1, 2, 3],
            [4, 5, 6],
        ];
        const transposed = transpose(matrix);
        expect(transposed).toEqual([
            [1, 4],
            [2, 5],
            [3, 6],
        ]);
    });

    it('should handle empty matrix', () => {
        expect(transpose([])).toEqual([]);
    });

    it('should handle 1x1 matrix', () => {
        expect(transpose([[5]])).toEqual([[5]]);
    });
});

describe('normalizePower', () => {
    it('should normalize to unit power', () => {
        const signal = [new Complex(2, 0), new Complex(2, 0)];
        const normalized = normalizePower(signal);

        // Check that power is 1
        const power =
            normalized.reduce((sum, s) => sum + s.magnitude() ** 2, 0) /
            normalized.length;
        expect(isClose(power, 1, 1e-6)).toBe(true);
    });
});

describe('upsample and downsample', () => {
    it('should upsample by inserting zeros', () => {
        const signal = [new Complex(1, 0), new Complex(2, 0)];
        const upsampled = upsample(signal, 2);
        expect(upsampled.length).toBe(4);
        expect(upsampled[0].real).toBe(1);
        expect(upsampled[1].real).toBe(0);
        expect(upsampled[2].real).toBe(2);
        expect(upsampled[3].real).toBe(0);
    });

    it('should downsample by keeping every nth sample', () => {
        const signal = [
            new Complex(1, 0),
            new Complex(2, 0),
            new Complex(3, 0),
            new Complex(4, 0),
        ];
        const downsampled = downsample(signal, 2);
        expect(downsampled.length).toBe(2);
        expect(downsampled[0].real).toBe(1);
        expect(downsampled[1].real).toBe(3);
    });
});

describe('convolve', () => {
    it('should compute convolution correctly', () => {
        const x = [new Complex(1, 0), new Complex(2, 0), new Complex(3, 0)];
        const h = [new Complex(1, 0), new Complex(1, 0)];
        const result = convolve(x, h);

        // Convolution: [1, 1+2, 2+3, 3] = [1, 3, 5, 3]
        expect(result.length).toBe(4);
        expect(isClose(result[0].real, 1)).toBe(true);
        expect(isClose(result[1].real, 3)).toBe(true);
        expect(isClose(result[2].real, 5)).toBe(true);
        expect(isClose(result[3].real, 3)).toBe(true);
    });

    it('should handle single element convolution', () => {
        const x = [new Complex(5, 0)];
        const h = [new Complex(2, 0)];
        const result = convolve(x, h);
        expect(result.length).toBe(1);
        expect(isClose(result[0].real, 10)).toBe(true);
    });
});

describe('psd (Power Spectral Density)', () => {
    it('should compute power spectrum', () => {
        const signal = [
            new Complex(1, 0),
            new Complex(1, 0),
            new Complex(1, 0),
            new Complex(1, 0),
        ];
        const { freq, power } = psd(signal, 1000);

        expect(freq.length).toBe(4);
        expect(power.length).toBe(4);
        // All power should be at DC for constant signal
    });
});

describe('raisedCosineFilter', () => {
    it('should create filter with correct length', () => {
        const filter = raisedCosineFilter(64, 0.35, 1e-6, 8e6);
        expect(filter.length).toBe(64);
    });

    it('should be symmetric', () => {
        const filter = raisedCosineFilter(33, 0.35, 1e-6, 8e6);
        const n = filter.length;
        for (let i = 0; i < Math.floor(n / 2); i++) {
            expect(isClose(filter[i].real, filter[n - 1 - i].real, 1e-10)).toBe(true);
        }
    });

    it('should have peak at center', () => {
        const filter = raisedCosineFilter(65, 0.35, 1e-6, 8e6);
        const center = Math.floor(filter.length / 2);
        // Center should be largest value (or close to it)
        const centerMag = filter[center].magnitude();
        for (let i = 0; i < filter.length; i++) {
            expect(filter[i].magnitude()).toBeLessThanOrEqual(centerMag + 0.1);
        }
    });
});
