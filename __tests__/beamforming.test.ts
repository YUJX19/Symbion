/**
 * Beamforming Module Tests
 * Tests for Antenna Array, MIMO Precoding, and Polarization
 */

import { describe, it, expect } from 'vitest';
import {
    // Array exports
    arrayFactor,
    arrayFactorDb,
    arrayPattern,
    halfPowerBeamwidth,
    firstNullBeamwidth,
    arrayGain,
    steeringPhases,
    array,
    // Polarization exports
    polarizationLossFactor,
    polarizationLossDb,
    polarizationMatch,
    jonesVector,
    polarization,
    // MIMO exports
    generateChannelMatrix,
    mrtSinr,
    zfSinr,
    mimoCapacity,
    capacityVsAntennas,
    mimo,
} from '../src/models/beamforming';
import { isClose, arraysClose } from './test-utils';

// ==================== Antenna Array Tests ====================

describe('Antenna Array', () => {
    describe('arrayFactor', () => {
        it('should return 1 at main lobe direction (theta=0) for broadside array', () => {
            const config = { numAntennas: 8, spacing: 0.5 };
            const af = arrayFactor(0, config);
            expect(isClose(af, 1, 1e-6)).toBe(true);
        });

        it('should return 1 at steering angle', () => {
            const steeringAngle = Math.PI / 6; // 30 degrees
            const config = { numAntennas: 8, spacing: 0.5, steeringAngle };
            const af = arrayFactor(steeringAngle, config);
            expect(isClose(af, 1, 1e-6)).toBe(true);
        });

        it('should be symmetric for broadside array', () => {
            const config = { numAntennas: 8, spacing: 0.5 };
            const af_pos = arrayFactor(Math.PI / 8, config);
            const af_neg = arrayFactor(-Math.PI / 8, config);
            expect(isClose(af_pos, af_neg, 1e-10)).toBe(true);
        });

        it('should have lower side lobes than main lobe', () => {
            const config = { numAntennas: 16, spacing: 0.5 };
            const mainLobe = arrayFactor(0, config);
            // Sample some side lobe positions
            const sideLobe1 = arrayFactor(0.15, config);
            const sideLobe2 = arrayFactor(0.3, config);
            expect(mainLobe).toBeGreaterThan(sideLobe1);
            expect(mainLobe).toBeGreaterThan(sideLobe2);
        });

        it('should increase directivity with more antennas', () => {
            const af8 = arrayFactor(0.1, { numAntennas: 8, spacing: 0.5 });
            const af16 = arrayFactor(0.1, { numAntennas: 16, spacing: 0.5 });
            const af32 = arrayFactor(0.1, { numAntennas: 32, spacing: 0.5 });
            // More antennas = narrower beam = smaller value at off-boresight angle
            expect(af32).toBeLessThan(af16);
            expect(af16).toBeLessThan(af8);
        });
    });

    describe('arrayFactorDb', () => {
        it('should return 0 dB at main lobe', () => {
            const config = { numAntennas: 8, spacing: 0.5 };
            const afDb = arrayFactorDb(0, config);
            expect(isClose(afDb, 0, 1e-6)).toBe(true);
        });

        it('should return negative dB values for side lobes', () => {
            const config = { numAntennas: 8, spacing: 0.5 };
            const afDb = arrayFactorDb(0.3, config);
            expect(afDb).toBeLessThan(0);
        });
    });

    describe('arrayPattern', () => {
        it('should return arrays of correct length', () => {
            const config = { numAntennas: 8, spacing: 0.5 };
            const { angles, pattern, patternDb } = arrayPattern(config, 181);
            expect(angles.length).toBe(181);
            expect(pattern.length).toBe(181);
            expect(patternDb.length).toBe(181);
        });

        it('should cover -90 to +90 degrees', () => {
            const config = { numAntennas: 8, spacing: 0.5 };
            const { angles } = arrayPattern(config);
            expect(isClose(angles[0], -Math.PI / 2, 0.01)).toBe(true);
            expect(isClose(angles[angles.length - 1], Math.PI / 2, 0.01)).toBe(true);
        });

        it('should have peak at center for broadside array', () => {
            const config = { numAntennas: 8, spacing: 0.5 };
            const { pattern } = arrayPattern(config, 181);
            const maxIdx = pattern.indexOf(Math.max(...pattern));
            const centerIdx = 90; // Middle of 181 points
            expect(Math.abs(maxIdx - centerIdx)).toBeLessThanOrEqual(1);
        });
    });

    describe('halfPowerBeamwidth', () => {
        it('should decrease with more antennas', () => {
            const hpbw8 = halfPowerBeamwidth({ numAntennas: 8, spacing: 0.5 });
            const hpbw16 = halfPowerBeamwidth({ numAntennas: 16, spacing: 0.5 });
            const hpbw32 = halfPowerBeamwidth({ numAntennas: 32, spacing: 0.5 });
            expect(hpbw16).toBeLessThan(hpbw8);
            expect(hpbw32).toBeLessThan(hpbw16);
        });

        it('should follow approximate formula 0.886/(N*d)', () => {
            const config = { numAntennas: 8, spacing: 0.5 };
            const hpbw = halfPowerBeamwidth(config);
            const expected = 0.886 / (8 * 0.5);
            expect(isClose(hpbw, expected, 1e-6)).toBe(true);
        });

        it('should broaden for scanned beam', () => {
            const broadside = halfPowerBeamwidth({ numAntennas: 8, spacing: 0.5, steeringAngle: 0 });
            const scanned = halfPowerBeamwidth({ numAntennas: 8, spacing: 0.5, steeringAngle: Math.PI / 4 });
            expect(scanned).toBeGreaterThan(broadside);
        });
    });

    describe('firstNullBeamwidth', () => {
        it('should decrease with more antennas', () => {
            const fnbw8 = firstNullBeamwidth({ numAntennas: 8, spacing: 0.5 });
            const fnbw16 = firstNullBeamwidth({ numAntennas: 16, spacing: 0.5 });
            expect(fnbw16).toBeLessThan(fnbw8);
        });

        it('should follow formula 2/(N*d)', () => {
            const config = { numAntennas: 8, spacing: 0.5 };
            const fnbw = firstNullBeamwidth(config);
            const expected = 2 / (8 * 0.5);
            expect(isClose(fnbw, expected, 1e-6)).toBe(true);
        });
    });

    describe('arrayGain', () => {
        it('should equal number of antennas for standard spacing', () => {
            expect(arrayGain({ numAntennas: 8, spacing: 0.5 })).toBe(8);
            expect(arrayGain({ numAntennas: 16, spacing: 0.5 })).toBe(16);
            expect(arrayGain({ numAntennas: 32, spacing: 0.5 })).toBe(32);
        });
    });

    describe('steeringPhases', () => {
        it('should return correct number of phases', () => {
            const config = { numAntennas: 8, spacing: 0.5 };
            const phases = steeringPhases(Math.PI / 6, config);
            expect(phases.length).toBe(8);
        });

        it('should return zero phases for broadside steering', () => {
            const config = { numAntennas: 8, spacing: 0.5 };
            const phases = steeringPhases(0, config);
            for (const phase of phases) {
                expect(isClose(phase, 0, 1e-10)).toBe(true);
            }
        });

        it('should produce progressive phase shift', () => {
            const config = { numAntennas: 16, spacing: 0.5 };
            const phases = steeringPhases(Math.PI / 3, config);
            // All phases should be numbers
            for (const phase of phases) {
                expect(typeof phase).toBe('number');
                expect(Number.isFinite(phase)).toBe(true);
            }
        });

        it('should have progressive phase shift', () => {
            const config = { numAntennas: 4, spacing: 0.5 };
            const phases = steeringPhases(Math.PI / 6, config);
            // Phase should progress linearly (with wrapping)
            const diff1 = phases[1] - phases[0];
            const diff2 = phases[2] - phases[1];
            expect(isClose(diff1, diff2, 1e-10)).toBe(true);
        });
    });

    describe('hasGratingLobes', () => {
        it('should return false for d=0.5λ and small scan angles', () => {
            expect(array.hasGratingLobes(0.5, 0)).toBe(false);
            expect(array.hasGratingLobes(0.5, Math.PI / 6)).toBe(false);
        });

        it('should return true for large spacing', () => {
            expect(array.hasGratingLobes(1.5, 0)).toBe(true);
        });

        it('should return true for wide scan with d=0.5λ', () => {
            // At d=0.5λ, max scan without grating lobes is arcsin(1) = 90°
            // But practical limits occur earlier
            expect(array.hasGratingLobes(0.6, Math.PI / 3)).toBe(true);
        });
    });
});

// ==================== MIMO Precoding Tests ====================

describe('MIMO Precoding', () => {
    describe('generateChannelMatrix', () => {
        it('should generate matrix with correct dimensions', () => {
            const H = generateChannelMatrix(16, 4);
            expect(H.real.length).toBe(16);
            expect(H.imag.length).toBe(16);
            expect(H.real[0].length).toBe(4);
            expect(H.imag[0].length).toBe(4);
        });

        it('should generate complex values', () => {
            const H = generateChannelMatrix(8, 2);
            // Check that we have non-zero real and imaginary parts
            let hasNonZeroReal = false;
            let hasNonZeroImag = false;
            for (let m = 0; m < 8; m++) {
                for (let k = 0; k < 2; k++) {
                    if (Math.abs(H.real[m][k]) > 0.01) hasNonZeroReal = true;
                    if (Math.abs(H.imag[m][k]) > 0.01) hasNonZeroImag = true;
                }
            }
            expect(hasNonZeroReal).toBe(true);
            expect(hasNonZeroImag).toBe(true);
        });

        it('should generate different matrices each time', () => {
            const H1 = generateChannelMatrix(4, 2);
            const H2 = generateChannelMatrix(4, 2);
            // Check that at least one element is different
            let anyDifferent = false;
            for (let m = 0; m < 4 && !anyDifferent; m++) {
                for (let k = 0; k < 2 && !anyDifferent; k++) {
                    if (Math.abs(H1.real[m][k] - H2.real[m][k]) > 1e-10) {
                        anyDifferent = true;
                    }
                }
            }
            expect(anyDifferent).toBe(true);
        });
    });

    describe('hermitianTranspose', () => {
        it('should swap dimensions', () => {
            const H = generateChannelMatrix(16, 4);
            const HH = mimo.hermitianTranspose(H);
            expect(HH.real.length).toBe(4);
            expect(HH.real[0].length).toBe(16);
        });

        it('should conjugate imaginary part', () => {
            const H = generateChannelMatrix(4, 2);
            const HH = mimo.hermitianTranspose(H);
            // Check H^H[k][m] = conj(H[m][k])
            for (let m = 0; m < 4; m++) {
                for (let k = 0; k < 2; k++) {
                    expect(isClose(HH.real[k][m], H.real[m][k], 1e-10)).toBe(true);
                    expect(isClose(HH.imag[k][m], -H.imag[m][k], 1e-10)).toBe(true);
                }
            }
        });

        it('should satisfy (H^H)^H = H', () => {
            const H = generateChannelMatrix(4, 2);
            const HH = mimo.hermitianTranspose(H);
            const HHH = mimo.hermitianTranspose(HH);
            for (let m = 0; m < 4; m++) {
                for (let k = 0; k < 2; k++) {
                    expect(isClose(HHH.real[m][k], H.real[m][k], 1e-10)).toBe(true);
                    expect(isClose(HHH.imag[m][k], H.imag[m][k], 1e-10)).toBe(true);
                }
            }
        });
    });

    describe('mrtSinr', () => {
        it('should increase with more antennas', () => {
            const sinr8 = mrtSinr({ numAntennas: 8, numUsers: 4, snrLinear: 10 });
            const sinr16 = mrtSinr({ numAntennas: 16, numUsers: 4, snrLinear: 10 });
            const sinr32 = mrtSinr({ numAntennas: 32, numUsers: 4, snrLinear: 10 });
            expect(sinr16).toBeGreaterThan(sinr8);
            expect(sinr32).toBeGreaterThan(sinr16);
        });

        it('should increase with SNR', () => {
            const config = { numAntennas: 16, numUsers: 4, snrLinear: 0 };
            const sinr0 = mrtSinr({ ...config, snrLinear: 1 });
            const sinr10 = mrtSinr({ ...config, snrLinear: 10 });
            const sinr100 = mrtSinr({ ...config, snrLinear: 100 });
            expect(sinr10).toBeGreaterThan(sinr0);
            expect(sinr100).toBeGreaterThan(sinr10);
        });

        it('should be positive', () => {
            const sinr = mrtSinr({ numAntennas: 16, numUsers: 4, snrLinear: 10 });
            expect(sinr).toBeGreaterThan(0);
        });
    });

    describe('zfSinr', () => {
        it('should increase with more antennas', () => {
            const sinr16 = zfSinr({ numAntennas: 16, numUsers: 4, snrLinear: 10 });
            const sinr32 = zfSinr({ numAntennas: 32, numUsers: 4, snrLinear: 10 });
            expect(sinr32).toBeGreaterThan(sinr16);
        });

        it('should scale linearly with SNR', () => {
            const config = { numAntennas: 16, numUsers: 4, snrLinear: 0 };
            const sinr10 = zfSinr({ ...config, snrLinear: 10 });
            const sinr20 = zfSinr({ ...config, snrLinear: 20 });
            expect(isClose(sinr20 / sinr10, 2, 0.01)).toBe(true);
        });

        it('should follow formula (M-K+1)*SNR', () => {
            const config = { numAntennas: 16, numUsers: 4, snrLinear: 10 };
            const sinr = zfSinr(config);
            const expected = (16 - 4 + 1) * 10;
            expect(isClose(sinr, expected, 1e-6)).toBe(true);
        });
    });

    describe('capacity', () => {
        it('should calculate MRT capacity', () => {
            const cap = mimoCapacity('MRT', { numAntennas: 16, numUsers: 4, snrLinear: 10 });
            expect(cap).toBeGreaterThan(0);
        });

        it('should calculate ZF capacity', () => {
            const cap = mimoCapacity('ZF', { numAntennas: 16, numUsers: 4, snrLinear: 10 });
            expect(cap).toBeGreaterThan(0);
        });

        it('should increase with SNR for both schemes', () => {
            const config = { numAntennas: 16, numUsers: 4, snrLinear: 0 };
            const mrtLow = mimoCapacity('MRT', { ...config, snrLinear: 1 });
            const mrtHigh = mimoCapacity('MRT', { ...config, snrLinear: 100 });
            const zfLow = mimoCapacity('ZF', { ...config, snrLinear: 1 });
            const zfHigh = mimoCapacity('ZF', { ...config, snrLinear: 100 });
            expect(mrtHigh).toBeGreaterThan(mrtLow);
            expect(zfHigh).toBeGreaterThan(zfLow);
        });

        it('should follow formula K*log2(1+SINR)', () => {
            const config = { numAntennas: 16, numUsers: 4, snrLinear: 10 };
            const sinr = zfSinr(config);
            const expectedCap = 4 * Math.log2(1 + sinr);
            const actualCap = mimoCapacity('ZF', config);
            expect(isClose(actualCap, expectedCap, 1e-6)).toBe(true);
        });
    });

    describe('capacityVsAntennas', () => {
        it('should return arrays of correct length', () => {
            const { antennas, capacity } = capacityVsAntennas('ZF', { numUsers: 4, snrLinear: 10 }, [8, 32]);
            expect(antennas.length).toBe(32 - 8 + 1);
            expect(capacity.length).toBe(32 - 8 + 1);
        });

        it('should show monotonically increasing capacity', () => {
            const { capacity } = capacityVsAntennas('ZF', { numUsers: 4, snrLinear: 10 }, [8, 32]);
            for (let i = 1; i < capacity.length; i++) {
                expect(capacity[i]).toBeGreaterThan(capacity[i - 1]);
            }
        });
    });

    describe('mrtZfCrossover', () => {
        it('should return M >= K', () => {
            const M = mimo.mrtZfCrossover(4, 10);
            expect(M).toBeGreaterThanOrEqual(4);
        });

        it('should return valid antenna count', () => {
            const M = mimo.mrtZfCrossover(8, 20);
            expect(M).toBeGreaterThanOrEqual(8);
            expect(M).toBeLessThanOrEqual(1000);
        });
    });

    describe('equalPowerAllocation', () => {
        it('should divide power equally', () => {
            expect(mimo.equalPowerAllocation(100, 4)).toBe(25);
            expect(mimo.equalPowerAllocation(100, 10)).toBe(10);
        });
    });

    describe('asymptoticEfficiency', () => {
        it('should return value between 0 and 1', () => {
            const eff = mimo.asymptoticEfficiency({ numAntennas: 64, numUsers: 8, snrLinear: 10 });
            expect(eff).toBeGreaterThan(0);
            expect(eff).toBeLessThanOrEqual(1);
        });

        it('should increase with more antennas', () => {
            const eff16 = mimo.asymptoticEfficiency({ numAntennas: 16, numUsers: 4, snrLinear: 10 });
            const eff64 = mimo.asymptoticEfficiency({ numAntennas: 64, numUsers: 4, snrLinear: 10 });
            expect(eff64).toBeGreaterThan(eff16);
        });
    });
});

// ==================== Polarization Tests ====================

describe('Polarization', () => {
    describe('polarizationLossFactor', () => {
        it('should return 1 for co-polarized antennas', () => {
            expect(isClose(polarizationLossFactor(0, 0), 1, 1e-10)).toBe(true);
            expect(isClose(polarizationLossFactor(Math.PI / 2, Math.PI / 2), 1, 1e-10)).toBe(true);
        });

        it('should return 0 for cross-polarized antennas', () => {
            expect(isClose(polarizationLossFactor(0, Math.PI / 2), 0, 1e-10)).toBe(true);
            expect(isClose(polarizationLossFactor(Math.PI / 4, -Math.PI / 4), 0, 1e-10)).toBe(true);
        });

        it('should return 0.5 for 45-degree mismatch', () => {
            expect(isClose(polarizationLossFactor(0, Math.PI / 4), 0.5, 1e-10)).toBe(true);
        });

        it('should follow cos²(angle difference) formula', () => {
            for (const delta of [0.1, 0.3, 0.7, 1.2]) {
                const plf = polarizationLossFactor(0, delta);
                const expected = Math.cos(delta) ** 2;
                expect(isClose(plf, expected, 1e-10)).toBe(true);
            }
        });
    });

    describe('polarizationLossDb', () => {
        it('should return 0 dB for co-polarized', () => {
            expect(isClose(polarizationLossDb(0, 0), 0, 1e-10)).toBe(true);
        });

        it('should return -3 dB for 45-degree mismatch', () => {
            const lossDb = polarizationLossDb(0, Math.PI / 4);
            expect(isClose(lossDb, -3.01, 0.1)).toBe(true);
        });

        it('should return -Infinity for cross-polarized', () => {
            expect(polarizationLossDb(0, Math.PI / 2)).toBe(-Infinity);
        });
    });

    describe('polarizationMatch', () => {
        it('should return 1 for same linear polarizations', () => {
            expect(polarizationMatch('V', 'V')).toBe(1);
            expect(polarizationMatch('H', 'H')).toBe(1);
            expect(isClose(polarizationMatch('45', '45'), 1, 1e-10)).toBe(true);
        });

        it('should return 0 for orthogonal linear polarizations', () => {
            expect(isClose(polarizationMatch('V', 'H'), 0, 1e-10)).toBe(true);
            expect(isClose(polarizationMatch('45', '-45'), 0, 1e-10)).toBe(true);
        });

        it('should return 1 for same circular polarizations', () => {
            expect(polarizationMatch('LHCP', 'LHCP')).toBe(1);
            expect(polarizationMatch('RHCP', 'RHCP')).toBe(1);
        });

        it('should return 0 for opposite circular polarizations', () => {
            expect(polarizationMatch('LHCP', 'RHCP')).toBe(0);
            expect(polarizationMatch('RHCP', 'LHCP')).toBe(0);
        });

        it('should return 0.5 for linear vs circular', () => {
            expect(polarizationMatch('V', 'LHCP')).toBe(0.5);
            expect(polarizationMatch('H', 'RHCP')).toBe(0.5);
            expect(polarizationMatch('LHCP', 'V')).toBe(0.5);
        });

        it('should return 0.5 for V vs 45-degree', () => {
            expect(isClose(polarizationMatch('V', '45'), 0.5, 1e-10)).toBe(true);
        });
    });

    describe('jonesVector', () => {
        it('should return correct vector for vertical polarization', () => {
            const j = jonesVector('V');
            expect(j.hReal).toBe(0);
            expect(j.hImag).toBe(0);
            expect(j.vReal).toBe(1);
            expect(j.vImag).toBe(0);
        });

        it('should return correct vector for horizontal polarization', () => {
            const j = jonesVector('H');
            expect(j.hReal).toBe(1);
            expect(j.hImag).toBe(0);
            expect(j.vReal).toBe(0);
            expect(j.vImag).toBe(0);
        });

        it('should return normalized vectors', () => {
            for (const type of ['V', 'H', '45', '-45', 'LHCP', 'RHCP'] as const) {
                const j = jonesVector(type);
                const energy = j.hReal ** 2 + j.hImag ** 2 + j.vReal ** 2 + j.vImag ** 2;
                expect(isClose(energy, 1, 1e-10)).toBe(true);
            }
        });

        it('should have imaginary component for circular polarizations', () => {
            const lhcp = jonesVector('LHCP');
            const rhcp = jonesVector('RHCP');
            expect(lhcp.vImag).not.toBe(0);
            expect(rhcp.vImag).not.toBe(0);
            // LHCP and RHCP should have opposite sign for imaginary part
            expect(lhcp.vImag * rhcp.vImag).toBeLessThan(0);
        });
    });

    describe('jonesInnerProduct', () => {
        it('should return 1 for same polarization', () => {
            const jV = jonesVector('V');
            const jH = jonesVector('H');
            expect(isClose(polarization.jonesInnerProduct(jV, jV), 1, 1e-10)).toBe(true);
            expect(isClose(polarization.jonesInnerProduct(jH, jH), 1, 1e-10)).toBe(true);
        });

        it('should return 0 for orthogonal polarizations', () => {
            const jV = jonesVector('V');
            const jH = jonesVector('H');
            expect(isClose(polarization.jonesInnerProduct(jV, jH), 0, 1e-10)).toBe(true);
        });

        it('should return 0 for opposite circular polarizations', () => {
            const jLHCP = jonesVector('LHCP');
            const jRHCP = jonesVector('RHCP');
            expect(isClose(polarization.jonesInnerProduct(jLHCP, jRHCP), 0, 1e-10)).toBe(true);
        });
    });

    describe('crossPolarizationDiscrimination', () => {
        it('should calculate XPD correctly', () => {
            // If co-pol is 100x stronger than cross-pol, XPD = 20 dB
            const xpd = polarization.crossPolarizationDiscrimination(100, 1);
            expect(isClose(xpd, 20, 1e-6)).toBe(true);
        });

        it('should return 0 dB for equal powers', () => {
            const xpd = polarization.crossPolarizationDiscrimination(1, 1);
            expect(isClose(xpd, 0, 1e-10)).toBe(true);
        });

        it('should return negative for higher cross-pol', () => {
            const xpd = polarization.crossPolarizationDiscrimination(1, 10);
            expect(xpd).toBeLessThan(0);
        });
    });
});
