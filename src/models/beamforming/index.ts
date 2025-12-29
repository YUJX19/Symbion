/**
 * @module beamforming
 * @description Beamforming processing module
 * 
 * Includes antenna array and beamforming algorithms:
 * - Array factor and pattern generation
 * - Polarization matching
 * - Massive MIMO precoding
 */

// Antenna Array
export * as array from './array';

// Polarization
export * as polarization from './polarization';

// Massive MIMO
export * as mimo from './mimo';

// Convenient exports for common functions
export {
    arrayFactor,
    arrayFactorDb,
    arrayPattern,
    halfPowerBeamwidth,
    firstNullBeamwidth,
    arrayGain,
    arrayGainDb,
    steeringPhases
} from './array';

export {
    polarizationLossFactor,
    polarizationLossDb,
    polarizationMatch,
    jonesVector
} from './polarization';

export {
    generateChannelMatrix,
    mrtSinr,
    zfSinr,
    capacity as mimoCapacity,
    capacityVsAntennas
} from './mimo';

