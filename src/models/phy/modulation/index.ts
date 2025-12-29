/**
 * @module modulation
 * @description Modulation Techniques Library
 * 
 * Includes various digital and analog modulation technologies:
 * - QAM (Quadrature Amplitude Modulation)
 * - Analog Modulation (AM, FM, PM)
 * - Pulse Shaping (Raised Cosine, etc.)
 * - OTFS (Orthogonal Time Frequency Space)
 */

export * from './qam';
export * from './analog';
export * from './pulse-shaping';
export * from './otfs';

// Namespace exports for organized access
export * as analog from './analog';
export * as pulseShaping from './pulse-shaping';
export * as qam from './qam';
export * as otfs from './otfs';

// Convenient exports for common functions
export {
    generateConstellation,
    normalizeConstellation,
    addNoise as addQamNoise,
    hardDecision,
    theoreticalSer,
    generateQAMData
} from './qam';

export {
    dsbScModulate,
    dsbScDemodulate,
    ssbModulate,
    amModulate,
    amEnvelopeDetect
} from './analog';

export {
    raisedCosine,
    rootRaisedCosine,
    raisedCosineFilter,
    rootRaisedCosineFilter,
    raisedCosineBandwidth,
    sinc
} from './pulse-shaping';

