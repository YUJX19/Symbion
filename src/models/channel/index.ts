/**
 * @module channel
 * @description Channel Modeling Module
 * 
 * Includes implementations of various wireless channel models:
 * - AWGN (Additive White Gaussian Noise)
 * - Two-Ray (Ground Reflection)
 * - Rayleigh/Rician Fading
 * - GSCM (Geometric Stochastic Channel Model)
 * - MIMO (Multiple-Input Multiple-Output)
 */

// AWGN Channel
export * as awgn from './awgn';

// Two-Ray Ground Reflection Model
export * as twoRay from './two-ray';

// Fading Channel (Rayleigh, Rician)
export * as fading from './fading';

// GSCM (Geometric Stochastic Channel Model)
export * as gscm from './gscm';

// MIMO Channel
export * as mimo from './mimo';

// Convenient exports for common functions
export { shannonCapacity, addNoise, calculate as awgnCalculate } from './awgn';
export { breakpointDistance, calculate as twoRayCalculate } from './two-ray';
export {
    rayleighFading,
    ricianFading,
    dopplerShift,
    maxDopplerShift,
    coherenceTime,
    coherenceBandwidth
} from './fading';
export {
    // Primary exports with descriptive names
    initScatterChannel,
    updateScatterChannel,
    getFrequencyResponse,
    type ScatterChannelParams,
    type ScatterChannelState,
    // Backward-compatible aliases (for existing code using GSCM naming)
    initScatterChannel as initGSCMChannel,
    updateScatterChannel as updateGSCMChannel
} from './gscm';
export {
    generateMIMOChannel,
    calculateMIMOCapacity,
    capacityBounds,
    diversityMultiplexingTradeoff,
    type MIMOChannelMatrix
} from './mimo';
