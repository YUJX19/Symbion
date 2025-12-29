/**
 * @module utils
 * @description General utility functions module
 * 
 * Provides dB conversion, complex number operations, and statistical functions.
 */

// Re-export all utility functions
export {
    linearToDb,
    dbToLinear,
    wattsToDbm,
    dbmToWatts,
    frequencyToWavelength,
    wavelengthToFrequency
} from './conversion';

export {
    gaussianPdf,
    gaussianRandom,
    complexGaussianRandom,
    mean,
    variance,
    standardDeviation,
    qFunction,
    erfc
} from './statistics';

// Coordinate system transformations (Three.js ↔ Physics/MINCO)
export {
    threeToPhysics,
    physicsToThree,
    mincoToThree,
    threeToMinco,
    threeToPhysicsVel,
    physicsToThreeVel,
    threeToPhysicsQuat,
    physicsToThreeQuat,
    threeStateToPhysics,
    physicsStateToThree,
    trajectoryStateToThree,
    trajectoryStateToPhysics,
    createHoverStateFromThree,
    createHoverStateFromPhysics,
    physicsPathToThree,
    threePathToPhysics,
    threeBuildingToPhysics,
    threeBuildingsToPhysics,
    vec3ApproxEqual,
    verifyRoundTrip,
    getGravityVector,
    getPhysicsUpVector,
    getPhysicsForwardVector,
    type ThreeJSDroneState,
    type ThreeJSBuilding,
    type PhysicsBuilding
} from './coordinate-transform';

// OccupancyGrid builder utilities
export {
    buildOccupancyGridFromThreeJS,
    isPositionFree,
    isLineFree,
    findClosestFreePosition,
    type ThreeJSBuilding as OccupancyThreeJSBuilding,
    type OccupancyGridConfig
} from './occupancy-grid-builder';

// Physics debugging utilities
export {
    PhysicsDebugger,
    getBodyZAxis,
    checkThrustAlignment,
    runHoverTest,
    type PhysicsDebugEntry
} from './physics-debugger';
