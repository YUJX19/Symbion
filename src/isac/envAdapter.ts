/**
 * @module src/isac/envAdapter
 * @description Environment Adapter for AI Training
 */

// Re-export from tasks module
export {
    IsacTrajectoryEnvironment,
    createEnvironment,
    type IsacStepInfo,
} from '../tasks/isac-trajectory/environment';

export {
    extractInput,
    inputToVector,
    getInputSpace,
    getOutputSpace,
    continuousOutputToOffset,
    discreteOutputToOffset,
    type IsacInput,
} from '../tasks/isac-trajectory/observation';

