/**
 * @module ai/env
 * @description Iterative AI client layer exports
 *
 * Provides the iterative AI client (IterativeAiClient) and legacy aliases
 * (UavGymEnvClient, GymEnvClient, RlEnvironmentClient) along with
 * input/output space utilities.
 */

// RL-Specific Event Types (from rlTypes)
export {
    type RlClientEventType,
    type RlClientEvent,
    type RlEventCallback,
} from './rlTypes';

// Iterative AI Client (recommended) and legacy aliases
export {
    UavGymEnvClient,
    IterativeAiClient,
    RlEnvironmentClient,
    GymEnvClient,
    type StepResult,
    type ClientState,
    type RlQueryContext,
} from './rlEnvClient';

// RL Observation Utilities
export {
    defaultUavObservation,
    minimalUavObservation,
    fullUavObservation,
    ObservationBuilder,
    normalizeObservation,
    DEFAULT_UAV_OBSERVATION_SPACE,
} from './rlObservation';

// RL Action Utilities
export {
    defaultUavActionMapper,
    clampAction,
    ActionBuilder,
    getMcsSpectralEfficiency,
    getMcsModulation,
    findOptimalMcs,
    DEFAULT_UAV_ACTION_SPACE,
    SIMPLE_ACTION_SPACE,
    FLIGHT_ONLY_ACTION_SPACE,
    EXTENDED_UAV_ACTION_SPACE,
    type ActionMapperConfig,
} from './rlAction';
