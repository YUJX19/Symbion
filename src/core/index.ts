/**
 * @module core
 * @description Core framework for reproducible experiments
 *
 * Provides cross-task common experiment running capabilities and reproducibility guarantees.
 * This is the foundational layer that domain-specific modules (ai, isac, models, extras) build upon.
 *
 * ## Modules
 * - `space`: Input/Output space definitions for iterative environments
 * - `objective`: MetricSpec and objective composition with tracking
 * - `constraint`: Hard/soft constraint abstraction
 * - `runner`: Unified experiment execution loop
 * - `logging`: JSONL/CSV structured logging
 * - `repro`: Reproducibility (TaskConfig, seeded RNG, schema hash)
 * - `errors`: Unified error types and codes
 */

// ==================== Space ====================

export type {
    DiscreteSpace,
    BoxSpace,
    MultiDiscreteSpace,
    DictSpace,
    TupleSpace,
    Space,
    SpaceValue,
    DiscreteValue,
    BoxValue,
    MultiDiscreteValue,
    DictValue,
    TupleValue,
} from './space';

export {
    discrete,
    box,
    multiDiscrete,
    dict,
    tuple,
    sample,
    contains,
    getDimension,
    serialize,
    computeSchemaHash,
    flatten,
    unflatten,
} from './space';

// ==================== Objective ====================

export type {
    OptimizationDirection,
    MetricCategory,
    MetricMetadata,
    MetricEvaluator,
    MetricSpec,
    MetricEvaluationResult,
    MetricComposition,
} from './objective';

export {
    asMetricSpec,
    simpleMetric,
    normalizeValue,
    normalizeMetric,
    clampMetric,
    createSmoothMetric,
    combineMetricsWithTracking,
    combineMetrics,
    MetricRegistry,
    createMetricRegistry,
    globalMetricRegistry,
} from './objective';

// ==================== Constraint ====================

export type {
    ConstraintSeverity,
    ConstraintOperator,
    ConstraintMetadata,
    ConstraintEvaluator,
    ConstraintSpec,
    ConstraintResult,
    ConstraintReport,
} from './constraint';

export {
    asConstraintSpec,
    leConstraint,
    geConstraint,
    eqConstraint,
    evaluateConstraint,
    evaluateConstraints,
    constraintsToPenalty,
} from './constraint';

// ==================== Runner ====================

export type {
    Environment,
    DecisionMaker,
    RunnerConfig,
    StepResult,
    EpisodeResult,
    RunResult,
} from './runner';

export {
    Runner,
    runExperiment,
    createDummyEnvironment,
    createRandomDecisionMaker,
} from './runner';

// ==================== Logging ====================

export type {
    LogLevel,
    BaseLogEntry,
    StepLogEntry,
    EpisodeLogEntry,
    ReportLogEntry,
    LogEntry,
    Logger,
    LoggerConfig,
    JsonlLogger,
    CsvLogger,
} from './logging';

export {
    MultiLogger,
    ConsoleLogger,
    MemoryLogger,
    createLogger,
    createBrowserLogger,
} from './logging';

// ==================== Repro ====================

export type {
    TaskConfig,
    TaskConfigInput,
    ValidationResult,
    ExperimentMetadata,
} from './repro';

export {
    createTaskConfig,
    serializeTaskConfig,
    deserializeTaskConfig,
    computeConfigHash,
    validateTaskConfig,
    areConfigsCompatible,
    SeededRandom,
    createRng,
    createExperimentMetadata,
} from './repro';

// ==================== Errors ====================

export {
    ErrorCodes,
    SymbionError,
    TimeoutError,
    ProtocolError,
    ValidationError,
    ConstraintViolationError,
    SchemaMismatchError,
    ConnectionError,
    NotInitializedError,
    isSymbionError,
    hasErrorCode,
    wrapError,
    createTimeout,
    withTimeout,
} from './errors';

export type {
    ErrorCode,
    ConstraintViolationDetail,
} from './errors';
