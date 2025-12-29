# Core Framework Module Guide

> Infrastructure layer for Symbion reproducible experiments

This guide details how to use the Core framework module (`symbion/core`) of the Symbion library, which provides the core abstractions and tools needed for building reproducible scientific experiments.

## Table of Contents

1. [Module Overview](#1-module-overview)
2. [Space - Space Definition](#2-space---space-definition)
3. [Objective - Metric System](#3-objective---metric-system)
4. [Constraint - Constraint System](#4-constraint---constraint-system)
5. [Runner - Experiment Runner](#5-runner---experiment-runner)
6. [Logging - Logging System](#6-logging---logging-system)
7. [Repro - Reproducibility](#7-repro---reproducibility)
8. [Errors - Error Handling](#8-errors---error-handling)
9. [Full Example](#9-full-example)

---

## 1. Module Overview

### 1.1 Design Philosophy

The Core module is the infrastructure layer of the Symbion library, following these design principles:

- ** Reproducibility First**: All experiments can be fully reproduced via seeds and configurations
- ** Type Safety**: Complete TypeScript type definitions
- ** Modular Design**: Each sub-module is independent and composable
- ** Standardized Logging**: Structured logs in JSONL/CSV format
- ** Standard RL Compatible**: Seamless integration with Python Reinforcement Learning ecosystem

### 1.2 Core Submodules

| Submodule | Function | Main Usage |
|-----------|----------|------------|
| **Space** | Space Definition | Define structure of observations and actions |
| **Objective** | Metric System | Combine multiple optimization objectives |
| **Constraint** | Constraint System | Define and evaluate hard/soft constraints |
| **Runner** | Experiment Runner | Execute complete experiment loop |
| **Logging** | Logging System | Record experiment data |
| **Repro** | Reproducibility | Seed, configuration, version management |
| **Errors** | Error Handling | Unified error types and codes |

### 1.3 Relationship with Other Modules

```
┌─────────────────────────────────────────────────────────┐
│                    Research Tasks                       │
│              (U2U-MCS, ISAC-Trajectory)                 │
51: └────────────────────┬────────────────────────────────────┘
                     │ Uses
                     ↓
┌─────────────────────────────────────────────────────────┐
│                    Core Framework                        │
│  ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐ ┌──────┐          │
│  │Space │ │Metric│ │Constr│ │Runner│ │Logger│          │
│  └──────┘ └──────┘ └──────┘ └──────┘ └──────┘          │
└────────────────────┬────────────────────────────────────┘
                     │ Supports
                     ↓
┌─────────────────────────────────────────────────────────┐
│                 Domain Modules                          │
│      (Planning, Control, Communication, etc.)           │
└─────────────────────────────────────────────────────────┘
```

---

## 2. Space - Space Definition

The Space module provides standard RL-compatible observation space and action space definitions.

### 2.1 Space Types

#### DiscreteSpace

Represents a finite set of discrete options [0, n).

```typescript
import { discrete } from 'symbion/core';

// MCS Selection: 0-28 Total 29 options
const mcsSpace = discrete(29, ['MCS-0', 'MCS-1', ..., 'MCS-28']);

// Direction Selection
const directionSpace = discrete(4, ['North', 'East', 'South', 'West']);
```

#### BoxSpace - Continuous Space

Represents an n-dimensional continuous space, with bounds for each dimension.

```typescript
import { box } from 'symbion/core';

// 3D Position Space
const positionSpace = box(
  [3],                    // Shape: 3D vector
  [0, 0, 30],            // Low: x, y, z min values
  [1000, 1000, 150]       // High: x, y, z max values
);

// Normalized Observation Space
const normalizedObs = box(
  [12],        // 12D observation vector
  -1,          // Lower bound -1 for all dimensions
  1,           // Upper bound 1 for all dimensions
  'float32'    // Data type
);
```

#### MultiDiscreteSpace

Combination of multiple discrete spaces.

```typescript
import { multiDiscrete } from 'symbion/core';

// Grid Movement: Horizontal 9 options × Vertical 9 options × Height 5 options
const gridActionSpace = multiDiscrete(
  [9, 9, 5],
  [
    ['left-2', 'left-1', 'stay', 'right-1', 'right-2', ...],
    ['back-2', 'back-1', 'stay', 'forward-1', 'forward-2', ...],
    ['down-2', 'down-1', 'stay', 'up-1', 'up-2'],
  ]
);
```

#### DictSpace

Named dictionary of spaces, used for structured inputs or outputs.

```typescript
import { dict, box, discrete } from 'symbion/core';

const inputSpace = dict({
  // Motion State
  position: box([3], 0, 1000),
  velocity: box([3], -20, 20),
  acceleration: box([3], -5, 5),
  
  // Communication State
  mcsIndex: discrete(29),
  txPower: box([1], 0, 23),
  sinrDb: box([1], -20, 40),
  bler: box([1], 0, 1),
  
  // Sensing State
  targetDistance: box([1], 0, 500),
  targetAngle: box([1], -Math.PI, Math.PI),
});
```

#### TupleSpace

Ordered sequence of spaces.

```typescript
import { tuple, box, discrete } from 'symbion/core';

// Hybrid Output Space: [Continuous Motion Control, MCS Selection]
const outputSpace = tuple([
  box([3], -1, 1),  // Normalized Motion Control
  discrete(29),     // MCS Index
]);
```

### 2.2 Space Operations

#### Sampling

Sample a random valid value from the space.

```typescript
import { sample, createRng } from 'symbion/core';

const space = box([3], 0, 1000);
const rng = createRng(42); // Use seed for reproducibility

const value1 = sample(space, rng.random.bind(rng));
const value2 = sample(space, rng.random.bind(rng));

console.log('Sample 1:', value1); // e.g.: [234.5, 678.9, 123.4]
console.log('Sample 2:', value2); // Different but reproducible
```

#### Validation

Check if a value is within the space.

```typescript
import { contains, box } from 'symbion/core';

const space = box([3], [0, 0, 30], [1000, 1000, 150]);

console.log(contains(space, [100, 200, 50]));  // true
console.log(contains(space, [100, 200, 200])); // false (z out of bounds)
console.log(contains(space, [100, 200]));      // false (dimension mismatch)
```

#### Dimension Calculation

Get total flattened dimensions.

```typescript
import { getDimension, dict, box, discrete } from 'symbion/core';

const space = dict({
  position: box([3], 0, 1000),    // 3D
  mcs: discrete(29),               // 1D
  power: box([1], 0, 23),          // 1D
});

console.log(getDimension(space)); // 5
```

#### Flattening & Unflattening

Convert structured space values to flat vectors.

```typescript
import { flatten, unflatten, dict, box, discrete } from 'symbion/core';

const space = dict({
  position: box([3], 0, 1000),
  mcs: discrete(29),
});

const value = { position: [100, 200, 50], mcs: 12 };

// Flatten
const flattened = flatten(space, value);
console.log(flattened); // [100, 200, 50, 12]

// Unflatten
const restored = unflatten(space, flattened);
console.log(restored);  // { position: [100, 200, 50], mcs: 12 }
```

#### Schema Hash

Compute space hash for validating configuration consistency.

```typescript
import { computeSchemaHash, box } from 'symbion/core';

const space = box([3], 0, 1000);
const hash = computeSchemaHash(space);

console.log('Schema Hash:', hash); // e.g.: "7a3f2e1b..."
```

---

## 3. Objective - Metric System

Objective module provides metric definition, composition, and tracking functions for multi-objective optimization.

### 3.1 MetricSpec Interface

Each metric contains metadata and an evaluation function.

```typescript
import { asMetricSpec, type MetricSpec } from 'symbion/core';

interface State {
  reward: number;
  energy: number;
  distance: number;
}

const rewardMetric: MetricSpec<State> = asMetricSpec(
  (state) => state.reward,  // Evaluation function
  {
    id: 'reward',
    description: 'Cumulative Reward',
    direction: 'maximize',   // Optimization direction
    unit: 'points',
    category: 'performance',
    weight: 0.5,             // Composition weight
  }
);
```

### 3.2 Simplified Metric Creation

```typescript
import { simpleMetric } from 'symbion/core';

// Maximize reward
const reward = simpleMetric('reward', (s) => s.reward, 'maximize');

// Minimize energy
const energy = simpleMetric('energy', (s) => s.energy, 'minimize');

// Minimize distance
const distance = simpleMetric('distance', (s) => s.distance, 'minimize');
```

### 3.3 Metric Transformations

#### Normalization

```typescript
import { normalizeMetric, simpleMetric } from 'symbion/core';

const rawMetric = simpleMetric('sinr', (s) => s.sinrDb, 'maximize');

// Normalize SINR to [0, 1]
const normalized = normalizeMetric(
  rawMetric,
  -20,  // Min
  40    // Max
);

const state = { sinrDb: 10 };
console.log(normalized.evaluate(state)); // (10 - (-20)) / (40 - (-20)) = 0.5
```

#### Clamping

```typescript
import { clampMetric, simpleMetric } from 'symbion/core';

const rawMetric = simpleMetric('speed', (s) => s.speed, 'maximize');

// Limit speed to [0, 20] range
const clamped = clampMetric(rawMetric, 0, 20);

console.log(clamped.evaluate({ speed: 25 }));  // 20
console.log(clamped.evaluate({ speed: -5 }));  // 0
console.log(clamped.evaluate({ speed: 15 }));  // 15
```

#### Smoothing

```typescript
import { createSmoothMetric, simpleMetric } from 'symbion/core';

const rawMetric = simpleMetric('reward', (s) => s.reward, 'maximize');

// Create exponentially smoothed version
const { metric: smoothed, reset } = createSmoothMetric(rawMetric, 0.9);

console.log(smoothed.evaluate({ reward: 10 }));  // First: 10
console.log(smoothed.evaluate({ reward: 20 }));  // 0.9 * 10 + 0.1 * 20 = 11
console.log(smoothed.evaluate({ reward: 30 }));  // 0.9 * 11 + 0.1 * 30 = 12.9

reset(); // Reset smooth state
```

### 3.4 Metric Composition

#### Composition with Tracking

```typescript
import { combineMetricsWithTracking, simpleMetric } from 'symbion/core';

const metrics = [
  simpleMetric('throughput', (s) => s.throughput, 'maximize'),
  simpleMetric('energy', (s) => s.energy, 'minimize'),
  simpleMetric('latency', (s) => s.latency, 'minimize'),
];

// Create composite evaluator
const composite = combineMetricsWithTracking(metrics, 'weighted_sum');

const state = {
  throughput: 100,  // Weight default 1
  energy: 50,       // Minimize, auto-negated
  latency: 10,      // Minimize, auto-negated
};

const result = composite.evaluate(state);

console.log('Total:', result.total);
console.log('Breakdown:', result.breakdown);
// {
//   throughput: { raw: 100, weight: 0.333, weighted: 33.3, direction: 'maximize' },
//   energy: { raw: 50, weight: 0.333, weighted: -16.7, direction: 'minimize' },
//   latency: { raw: 10, weight: 0.333, weighted: -3.3, direction: 'minimize' }
// }
```

#### Different Composition Methods

```typescript
import { combineMetricsWithTracking } from 'symbion/core';

// Weighted Sum (Default)
const weighted = combineMetricsWithTracking(metrics, 'weighted_sum');

// Product
const product = combineMetricsWithTracking(metrics, 'product');

// Min
const min = combineMetricsWithTracking(metrics, 'min');

// Max
const max = combineMetricsWithTracking(metrics, 'max');
```

### 3.5 MetricRegistry

Global metric registry for managing and querying metrics.

```typescript
import { MetricRegistry, simpleMetric } from 'symbion/core';

const registry = new MetricRegistry();

// Register metrics
registry.register(simpleMetric('reward', (s) => s.reward, 'maximize'));
registry.register(simpleMetric('energy', (s) => s.energy, 'minimize'));

// Get metrics
const rewardMetric = registry.get('reward');
const energyMetric = registry.get('energy');

// List all metrics
const allIds = registry.list();
console.log('Registered Metrics:', allIds); // ['reward', 'energy']

// Filter by category
const performanceMetrics = registry.getByCategory('performance');
```

---

## 4. Constraint - Constraint System

Constraint module provides constraint definition and evaluation, distinguishing between hard constraints (must satisfy) and soft constraints (violation incurs penalty).

### 4.1 Constraint Severity

- **hard**: Hard constraint, violation makes solution infeasible
- **soft**: Soft constraint, violation allowed but penalized

### 4.2 Constraint Operators

- **le**: ≤ (Less than or equal)
- **ge**: ≥ (Greater than or equal)
- **eq**: = (Equal, within tolerance)

### 4.3 Creating Constraints

#### Basic Constraint Factory

```typescript
import { leConstraint, geConstraint, eqConstraint } from 'symbion/core';

interface State {
  speed: number;
  altitude: number;
  sinrDb: number;
}

// Max Speed Constraint: speed ≤ 20
const maxSpeed = leConstraint<State>(
  'max_speed',
  (state) => state.speed,
  20,
  'hard',
  { description: 'Max speed limit', unit: 'm/s' }
);

// Min SINR Constraint: sinrDb ≥ 0
const minSinr = geConstraint<State>(
  'min_sinr',
  (state) => state.sinrDb,
  0,
  'soft',
  { description: 'Min SNR', unit: 'dB', penaltyWeight: 10 }
);

// Target Altitude Constraint: altitude ≈ 100 (±5)
const targetAlt = eqConstraint<State>(
  'target_altitude',
  (state) => state.altitude,
  100,
  5,    // Tolerance
  'soft'
);
```

### 4.4 Domain-Specific Constraints

> **Note**: Domain-specific constraints (UAV flight, communication, etc.) have been moved to the **ISAC module** for better separation of concerns. Core module only provides **generic constraint factories**.

For domain-specific constraints, see the [ISAC Module Guide](./isac-module-guide.md#constraints):

```typescript
// Core provides generic factories
import { leConstraint, geConstraint, eqConstraint } from 'symbion/core';

// Domain-specific constraints are in ISAC module
import {
  maxSpeedConstraint,
  altitudeConstraint,
  minSinrConstraint,
  // ... other domain constraints
} from 'symbion/isac/constraints';
```

Available ISAC constraints:
- **Flight**: `maxSpeedConstraint`, `maxAccelerationConstraint`, `altitudeConstraint`, `noFlyZoneConstraint`, `energyBudgetConstraint`
- **Communication**: `minSinrConstraint`, `minDistanceConstraint`

### 4.5 Constraint Evaluation

#### Single Constraint

```typescript
import { evaluateConstraint } from 'symbion/core';
import { maxSpeedConstraint } from 'symbion/isac/constraints';

const constraint = maxSpeedConstraint(20, 'hard');
const state = { speed: 25 };

const result = evaluateConstraint(constraint, state);

console.log(result);
// {
//   id: 'max_speed',
//   value: 25,
//   threshold: 20,
//   operator: 'le',
//   severity: 'hard',
//   satisfied: false,
//   violation: 5,      // 25 - 20
//   penalty: 1e6       // Hard constraint default penalty
// }
```

#### Multiple Constraints

```typescript
import { evaluateConstraints } from 'symbion/core';
import { maxSpeedConstraint, altitudeConstraint, minSinrConstraint } from 'symbion/isac/constraints';

const constraints = [
  maxSpeedConstraint(20, 'hard'),
  altitudeConstraint(30, 150, 'hard'),
  minSinrConstraint(0, 'soft'),
];

const state = {
  speed: 18,
  position: { z: 100 },
  sinrDb: -3,
};

const report = evaluateConstraints(constraints, state);

console.log('Feasibility:', report.feasible);        // false (Soft constraint violation)
console.log('Total Penalty:', report.totalPenalty);    // 30 (From SINR)
console.log('Violations:', report.violations);
// [
//   { constraintId: 'min_sinr', value: -3, threshold: 0, severity: 'soft' }
// ]
```

### 4.6 Constraints to Penalty

Convert constraint violations to optimization penalties.

```typescript
import { constraintsToPenalty } from 'symbion/core';
import { maxSpeedConstraint, minSinrConstraint } from 'symbion/isac/constraints';

const constraints = [
  maxSpeedConstraint(20, 'hard'),
  minSinrConstraint(0, 'soft', { penaltyWeight: 10 }),
];

const state1 = { speed: 18, sinrDb: 5 };
console.log(constraintsToPenalty(constraints, state1)); // 0 (No violation)

const state2 = { speed: 25, sinrDb: 5 };
console.log(constraintsToPenalty(constraints, state2)); // 1e6 (Hard violation)

const state3 = { speed: 18, sinrDb: -3 };
console.log(constraintsToPenalty(constraints, state3)); // 30 (Soft penalty)
```

---

## 5. Runner - Experiment Runner

Runner module provides a unified experiment execution framework, integrating environment, policy, metrics, and constraints.

### 5.1 Environment Interface

Generic iterative decision environment interface. Works with optimization methods, online learning, heuristic planners, and RL agents.

```typescript
import type { Environment, Space } from 'symbion/core';

class MyEnvironment implements Environment<MyInput, MyOutput, MyInfo> {
  inputSpace: Space;   // State/observation/features space
  outputSpace: Space;  // Decision/action/prediction space

  async reset(seed?: number): Promise<{ input: MyInput; info: MyInfo }> {
    // Reset environment
    return { input, info };
  }

  async step(output: MyOutput): Promise<{
    input: MyInput;
    metric: number;     // Performance metric (reward/cost/error)
    done: boolean;
    truncated: boolean;
    info: MyInfo;
  }> {
    // Execute decision
    return { input, metric, done, truncated, info };
  }

  close(): void {
    // Cleanup resources
  }
}
```

### 5.2 DecisionMaker Interface

DecisionMaker selects outputs (decisions) given inputs (states).

```typescript
import type { DecisionMaker, SeededRandom } from 'symbion/core';

class MyDecisionMaker implements DecisionMaker<MyInput, MyOutput> {
  name = 'my-decision-maker';

  decide(input: MyInput, rng: SeededRandom): MyOutput {
    // Select output based on input
    return output;
  }

  reset(): void {
    // Reset internal state (e.g., recurrent state, optimizer state)
  }
}
```

### 5.3 Runner Configuration

```typescript
import { Runner, createTaskConfig, ConsoleLogger, simpleMetric } from 'symbion/core';
import { maxSpeedConstraint, altitudeConstraint } from 'symbion/isac/constraints';

const config = {
  // Task Config
  taskConfig: createTaskConfig({
    taskName: 'uav-navigation',
    seed: 42,
    schemaHash: computeSchemaHash(env.inputSpace),
    hyperparams: {
      maxStepsPerEpisode: 500,
      learningRate: 0.001,
    },
  }),

  // Environment and DecisionMaker
  environment: myEnvironment,
  decisionMaker: myDecisionMaker,

  // Metrics and Constraints (Optional)
  metrics: [
    simpleMetric('metric', (s) => s.metric, 'maximize'),
    simpleMetric('energy', (s) => s.energy, 'minimize'),
  ],
  constraints: [
    maxSpeedConstraint(20),
    altitudeConstraint(30, 150),
  ],

  // State Extractor (Extract state from input and info)
  stateExtractor: (input, info) => ({
    metric: info.metric,
    energy: info.energy,
    speed: info.speed,
    position: input.position,
  }),

  // Loggers
  loggers: [
    new ConsoleLogger('info'),
  ],

  // Run Parameters
  maxStepsPerEpisode: 500,
  totalEpisodes: 100,
  stopOnConstraintViolation: false,
};

const runner = new Runner(config);
```

### 5.4 Running Experiments

```typescript
// Run complete experiment
const result = await runner.run();

console.log('Experiment Result:');
console.log(`  Total Episodes: ${result.totalEpisodes}`);
console.log(`  Total Steps: ${result.totalSteps}`);
console.log(`  Avg Episode Metric: ${result.avgEpisodeMetric.toFixed(2)}`);
console.log(`  Avg Episode Length: ${result.avgEpisodeLength.toFixed(1)}`);
console.log(`  Success Rate: ${(result.successRate * 100).toFixed(1)}%`);

if (result.objectiveBreakdown) {
  console.log('  Objective Breakdown:');
  for (const [metric, value] of Object.entries(result.objectiveBreakdown)) {
    console.log(`    ${metric}: ${value.toFixed(2)}`);
  }
}
```

### 5.5 Single Step Execution

```typescript
// Reset
const input = await runner.reset();

// Single Step Loop
for (let i = 0; i < 100; i++) {
  const stepResult = await runner.step();
  
  if (stepResult.done) {
    break;
  }
  
  console.log(`Step ${i}: Metric = ${stepResult.metric.toFixed(2)}`);
  
  if (stepResult.metrics) {
    console.log('  Metrics:', stepResult.metrics.total);
  }
  
  if (stepResult.constraints && !stepResult.constraints.feasible) {
    console.log('  Constraint Violations:', stepResult.constraints.violations);
  }
}
```

### 5.6 Episode Execution

```typescript
// Execute Single Episode
const episodeResult = await runner.runEpisode();

console.log('Episode Result:');
console.log(`  Steps: ${episodeResult.totalSteps}`);
console.log(`  Total Metric: ${episodeResult.totalMetric.toFixed(2)}`);
console.log(`  Avg Metric: ${episodeResult.avgMetric.toFixed(2)}`);
console.log(`  Success: ${episodeResult.success}`);
console.log(`  Constraint Violations: ${episodeResult.constraintViolations}`);
```

---

## 6. Logging - Logging System

Logging module provides structured logging, supporting step-level, episode-level, and report-level logs.

> **Note:** Log entries use RL-style field names (`observation`, `action`, `reward`) for backward compatibility. The `Runner` automatically maps: `input` → `observation`, `output` → `action`, `metric` → `reward`.

### 6.1 Log Entry Types

| Type | Description | Fields |
|------|-------------|--------|
| `StepLogEntry` | Step-level Log | episode, step, observation, action, reward, done, ... |
| `EpisodeLogEntry` | Episode-level Log | episode, totalSteps, totalReward, success, ... |
| `ReportLogEntry` | Report-level Log | totalEpisodes, avgReward, successRate, config, ... |

### 6.2 ConsoleLogger

Console logger (supported in both Browser and Node.js).

```typescript
import { ConsoleLogger } from 'symbion/core';

// Create Logger
const logger = new ConsoleLogger('info');

// Or with full config
const logger2 = new ConsoleLogger({
  task: 'uav-navigation',
  seed: 42,
  schemaVersion: '1.0.0',
  logSteps: true,
});

// Log Step
logger.logStep({
  task: 'uav-navigation',
  seed: 42,
  episode: 0,
  step: 10,
  observation: { position: [100, 200, 50] },
  action: { movement: [0.1, 0.2, 0] },
  reward: 1.5,
  done: false,
  truncated: false,
});

// Log Episode
logger.logEpisode({
  task: 'uav-navigation',
  seed: 42,
  episode: 0,
  totalSteps: 250,
  totalReward: 150.5,
  avgReward: 0.602,
  success: true,
});

// Log Report
logger.logReport({
  task: 'uav-navigation',
  seed: 42,
  totalEpisodes: 100,
  totalSteps: 25000,
  avgEpisodeReward: 145.2,
  avgEpisodeLength: 250,
  successRate: 0.85,
  config: { /* ... */ },
});
```

### 6.3 MemoryLogger

In-memory logger for testing and data collection.

```typescript
import { MemoryLogger } from 'symbion/core';

const logger = new MemoryLogger({ task: 'test', seed: 42 });

// Log Data
logger.logStep({ /* ... */ });
logger.logEpisode({ /* ... */ });

// Read Logs
const allLogs = logger.getLogs();
const stepLogs = logger.getLogs('step');
const episodeLogs = logger.getLogs('episode');

console.log(`Logged ${allLogs.length} entries`);

// Clear Logs
logger.clear();
```

### 6.4 MultiLogger

Combine multiple loggers.

```typescript
import { MultiLogger, ConsoleLogger, MemoryLogger } from 'symbion/core';

const logger = new MultiLogger([
  new ConsoleLogger('info'),
  new MemoryLogger({ task: 'test', seed: 42 }),
]);

// Logs to all loggers
logger.logStep({ /* ... */ });
logger.logEpisode({ /* ... */ });
```

### 6.5 Factory Functions

```typescript
import { createLogger, createBrowserLogger } from 'symbion/core';

// Auto-select based on environment
const logger = createLogger({
  task: 'uav-navigation',
  seed: 42,
  outputDir: './logs',  // Only valid in Node.js
});

// Browser specific
const browserLogger = createBrowserLogger({
  task: 'web-experiment',
  seed: 123,
  logSteps: false,  // Reduce output
});
```

---

## 7. Repro - Reproducibility

Repro module provides full reproducibility guarantees, including task configuration, seeded random number generators, and version management.

### 7.1 TaskConfig

Complete experiment configuration description.

```typescript
import { createTaskConfig, serializeTaskConfig } from 'symbion/core';

// Create Task Config
const config = createTaskConfig({
  taskName: 'uav-navigation',
  version: '1.0.0',
  seed: 42,
  schemaHash: 'abc123...',
  hyperparams: {
    maxSteps: 500,
    learningRate: 0.001,
    gamma: 0.99,
    epsilon: 0.1,
  },
  description: 'UAV Navigation Experiment',
  tags: ['uav', 'navigation', 'rl'],
});

console.log('Config:', config);
// {
//   taskName: 'uav-navigation',
//   version: '1.0.0',
//   seed: 42,
//   schemaHash: 'abc123...',
//   libraryVersion: '2.0.2',
//   createdAt: 1703520000000,
//   hyperparams: { ... },
//   description: '...',
//   tags: [...]
// }

// Serialize (Deterministic Sort)
const json = serializeTaskConfig(config);
console.log(json);
```

### 7.2 Configuration Validation

```typescript
import { validateTaskConfig, areConfigsCompatible } from 'symbion/core';

// Validate Config
const validation = validateTaskConfig(config);

if (!validation.valid) {
  console.error('Config Errors:', validation.errors);
}

if (validation.warnings.length > 0) {
  console.warn('Warnings:', validation.warnings);
}

// Check Compatibility
const config1 = createTaskConfig({ /* ... */ });
const config2 = createTaskConfig({ /* ... */ });

if (areConfigsCompatible(config1, config2)) {
  console.log('Configs Compatible, can compare results');
} else {
  console.log('Configs Incompatible, schema or version mismatch');
}
```

### 7.3 SeededRandom

Deterministic random number generator (Mulberry32 algorithm).

```typescript
import { SeededRandom, createRng } from 'symbion/core';

// Create RNG
const rng = createRng(42);

// Basic Generation
console.log(rng.random());      // [0, 1)
console.log(rng.randint(0, 10)); // [0, 10) Integer
console.log(rng.uniform(-1, 1)); // [-1, 1) Float
console.log(rng.normal(0, 1));   // Standard Normal Distribution

// Array Operations
const numbers = [1, 2, 3, 4, 5];

// Shuffle (In-place)
rng.shuffle(numbers);
console.log('Shuffle:', numbers);

// Sample (No modification)
const sampled = rng.sample([1, 2, 3, 4, 5], 3);
console.log('Sample:', sampled); // e.g.: [2, 5, 1]
```

### 7.4 State Save and Restore

```typescript
import { createRng } from 'symbion/core';

const rng = createRng(42);

// Generate some numbers
console.log(rng.random()); // 0.374...
console.log(rng.random()); // 0.628...

// Save State
const saved = rng.getState();

// Continue
console.log(rng.random()); // 0.891...

// Restore
rng.setState(saved);

// Generate again, result same
console.log(rng.random()); // 0.891...
```

### 7.5 ExperimentMetadata

```typescript
import { createExperimentMetadata } from 'symbion/core';

const metadata = createExperimentMetadata({
  taskName: 'uav-navigation',
  seed: 42,
  author: 'researcher@example.com',
  description: 'UAV Autonomous Navigation Experiment',
  tags: ['uav', 'rl'],
});

console.log(metadata);
// {
//   taskName: 'uav-navigation',
//   seed: 42,
//   author: 'researcher@example.com',
//   timestamp: 1703520000000,
//   libraryVersion: '2.0.2',
//   ...
// }
```

---

## 8. Errors - Error Handling

Errors module provides unified error types and codes, ensuring consistent exception handling.

### 8.1 ErrorCodes

Standard error code constants.

```typescript
import { ErrorCodes } from 'symbion/core';

console.log(ErrorCodes.TIMEOUT);              // 'TIMEOUT'
console.log(ErrorCodes.VALIDATION_ERROR);     // 'VALIDATION_ERROR'
console.log(ErrorCodes.CONSTRAINT_VIOLATION); // 'CONSTRAINT_VIOLATION'
console.log(ErrorCodes.SCHEMA_MISMATCH);      // 'SCHEMA_MISMATCH'
console.log(ErrorCodes.CONNECTION_ERROR);     // 'CONNECTION_ERROR'
```

### 8.2 Error Classes

#### SymbionError - Base Class

```typescript
import { SymbionError, ErrorCodes } from 'symbion/core';

const error = new SymbionError(
  ErrorCodes.VALIDATION_ERROR,
  'Invalid action value',
  { value: 100, max: 28 }
);

console.log(error.code);        // 'VALIDATION_ERROR'
console.log(error.message);     // 'Invalid action value'
console.log(error.details);     // { value: 100, max: 28 }
console.log(error.timestamp);   // 1703520000000

// Serialize
const json = error.toJSON();
```

#### Specific Error Classes

```typescript
import {
  TimeoutError,
  ProtocolError,
  ValidationError,
  ConstraintViolationError,
  SchemaMismatchError,
  ConnectionError,
  NotInitializedError,
} from 'symbion/core';

// Timeout
throw new TimeoutError('Request timed out after 30s', { requestId: '123' });

// Protocol Error
throw new ProtocolError('Invalid message format');

// Validation Error
throw new ValidationError('Action out of bounds', { value: 30, max: 28 });

// Constraint Violation
throw new ConstraintViolationError('Constraints violated', [
  { constraintId: 'max_speed', value: 25, threshold: 20, severity: 'hard' },
]);

// Schema Mismatch
throw new SchemaMismatchError('expected-hash', 'actual-hash');

// Connection Error
throw new ConnectionError('WebSocket disconnected');

// Not Initialized
throw new NotInitializedError();
```

### 8.3 Error Utilities

```typescript
import { isSymbionError, hasErrorCode, wrapError, ErrorCodes } from 'symbion/core';

try {
  // Code that might throw
} catch (error) {
  // Check if SymbionError
  if (isSymbionError(error)) {
    console.log('Error Code:', error.code);
  }
  
  // Check specific code
  if (hasErrorCode(error, ErrorCodes.TIMEOUT)) {
    console.log('Request Timeout');
  }
  
  // Wrap arbitrary error
  const wrapped = wrapError(error);
  console.log(wrapped.code); // 'INTERNAL_ERROR' (Default)
}
```

### 8.4 Timeout Utilities

```typescript
import { withTimeout, createTimeout } from 'symbion/core';

// Add timeout to Promise
async function fetchData() {
  const promise = fetch('https://api.example.com/data');
  
  try {
    const result = await withTimeout(promise, 5000, 'Data fetch timeout');
    return result;
  } catch (error) {
    if (hasErrorCode(error, ErrorCodes.TIMEOUT)) {
      console.error('Request Timeout');
    }
    throw error;
  }
}

// Create delayed timeout
const timeout = createTimeout(1000, 'Operation timeout');
await Promise.race([
  someOperation(),
  timeout,
]);
```

---

## 9. Full Example

Below is a complete end-to-end example demonstrating all modules working together.

```typescript
import {
  // Space
  dict, box, discrete, sample, computeSchemaHash,
  // Objective
  simpleMetric, combineMetricsWithTracking,
  // Constraint evaluation (core provides generic evaluation)
  evaluateConstraints,
  // Runner
  Runner, type Environment, type DecisionMaker,
  // Logging
  ConsoleLogger, MemoryLogger, MultiLogger,
  // Repro
  createTaskConfig, createRng, type SeededRandom,
} from 'symbion/core';

// Domain-specific constraints from ISAC module
import { maxSpeedConstraint, altitudeConstraint } from 'symbion/isac/constraints';

// ========== 1. Define Space ==========
const inputSpace = dict({
  position: box([3], 0, 1000),
  velocity: box([3], -20, 20),
  sinrDb: box([1], -20, 40),
});

const outputSpace = dict({
  movement: box([3], -1, 1),
  mcs: discrete(29),
});

// ========== 2. Implement Environment ==========
interface State {
  position: [number, number, number];
  velocity: [number, number, number];
  sinrDb: number;
  speed: number;
  metric: number;
  energy: number;
}

class UAVEnvironment implements Environment {
  inputSpace = inputSpace;
  outputSpace = outputSpace;
  
  private state: State = {
    position: [0, 0, 50],
    velocity: [0, 0, 0],
    sinrDb: 0,
    speed: 0,
    metric: 0,
    energy: 0,
  };
  private rng!: SeededRandom;
  
  async reset(seed?: number) {
    this.rng = createRng(seed ?? 42);
    this.state = {
      position: [0, 0, 50],
      velocity: [0, 0, 0],
      sinrDb: 0,
      speed: 0,
      metric: 0,
      energy: 0,
    };
    
    const input = {
      position: this.state.position,
      velocity: this.state.velocity,
      sinrDb: [this.state.sinrDb],
    };
    
    return { input, info: { ...this.state } };
  }
  
  async step(output: any) {
    // Update State
    const [dx, dy, dz] = output.movement;
    this.state.position[0] += dx * 10;
    this.state.position[1] += dy * 10;
    this.state.position[2] += dz * 5;
    
    this.state.speed = Math.sqrt(dx*dx + dy*dy + dz*dz) * 10;
    this.state.sinrDb = this.rng.uniform(-5, 20);
    
    // Calculate Metric (e.g., negative distance as reward)
    const distanceToGoal = Math.sqrt(
      (this.state.position[0] - 1000)**2 +
      (this.state.position[1] - 1000)**2
    );
    this.state.metric = -distanceToGoal / 1000;
    this.state.energy += this.state.speed * 0.1;
    
    const done = distanceToGoal < 50;
    
    const input = {
      position: this.state.position,
      velocity: this.state.velocity,
      sinrDb: [this.state.sinrDb],
    };
    
    return {
      input,
      metric: this.state.metric,
      done,
      truncated: false,
      info: { ...this.state },
    };
  }
  
  close() {
    // Cleanup
  }
}

// ========== 3. Implement DecisionMaker ==========
class RandomDecisionMaker implements DecisionMaker {
  name = 'random-decision-maker';
  
  decide(input: any, rng: SeededRandom) {
    return {
      movement: [
        rng.uniform(-1, 1),
        rng.uniform(-1, 1),
        rng.uniform(-1, 1),
      ],
      mcs: rng.randint(0, 29),
    };
  }
  
  reset() {
    // Stateless decision maker, no reset needed
  }
}

// ========== 4. Configure Experiment ==========
async function runExperiment() {
  const taskConfig = createTaskConfig({
    taskName: 'uav-navigation-demo',
    seed: 42,
    schemaHash: computeSchemaHash(inputSpace),
    hyperparams: {
      maxStepsPerEpisode: 100,
      totalEpisodes: 10,
    },
    description: 'UAV Autonomous Navigation Demo',
    tags: ['uav', 'demo'],
  });
  
  const runner = new Runner({
    taskConfig,
    environment: new UAVEnvironment(),
    decisionMaker: new RandomDecisionMaker(),
    
    // Metrics
    metrics: [
      simpleMetric('metric', (s: State) => s.metric, 'maximize'),
      simpleMetric('energy', (s: State) => s.energy, 'minimize'),
    ],
    
    // Constraints
    constraints: [
      maxSpeedConstraint<State>(20, 'soft'),
      altitudeConstraint<State>(30, 150, 'hard'),
    ],
    
    // State Extractor
    stateExtractor: (input, info) => info as State,
    
    // Loggers
    loggers: [
      new ConsoleLogger('info'),
      new MemoryLogger({ task: 'demo', seed: 42 }),
    ],
    
    maxStepsPerEpisode: 100,
    totalEpisodes: 10,
  });
  
  // Run Experiment
  const result = await runner.run();
  
  console.log('\n========== Experiment Result ==========');
  console.log(`Total Episodes: ${result.totalEpisodes}`);
  console.log(`Total Steps: ${result.totalSteps}`);
  console.log(`Avg Episode Metric: ${result.avgEpisodeMetric.toFixed(2)}`);
  console.log(`Avg Episode Length: ${result.avgEpisodeLength.toFixed(1)}`);
  console.log(`Success Rate: ${(result.successRate * 100).toFixed(1)}%`);
  
  if (result.objectiveBreakdown) {
    console.log('\nObjective Breakdown:');
    for (const [key, value] of Object.entries(result.objectiveBreakdown)) {
      console.log(`  ${key}: ${value.toFixed(2)}`);
    }
  }
  
  runner.close();
}

// Run
runExperiment().catch(console.error);
```

---

## Related Links

- [API Reference](../API_REFERENCE.md)
- [AI Module Guide](./ai-module-guide.md)
- [ISAC Module Guide](./isac-module-guide.md)
- [Example Code Repository](https://github.com/symbion-io/symbion-examples)

---

*Document Version: 1.1 | Last Updated: 2025-12-29*
