# Symbion API Reference

Complete API reference for the Symbion library - an Integrated Sensing and Communication (ISAC) algorithm library for research and development.

## Quick Start

```typescript
// Option 1: Full library import
import symbion from 'symbion';

// Option 2: Module-specific imports (recommended)
import { createRng, createRunner } from 'symbion/core';
import { channel, modulation, planning } from 'symbion/models';
import { u2uMcs, isacTrajectory } from 'symbion/tasks';
```

## Module Overview

| Module | Import Path | Description |
|--------|-------------|-------------|
| **Core** | `symbion/core` | Infrastructure: Space, Objective, Constraint, Runner, Logging, Repro, Errors |
| **Models** | `symbion/models` | Physics: Channel, Beamforming, Modulation, Planning, Sensing, Robotics |
| **Tasks** | `symbion/tasks` | Research tasks: U2U-MCS, ISAC Trajectory |
| **AI** | `symbion/ai` | Framework-agnostic AI interface (typed spaces + WebSocket protocol) |
| **ISAC** | `symbion/isac` | ISAC joint optimization framework |
| **Extras** | `symbion/extras` | Experimental: Networking protocols |

---

## Table of Contents

1. [Core Module](#1-core-module)
2. [Tasks Module](#2-tasks-module)
3. [AI Interface](#3-ai-interface-ai)
4. [ISAC Framework](#4-isac-framework-isac)
5. [Channel Modeling](#5-channel-modeling-channel)
6. [Beamforming](#6-beamforming-beamforming)
7. [Modulation](#7-modulation-modulation)
8. [Channel Coding](#8-channel-coding-coding)
9. [Planning](#9-planning-planning)
10. [Trajectory](#10-trajectory-trajectory)
11. [Control](#11-control-control)
12. [Robotics & Dynamics](#12-robotics--dynamics-robotics)
13. [Sensing](#13-sensing-sensing)
14. [Math Utilities](#14-math-utilities-math)
15. [Optimization](#15-optimization-optimization)
16. [Utilities](#16-utilities-utils)
17. [Extras](#17-extras)

---

## 1. Core Module

>  **Detailed Guide:** [Core Module Guide](./docs/guides/core-module-guide.md)

Infrastructure layer for reproducible experiments.

### Space Definition

```typescript
import { BoxSpace, DiscreteSpace, createSpace } from 'symbion/core';
```

- **`BoxSpace`**: Continuous space with bounds (e.g., input/output vectors)
- **`DiscreteSpace`**: Discrete input/output space
- **`MultiDiscreteSpace`**: Multiple discrete dimensions
- **`DictSpace`**: Nested dictionary of spaces
- **`TupleSpace`**: Ordered tuple of spaces
- **`createSpace(config)`**: Factory function for space creation

### Objective System

```typescript
import { createObjective, weightedSum, combineMetrics } from 'symbion/core';
```

- **`createObjective(fn, meta)`**: Creates objective function with metadata
- **`weightedSum(objectives, weights)`**: Combines objectives with weights
- **`MetricSpec`**: Metric definition interface (direction, unit, description)

### Constraint System

```typescript
import { createConstraint, checkConstraints } from 'symbion/core';
```

- **`createConstraint(fn, meta)`**: Creates constraint function
- **`checkConstraints(constraints, state)`**: Evaluates all constraints
- **`ConstraintResult`**: Constraint evaluation result (satisfied, violation)

> **Note**: Core provides generic constraint factories (`leConstraint`, `geConstraint`, `eqConstraint`). Domain-specific constraints (UAV, communication) are in the [ISAC module](#4-isac-framework-isac).

### Runner

```typescript
import { createRunner, EpisodeRunner, BatchRunner } from 'symbion/core';
```

- **`EpisodeRunner`**: Single episode execution
- **`BatchRunner`**: Multi-episode parallel execution
- **`Policy<O, A>`**: Policy interface (input → output)
- **`Environment<O, A, I>`**: Environment interface (step/reset pattern)

### Logging

```typescript
import { ConsoleLogger, JsonlLogger, CsvLogger } from 'symbion/core';
```

- **`ConsoleLogger`**: Console output logger
- **`JsonlLogger`**: JSONL file logger (for reproducibility)
- **`CsvLogger`**: CSV file logger (for analysis)

### Reproducibility

```typescript
import { createRng, SeededRandom } from 'symbion/core';
```

- **`createRng(seed)`**: Creates seeded random number generator
- **`SeededRandom`**: RNG class with `random()`, `uniform(a,b)`, `randint(a,b)`, `choice(arr)`, `shuffle(arr)`

### Error Handling

```typescript
import { SymbionError, ValidationError, ConfigError } from 'symbion/core';
```

- **`SymbionError`**: Base error class
- **`ValidationError`**: Input validation errors
- **`ConfigError`**: Configuration errors
- **`TimeoutError`**: Operation timeout
- **`ProtocolError`**: Protocol violation
- **`ConstraintViolationError`**: Constraint check failure
- **`ErrorCodes`**: Standard error codes enum

---

## 2. Tasks Module

>  **Detailed Guide:** [Tasks User Guide](./docs/tasks-user-guide.md)

Research-oriented simulation tasks.

### U2U-MCS Task

```typescript
import { u2uMcs } from 'symbion/tasks';

// Usage via namespace
const env = u2uMcs.createEnvironment(config);
const policy = new u2uMcs.GreedyMcsPolicy(config);
```

- **`createEnvironment(config)`**: Creates U2U-MCS environment
- **`U2UMcsEnvironment`**: Iterative Decision MCS selection environment
- **Policies**: `FixedMcsPolicy`, `RandomMcsPolicy`, `GreedyMcsPolicy`, `BlerAdaptivePolicy`, `OllaPolicy`
- **Configs**: `DEFAULT_U2U_MCS_CONFIG`, `PAPER_SCENARIO_CONFIG`

### ISAC Trajectory Task

```typescript
import { isacTrajectory } from 'symbion/tasks';

// Usage via namespace
const config = isacTrajectory.createConfig({});
const env = isacTrajectory.IsacTrajectoryEnvironment(config);
```

- **`IsacTrajectoryEnvironment`**: Gym-compatible trajectory environment
- **`createConfig(overrides)`**: Configuration factory
- **`runTask1(config)`**: Direct optimization (MINCO + L-BFGS)
- **Policies**: `HoverPolicy`, `RandomPolicy`, `RateOnlyPolicy`, `EnergyEfficientPolicy`, `ProposedPolicy`

---

## 3. AI Interface (ai)

> **Detailed Guide:** [AI Module Guide](./docs/guides/ai-module-guide.md)

Framework-agnostic interface connecting Symbion with external AI backends (Python RL agents, LLMs, etc.).

### Core Classes

```typescript
import { 
  AiInterfaceClient,      // One-shot AI inference (query/batch/stream)
  IterativeAiClient,      // Iterative AI interactions (step/reset)
  InMemoryTransport,      // In-memory transport for testing
  FakeAgentServer,        // Mock AI agent server for testing
} from 'symbion/ai';
```

- **`AiInterfaceClient`**: Base client for one-shot AI inference
- **`IterativeAiClient`**: Client for iterative step/reset interactions
- **`UavGymEnvClient`**: Legacy alias for `IterativeAiClient`
- **`InMemoryTransport`**: In-memory transport for unit testing
- **`FakeAgentServer`**: Mock AI Agent server for testing

### Space Types

```typescript
import type { Space, BoxSpace, DiscreteSpace, DictSpace } from 'symbion/ai';
```

- **`Space`**: Union type of all space definitions
- **`BoxSpace`**: Continuous multi-dimensional space
- **`DiscreteSpace`**: Discrete integer space
- **`DictSpace`**: Dictionary of nested spaces
- **`TupleSpace`**: Ordered tuple of spaces

### Input/Output Utilities

```typescript
import { 
  defaultUavObservation,
  normalizeObservation,
  clampAction,
  findOptimalMcs,
} from 'symbion/ai';
```

- **`defaultUavObservation(state)`**: Extracts standard UAV input vector
- **`minimalUavObservation(state)`**: Core motion state inputs
- **`normalizeObservation(obs, space)`**: Normalize to `[-1, 1]`
- **`clampAction(action, space)`**: Clamp output to valid range
- **`defaultUavActionMapper(action)`**: Maps AI outputs to UAV commands
- **`findOptimalMcs(snrDb, blerTarget)`**: Find optimal MCS index

---

## 4. ISAC Framework (isac)

>  **Detailed Guide:** [ISAC Module Guide](./docs/guides/isac-module-guide.md)

Integrated Sensing and Communication framework.

### Core

- **`ISACPlanner`**: Joint trajectory optimizer (MINCO + L-BFGS)
- **`ISACEnvAdapter`**: Adapter to Gym environments
- **`MetricRegistry`**: Optimization metric registry

### Metrics

- **`energyEfficiencyMetric`**: Energy efficiency (bits/Joule)
- **`weightedSumRateMetric`**: Weighted sum rate
- **`sensingCoverageMetric`**: Sensing coverage

### Constraints

- **`maxSpeedConstraint(limit)`**: Maximum speed constraint
- **`noFlyZoneConstraint(zones)`**: No-fly zone constraint
- **`minSinrConstraint(limit)`**: Minimum SINR constraint

### Policies

- **`WaypointFollowerPolicy`**: Waypoint following
- **`GreedyCommPolicy`**: Greedy communication (adaptive MCS/power)
- **`SensingAwarePolicy`**: Sensing-aware motion

---

## 5. Channel Modeling (channel)

>  **Detailed Guide:** [Models Module Guide](./docs/guides/models-module-guide.md)

### AWGN

```typescript
import { awgn, shannonCapacity, addNoise } from 'symbion/models';
```

- **`awgn.calculate(params)`**: Compute all AWGN parameters
- **`shannonCapacity(snrDb, bandwidth)`**: Shannon capacity formula
- **`addNoise(signal, snrDb)`**: Add Gaussian noise
- **`requiredSnr(capacity, bandwidth)`**: Required SNR for target capacity

### Two-Ray Ground Reflection

- **`breakpointDistance(txH, rxH, wavelength)`**: Calculate breakpoint distance
- **`twoRay.calculate(params)`**: Two-ray path loss calculation
- **`twoRay.powerVsDistance(...)`**: Generate power vs distance curve

### Fading

- **`rayleighFading()`**: Rayleigh fading coefficients (NLOS)
- **`ricianFading(params)`**: Rician fading coefficients (LOS)
- **`dopplerShift(velocity, freq, angle)`**: Doppler frequency shift
- **`coherenceTime(maxDoppler)`**: Channel coherence time

### GSCM

- **`initScatterChannel(config)`**: Initialize scatter-based channel
- **`updateScatterChannel(state, time)`**: Update channel state
- **`getFrequencyResponse(state, freqs)`**: Channel frequency response

### MIMO

- **`generateMIMOChannel(tx, rx, type)`**: Generate MIMO channel matrix

---

## 6. Beamforming (beamforming)

### Array Processing

- **`arrayFactor(theta, phi, N, d, wavelength)`**: Array factor computation
- **`arrayPattern(config)`**: Antenna array radiation pattern
- **`halfPowerBeamwidth(N)`**: Half-power beamwidth (HPBW)
- **`steeringPhases(theta, phi, positions, wavelength)`**: Beamforming phase weights

### MIMO & Polarization

- **`mimoCapacity(H, snr)`**: MIMO channel capacity
- **`zfSinr(H, precoder)`**: Zero-Forcing SINR
- **`mrtSinr(H, precoder)`**: MRT SINR
- **`polarizationLossFactor(polTX, polRX)`**: Polarization mismatch loss

---

## 7. Modulation (modulation)

### Digital

- **`qam.modulate(bits, M)`**: QAM modulation (BPSK, QPSK, 16QAM, ...)
- **`qam.demodulate(symbols, M)`**: QAM demodulation
- **`generateConstellation(M)`**: Standard constellation points

### Analog

- **`amModulate(signal, carrier)`**: Amplitude Modulation
- **`fmModulate(signal, carrier)`**: Frequency Modulation

### OFDM/OTFS

```typescript
import { phy } from 'symbion/models';
const { otfsModulate, otfsDemodulate } = phy.modulation;
```

- **`otfsModulate(bits, params?)`**: OTFS modulation (ISFFT + Heisenberg transform)
- **`otfsDemodulate(signal, params?)`**: OTFS demodulation (Wigner + SFFT)
- **`simulateOTFSvsOFDM(snrDb, channel, params?)`**: OTFS vs OFDM BER comparison

### Pulse Shaping

- **`rootRaisedCosine(t, alpha)`**: Root raised cosine filter

---

## 8. Channel Coding (coding)

### Hamming

- **`hammingEncode(data)`**: Hamming code encoding
- **`hammingDecode(received)`**: Hamming decoding with error correction

### CRC

- **`crcCalculate(data, poly)`**: Cyclic Redundancy Check
- **`crcCalculateSteps(...)`**: Detailed CRC calculation steps

### Convolutional

- **`convolutionalEncode(data)`**: Convolutional encoding
- **`viterbiDecode(received)`**: Viterbi decoding

### Turbo

- **`turboEncode(data)`**: Turbo code encoding (PCCC)
- **`turboDecode(sys, par1, par2, iter)`**: Turbo iterative decoding

### LDPC

- **`makeH(n, k)`**: LDPC parity-check matrix construction
- **`ldpcEncode(data, G)`**: LDPC encoding
- **`ldpcDecode(received, H)`**: LDPC decoding (Sum-Product)

### Spreading (phy.spreading)

- **`mSequence(taps, length)`**: m-sequence (PN code)
- **`goldSequence(degree, shift)`**: Gold code
- **`walshCode(index, length)`**: Walsh-Hadamard code
- **`dsssSpread(data, code)`**: DSSS spreading
- **`frequencyHoppingPattern(...)`**: Frequency hopping

---

## 9. Planning (planning)

### Trajectory Generation (MINCO)

```typescript
import { MinimumJerkTrajectory, MinimumSnapTrajectory } from 'symbion/models';
```

- **`MinimumJerkTrajectory`**: MINCO minimum jerk (s=2)
- **`MinimumSnapTrajectory`**: MINCO minimum snap (s=3)
- **`minco.propagateGrad(...)`**: Gradient propagation

### Global Planning

- **`astar(start, goal, grid)`**: A* path search
- **`rrtStar(start, goal, obstacles)`**: RRT* sampling

### Corridor Generation

- **`firi(path, obstacles)`**: FIRI Safe Flight Corridor
- **`inflatePolytope(...)`**: Polytope inflation

### Geometry

- **`quickHull(points)`**: 3D convex hull
- **`Hyperplane`**: Hyperplane operations

---

## 10. Trajectory (trajectory)

- **`Polynomial`**: Single-dimension polynomial evaluation
- **`PiecewiseTrajectory`** (Interface): Polynomial trajectory container
    - `pieces`: Array of polynomial pieces
- **`Vector3`**: `{x, y, z}` coordinate interface
- **`TrajectoryState`**: `{position, velocity, acceleration, jerk}`

---

## 11. Control (control)

### Differential Flatness

- **`FlatnessMap`**: Maps trajectory state to control inputs
    - `forward(state, yaw)`: Forward mapping
    - `backward(...)`: Backward propagation for gradients

### Controllers

- **`CascadeController`**: Cascade PID controller
- **`GeometricController`**: SE(3) geometric tracking

---

## 12. Robotics & Dynamics (robotics)

### Drone Utilities

```typescript
import { robotics } from 'symbion/models';
const { drone } = robotics;
```

- **`normalizeAngle(angle)`**: Normalize angle to [-π, π]
- **`distance(p1, p2)`**: Euclidean distance (2D)
- **`goalReached(pose, goal, threshold)`**: Check goal arrival
- **`gaussianRandom()`**: Standard normal distribution generator

### Dynamics

- **`QuadrotorDynamics`**: Quadrotor UAV dynamics
- **`DifferentialDrive`**: Differential drive kinematics
- **`applyAerodynamics(state)`**: Aerodynamic drag
- **`rayCircleIntersection(...)`**: Ray-circle intersection

---

## 13. Sensing (sensing)

### Perception

- **`simulateLiDAR(pose, obstacles)`**: 2D LiDAR simulation
- **`isInFOV(pose, target)`**: Field of View detection
- **`rayCircleIntersection(...)`**: Ray-circle intersection

### Mapping

- **`OccupancyGrid`**: Occupancy grid map

### Environment Generation

- **`generateRandomObstacles(...)`**: Create random circle/rect obstacles
- **`generateRandomPolygonObstacles(...)`**: Create random polygons
- **`generateCircleDistribution(n, cx, cy, r)`**: Create circular robot formation
- **`createSimpleGridMap(...)`**: Create empty grid map with walls
- **`createMazeGridMap(...)`**: Create random maze map

---

## 14. Math Utilities (math)

Lightweight linear algebra (no external dependencies).

- **`vec3`**: 3D vector ops (`add`, `sub`, `scale`, `dot`, `cross`, `norm`)
- **`mat3`**: 3x3 matrix ops (`mul`, `transpose`, `det`, `cholesky`)
- **`lbfgs`**: L-BFGS optimization algorithm

---

## 15. Optimization (optimization)

- **`lbfgsOptimize(x0, evaluate)`**: L-BFGS unconstrained optimization
- **`minimize(costFn, x0)`**: Simplified optimization interface
- **`sdlp(constraints, objective)`**: Seidel LP algorithm
- **`lbfgsStatusMessage(status)`**: Optimization status string

---

## 16. Utilities (utils)

### Conversion

- **`linearToDb(val)` / `dbToLinear(val)`**: dB conversion
- **`wattsToDbm(watts)` / `dbmToWatts(dbm)`**: Power conversion
- **`frequencyToWavelength(freq)`**: Frequency to wavelength

### Statistics

- **`gaussianRandom(mean, std)`**: Gaussian random
- **`qFunction(x)`**: Q-function
- **`complexGaussianRandom(...)`**: Complex Gaussian random

### Coordinate Transforms

- **`threeToPhysics(vec)` / `physicsToThree(vec)`**: Coordinate conversion
- **`buildOccupancyGridFromThreeJS(...)`**: Build grid from Three.js scene

### Debugging

- **`PhysicsDebugger`**: Physics state visualization

---

## 17. Extras

>  **Experimental modules** - API may change

### Networking

```typescript
import { networking } from 'symbion/extras';
```

- **Protocols**: TDMA, CSMA/CA
- **Routing**: AODV, Dijkstra, Flooding

### Sandbox2d

- **`sandbox2d`**: 2D simulation sandbox (Educational/Experimental)

---

## Additional Resources

- **[README](./README.md)** - Project overview
- **[Contributing Guide](./CONTRIBUTING.md)** - How to contribute
- **[Module Guides](./docs/guides/README.md)** - Detailed documentation
- **[Tasks User Guide](./docs/tasks-user-guide.md)** - Research tasks guide

---

*Generated: 2025-12-27 | Version 1.0.0*
