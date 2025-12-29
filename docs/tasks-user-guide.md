# Symbion Tasks User Guide

This guide provides comprehensive documentation for the two research-oriented simulation tasks included in the Symbion library: **U2U-MCS** (UAV-to-UAV Modulation and Coding Scheme Selection) and **ISAC Trajectory** (Integrated Sensing and Communication Trajectory Optimization).

---

## Table of Contents

1. [Overview](#overview)
2. [Installation & Setup](#installation--setup)
3. [Quick Start](#quick-start)
4. [U2U-MCS Task](#u2u-mcs-task)
   - [Task Description](#task-description)
   - [Configuration](#u2u-mcs-configuration)
   - [Input Space](#u2u-mcs-observation-space)
   - [Output Space](#u2u-mcs-action-space)
   - [Reward Function](#u2u-mcs-reward-function)
   - [Baseline Policies](#u2u-mcs-baseline-policies)
   - [Usage Examples](#u2u-mcs-usage-examples)
   - [Running Benchmarks](#u2u-mcs-benchmarks)
   - [API Reference](#u2u-mcs-api-reference)
5. [ISAC Trajectory Task](#isac-trajectory-task)
   - [Task Description](#isac-task-description)
   - [Configuration](#isac-trajectory-configuration)
   - [Input Space](#isac-trajectory-observation-space)
   - [Output Space](#isac-trajectory-action-space)
   - [Reward Function](#isac-trajectory-reward-function)
   - [Baseline Policies](#isac-trajectory-baseline-policies)
   - [Usage Examples](#isac-trajectory-usage-examples)
   - [Running Benchmarks](#isac-trajectory-benchmarks)
   - [API Reference](#isac-trajectory-api-reference)
6. [Advanced Topics](#advanced-topics)
---

## Overview

The `src/tasks` module provides two research-oriented simulation tasks for wireless communication and UAV systems:

| Task | Description | Problem Type |
|------|-------------|--------------|
| **U2U-MCS** | 5G NR Sidelink MCS adaptive selection | Sequential decision-making |
| **ISAC Trajectory** | UAV trajectory optimization with LoS awareness | Constrained optimization |

Both tasks provide **standardized simulation interfaces** with step/reset patterns for iterative algorithms, suitable for:
- Numerical optimization methods
- Online learning approaches
- Heuristic planners
- External decision-making systems

### Key Features

- **Reproducibility**: All simulations support seeded random number generation
- **Standardized Interface**: Step/reset pattern for iterative decision-making
- **Rich Logging**: JSONL and CSV output formats for analysis
- **Multiple Baselines**: Pre-implemented baseline algorithms for comparison
- **Configurable**: Extensive configuration options for different scenarios



---

## Installation & Setup

### Prerequisites

- Node.js 18+ 
- TypeScript 5+
- npm or pnpm

### Installation

```bash
# Clone the repository
git clone https://github.com/YUJX19/Symbion.git
cd Symbion

# Install dependencies
npm install

# Build the project
npm run build
```

### Running Examples

```bash
# Run U2U-MCS simple example
npx tsx benchmarks/u2u-mcs/simple-example.ts

# Run ISAC Trajectory simple example
npx tsx benchmarks/isac-los-urllc/simple-example.ts
```

---

## Quick Start

### U2U-MCS Quick Example

```typescript
import { u2uMcs } from 'symbion/tasks';
import { createRng } from 'symbion/core';

async function main() {
    // Create environment with paper scenario config
    const env = u2uMcs.createEnvironment(u2uMcs.PAPER_SCENARIO_CONFIG);
    const policy = new u2uMcs.GreedyMcsPolicy(u2uMcs.PAPER_SCENARIO_CONFIG);
    const rng = createRng(42);

    // Reset environment
    const { input, info } = await env.reset(42);
    policy.reset();

    // Run one episode
    let done = false;
    let totalReward = 0;
    let currentInput = input;

    while (!done) {
        // Convert input vector to structured input
        const inputState = {
            sinrDb: currentInput[0] * 30,
            distance: currentInput[1] * 200,
            relativeSpeed: currentInput[2] * 30,
            recentAckRate: currentInput[3],
            recentBler: currentInput[4],
            lastMcs: Math.round(currentInput[5] * 22),
            normalizedStep: currentInput[6],
        };

        // Make decision and step
        const output = policy.decide(inputState, rng);
        const result = await env.step(output);
        
        currentInput = result.input;
        totalReward += result.info.rewardBreakdown.total;
        done = result.info.step >= u2uMcs.PAPER_SCENARIO_CONFIG.episodeLength;
    }

    console.log(`Total Reward: \${totalReward}`);
}

main().catch(console.error);
```

---

## U2U-MCS Task

### Task Description

The **U2U-MCS** (UAV-to-UAV MCS Selection) task implements a 5G NR Sidelink dynamic Modulation and Coding Scheme (MCS) selection problem. This is formulated as a **sequence prediction task** where the goal is to select optimal MCS indices at each time step to maximize throughput while meeting URLLC constraints.

**Problem Setting:**
- Two UAVs communicate via 5G NR Sidelink
- The channel quality varies due to mobility and fading
- MCS index (0-22) must be selected at each TTI (1ms)
- Higher MCS = higher throughput but higher BLER risk

**Solution Approaches:**
1. **LSTM-based sequence prediction**: Predict optimal MCS based on SINR history (online supervised learning)
2. **Heuristic methods**: SINR-based greedy selection, OLLA (3GPP standard), BLER-adaptive
3. **Fixed baselines**: Conservative or aggressive fixed MCS selection

**Key Objectives:**
1. **Maximize throughput** by selecting highest feasible MCS
2. **Maintain reliability** by keeping BLER below target (e.g., 1e-5)

3. **Minimize switching costs** (optional) to reduce signaling overhead

### U2U-MCS Configuration

The task is configured via `U2UMcsConfig`:

```typescript
interface U2UMcsConfig {
    // Simulation
    seed: number;                          // Random seed for reproducibility
    episodeLength: number;                 // Steps per episode (default: 1000)
    stepDurationMs: number;                // TTI duration (default: 1ms)
    
    // URLLC Target
    blerTarget: number;                    // Target BLER (default: 1e-5)
    
    // MCS Table
    mcsTableId: 'nr-table1' | 'nr-table2'; // 5G NR MCS table
    
    // Channel Model
    channelModel: 'rician' | 'nakagami' | 'simple-pathloss';
    ricianKDb?: number;                    // Rician K-factor in dB
    
    // Mobility
    mobilityConfig: {
        relativeSpeedRange: [number, number]; // m/s
        distanceRange: [number, number];      // meters
        dynamicSpeed: boolean;
        speedChangeRate?: number;
    };
    
    // PHY Parameters
    bandwidthHz: number;                   // Bandwidth (default: 10 MHz)
    txPowerDbm: number;                    // TX power (default: 23 dBm)
    noiseFigureDb: number;                 // Noise figure (default: 9 dB)
    numRBs: number;                        // Number of resource blocks
    
    // Reward
    rewardWeights: {
        throughput: number;                 // Throughput reward weight
        blerPenalty: number;                // BLER penalty weight
        switchPenalty?: number;             // MCS switch penalty
    };
}
```

#### Pre-defined Configurations

```typescript
// Default configuration
import { u2uMcs } from 'symbion/tasks';
const defaultConfig = u2uMcs.DEFAULT_U2U_MCS_CONFIG;

// Paper scenario (28 GHz, 100 MHz, Urban Micro)
const paperConfig = u2uMcs.PAPER_SCENARIO_CONFIG;
```

### U2U-MCS Input Space

The input (observation) is a 7-dimensional normalized vector:

| Index | Name | Description | Range |
|-------|------|-------------|-------|
| 0 | `sinrDb / 30` | Current SINR (normalized) | ~[-1, 2] |
| 1 | `distance / 200` | Distance to peer UAV | [0, 1] |
| 2 | `relativeSpeed / 30` | Relative speed | [0, 1] |
| 3 | `recentAckRate` | Recent ACK rate (last 100 packets) | [0, 1] |
| 4 | `recentBler` | Recent BLER | [0, 1] |
| 5 | `lastMcs / 22` | Last selected MCS (normalized) | [0, 1] |
| 6 | `normalizedStep` | Episode progress | [0, 1] |

### U2U-MCS Output Space

**Discrete Output Space**: Integer in range `[0, 22]`

| MCS Index | Modulation | Spectral Efficiency | Required SINR (dB) |
|-----------|------------|--------------------|--------------------|
| 0 | QPSK | 0.23 | -6.7 |
| 5 | QPSK | 0.89 | 2.6 |
| 10 | 64QAM | 2.41 | 11.8 |
| 15 | 64QAM | 3.90 | 21.1 |
| 20 | 256QAM | 5.12 | 26.0 |
| 22 | 256QAM | 5.89 | 28.0 |

### U2U-MCS Reward Function

The reward function balances throughput and reliability:

```
R = w_thr * (throughput / max_throughput) 
    - w_bler * max(0, BLER - BLER_target)
    - w_switch * I(MCS_changed)
```

Where:
- `w_thr`: Throughput reward weight (default: 1.0)
- `w_bler`: BLER penalty weight (default: 100.0)
- `w_switch`: MCS switch penalty (default: 0.1)

### U2U-MCS Baseline Policies

The task includes 5 baseline policies:

| Policy | Description | Use Case |
|--------|-------------|----------|
| `fixed-N` | Always select MCS=N | Conservative (N=3) or aggressive (N=20) |
| `random` | Random MCS selection | Random baseline |
| `greedy` | Select highest MCS meeting BLER target | SINR-based heuristic |
| `bler-adaptive` | Adjust MCS based on recent BLER | Reactive approach |
| `olla` | Outer Loop Link Adaptation | Standard 3GPP algorithm |

#### Using Policies

```typescript
import { u2uMcs, tasks } from 'symbion/tasks';

// List available policies
console.log(u2uMcs.getAvailablePolicies());

// Create policy by name
const policy = u2uMcs.createPolicy('greedy', config);

// Create policy directly
const fixedPolicy = new u2uMcs.FixedMcsPolicy(10);
const greedyPolicy = new u2uMcs.GreedyMcsPolicy(config);
const ollaPolicy = new u2uMcs.OllaPolicy(config, 0.1);
```

### U2U-MCS Usage Examples

#### Example 1: Basic Environment Usage

```typescript
import { u2uMcs } from 'symbion/tasks';

async function basicExample() {
    // Create environment
    const env = u2uMcs.createEnvironment(u2uMcs.PAPER_SCENARIO_CONFIG);
    
    // Reset
    const { input, info } = await env.reset(42);
    console.log(`Initial SINR: ${info.sinrDb.toFixed(1)} dB`);
    
    // Step with MCS=10
    const result = await env.step(10);
    console.log(`ACK: ${result.info.ack}, Throughput: ${result.info.throughput}`);
    
    // Get episode summary after multiple steps
    const summary = env.getEpisodeSummary();
    console.log(`Avg Throughput: ${summary.avgThroughput}`);
}
```

### U2U-MCS Benchmarks

#### Running Benchmarks

```bash
# Simple example
npx tsx benchmarks/u2u-mcs/simple-example.ts

# Run with specific config
npx tsx benchmarks/u2u-mcs/run-baseline.ts --config=baseline-high-speed.json --policy=greedy
```

### U2U-MCS API Reference

#### Core Functions

| Function | Description |
|----------|-------------|
| `createEnvironment(config)` | Create U2U-MCS environment |
| `createPolicy(name, config)` | Create baseline policy by name |
| `getAvailablePolicies()` | Get list of available policies |
| `getMcsEntry(tableId, index)` | Get MCS table entry |
| `calculateBler(sinrDb, mcsEntry)` | Calculate BLER for given SINR and MCS |

---

## ISAC Trajectory Task

### ISAC Task Description

The **ISAC Trajectory** (Integrated Sensing and Communication) task is a **constrained trajectory optimization problem** that generates UAV flight paths considering:

- **Line-of-Sight (LoS) Persistence**: Maintain LoS to ground users
- **URLLC Constraints**: Ultra-reliable low-latency communication requirements
- **Sensing-Communication Trade-off**: Balance sensing coverage and communication throughput

**Solution Approaches:**
1. **MINCO + L-BFGS optimization**: Direct trajectory optimization using minimum-jerk trajectories (task1.ts)
2. **Heuristic planners**: Greedy, energy-efficient, multi-objective rule-based planning (policies.ts)
3. **Simulation environments**: For evaluating different planning approaches (environment.ts)

**Key Objectives:**
1. **Maximize throughput** to all users
2. **Maintain LoS persistence** (>70% target)
3. **Satisfy URLLC constraints** (<5% violation rate)
4. **Minimize energy consumption**
5. **Maintain sensing coverage** (>50% target)

### ISAC Trajectory Configuration

```typescript
interface IsacTrajectoryConfig {
    // Simulation
    seed: number;
    numWaypoints: number;           // Number of trajectory waypoints
    waypointDurationS: number;      // Duration per waypoint (seconds)
    episodeHorizon: number;         // Steps per episode
    
    // Area
    areaBounds: [number, number, number, number]; // [minX, minY, maxX, maxY]
    startPosition: Position3D;
    endPosition?: Position3D;
    
    // Users
    users: GroundUserConfig[];
    
    // Obstacles
    obstacles: ObstacleConfig[];
    
    // UAV Kinematics
    kinematics: {
        maxSpeed: number;           // m/s
        maxAcceleration: number;    // m/s^2
        minAltitude: number;        // meters
        maxAltitude: number;        // meters
    };
    
    // Communication
    comm: {
        carrierFrequencyHz: number;
        bandwidthHz: number;
        txPowerDbm: number;
    };
    
    // Reward
    rewardWeights: IsacRewardWeights;
    urllcBlerTarget: number;
}
```

### ISAC Trajectory Input Space

The input is a variable-dimension vector based on the number of users:

```typescript
interface IsacInput {
    normalizedPosition: number[];    // UAV position (3D, normalized to [-1, 1])
    normalizedVelocity: number[];    // UAV velocity (3D, normalized)
    losStatus: number[];             // LoS status per user (0 or 1)
    normalizedSinr: number[];        // SINR per user (normalized)
    normalizedDistances: number[];   // Distance to each user (normalized)
    losPercentage: number;           // Overall LoS percentage
    urllcSatisfied: number[];        // URLLC satisfaction status
    normalizedEnergy: number;        // Energy used (normalized)
    progress: number;                // Episode progress [0, 1]
}
```

### ISAC Trajectory Output Space

**Continuous Output Space**: 3D position offset `[dx, dy, dz]` in range `[-1, 1]`

**Discrete Output Space** (alternative): 27 discrete directions (3x3x3 grid including hover)

### ISAC Trajectory Reward Function

The reward balances multiple objectives:

```
R = w_thr * (throughput / maxThroughput)
    + w_los * losPercentage
    - w_energy * (stepEnergy / maxStepEnergy)
    - w_urllc * (urllcViolations / numUrllcUsers)
    - w_sensing * max(0, sensingTarget - sensingCoverage)
```

### ISAC Trajectory Baseline Policies

| Policy | Description | Strategy |
|--------|-------------|----------|
| `hover` | Stay at initial position | Baseline energy efficiency |
| `random` | Random movement direction | Exploration baseline |
| `rate-only` | Move toward user with lowest SINR | Greedy throughput |
| `energy-efficient` | Minimize energy while maintaining coverage | Energy-focused |
| `proposed` | Balance LoS, throughput, energy, and URLLC | Multi-objective |

### ISAC Trajectory Benchmarks

#### Running Benchmarks

```bash
# Simple example
npx tsx benchmarks/isac-los-urllc/simple-example.ts

# Run with specific planner
npx tsx benchmarks/isac-los-urllc/run-baseline.ts --config=baseline-suburban.json --policy=proposed
```

### ISAC Trajectory API Reference

#### Core Functions

| Function | Description |
|----------|-------------|
| `createConfig(overrides)` | Create configuration with defaults |
| `createPolicy(name, config)` | Create baseline policy by name |
| `getAvailablePolicies()` | Get list of available policies |
| `extractInput(state, config)` | Extract input from state |
| `runTask1(config)` | Run direct optimization (MINCO + L-BFGS) |

---

## Advanced Topics

### Custom Policy Implementation

To create a custom policy, implement the `DecisionMaker` interface:

```typescript
import type { DecisionMaker, SeededRandom } from 'symbion/core';
import type { u2uMcs } from 'symbion/tasks';

class MyCustomPolicy implements DecisionMaker<u2uMcs.U2UMcsInput, number> {
    readonly name = 'my-custom-policy';

    decide(input: u2uMcs.U2UMcsInput, rng: SeededRandom): number {
        if (input.sinrDb > 20) {
            return 15;
        } else if (input.sinrDb > 10) {
            return 10;
        } else {
            return 5;
        }
    }

    reset(): void {}
}
```

---

## Troubleshooting

### Common Issues

#### 1. Environment not reset error

```
Error: Environment not reset. Call reset() first.
```

**Solution**: Always call `reset()` before calling `step()`.

#### 2. Action out of range

```
Error: Action X is out of valid range [0, 22]
```

**Solution**: Ensure action values are within the valid action space.

#### 3. TypeScript import errors

```
Cannot find module './src/tasks/u2u-mcs'
```

**Solution**: 
- Ensure the project is built: `npm run build`
- Check `tsconfig.json` paths are correctly configured

---

*Last updated: December 2025*
