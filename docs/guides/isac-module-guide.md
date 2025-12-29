# ISAC Framework Module Guide

> Joint Trajectory Optimization Framework for Integrated Sensing and Communication

This guide details how to use the Symbion library's ISAC framework module (`symbion/isac`) to achieve joint optimization of UAV trajectory planning and communication quality.

## Table of Contents

1. [Module Overview](#1-module-overview)
2. [Core Components](#2-core-components)
3. [Quick Start](#3-quick-start)
4. [Configuration Guide](#4-configuration-guide)
5. [Practical Examples](#5-practical-examples)
6. [Advanced Topics](#6-advanced-topics)
7. [Research Applications](#7-research-applications)

---

## 1. Module Overview

### 1.1 What is ISAC?

**ISAC (Integrated Sensing and Communication)** is one of the core technologies of 6G communication, aiming to integrate wireless sensing and communication functions into the same system. Key application scenarios include:

-  **UAV Communication**: Maintaining reliable connections with ground users
-  **Radar Sensing**: Using communication signals for target detection
-  **Joint Optimization**: Simultaneously optimizing communication quality and sensing performance

### 1.2 Module Capabilities

The Symbion ISAC module provides:

| Feature | Description |
|---------|-------------|
| **Trajectory Planning** | Waypoint generation based on LoS and URLLC constraints |
| **Metric Evaluation** | LoS probability, throughput, URLLC reliability calculation |
| **Constraint System** | Flight constraints and communication constraints verification |
| **AI Integration** | Converting ISAC problems into Reinforcement Learning environments |

### 1.3 Module Structure

```
src/isac/
├── index.ts           # Module Entry Point
├── planner.ts         # ISAC Trajectory Planner
├── envAdapter.ts      # AI Environment Adapter
├── report.ts          # Report Generation
├── metrics/           # Performance Metrics
│   ├── los.ts         # LoS Probability Metrics
│   ├── throughput.ts  # Throughput Metrics
│   └── urllc.ts       # URLLC Metrics
└── constraints/       # Constraint System
    ├── flight.ts      # Flight Constraints
    └── communication.ts # Communication Constraints
```

---

## 2. Core Components

### 2.1 Trajectory Planner (IsacPlanner)

`IsacPlanner` is the core class of the ISAC module, responsible for generating UAV trajectories tailored to communication constraints.

#### Basic Usage

```typescript
import { IsacPlanner, createPlanner } from 'symbion/isac';

// Method 1: Using Factory Function
const planner = createPlanner({
  startPosition: { x: 0, y: 0, z: 50 },
  endPosition: { x: 500, y: 500, z: 80 },
  numWaypoints: 10,
});

// Method 2: Direct Instantiation
const planner2 = new IsacPlanner({
  startPosition: { x: 0, y: 0, z: 50 },
  // ... other configurations
});
```

#### Core Interfaces

```typescript
interface PlannerState {
  step: number;                // Current step
  position: Position3D;        // Current position
  targetPosition?: Position3D; // Target position
  waypoints: Position3D[];     // Planned waypoints
}

interface PlanResult {
  trajectory: Position3D[];    // Trajectory waypoints
  totalDistance: number;       // Total distance (m)
  estimatedEnergy: number;     // Estimated energy consumption (J)
  feasible: boolean;           // Is feasible
}
```

### 2.2 Metric System (metrics)

#### LoS Probability Metrics

```typescript
import { metrics } from 'symbion/isac';

// Calculate Elevation Angle
const elevationAngle = metrics.calculateElevationAngle(
  { x: 100, y: 100, z: 100 },  // UAV Position
  { x: 150, y: 150, z: 0 }      // User Position
);
console.log(`Elevation Angle: ${elevationAngle.toFixed(1)}°`);

// Calculate LoS Probability (Based on ITU-R Model)
const losProb = metrics.calculateLosProbability(
  elevationAngle,
  'urban'  // Environment: 'urban' | 'suburban' | 'rural'
);
console.log(`LoS Probability: ${(losProb * 100).toFixed(1)}%`);

// Calculate Multi-User Average LoS
const avgLos = metrics.calculateAverageLoS(
  { x: 100, y: 100, z: 100 },  // UAV Position
  [
    { x: 50, y: 50, z: 0 },    // User 1
    { x: 150, y: 150, z: 0 },  // User 2
    { x: 100, y: 200, z: 0 },  // User 3
  ],
  'suburban'
);
console.log(`Average LoS: ${(avgLos * 100).toFixed(1)}%`);
```

#### URLLC Metrics

```typescript
import { metrics } from 'symbion/isac';

// URLLC Requirement Definition
const requirements: metrics.UrllcRequirements = {
  latencyMs: 1,          // Target Latency: 1ms
  reliability: 0.99999,  // Target Reliability: 99.999%
  minRateBps: 1e6,       // Min Rate: 1 Mbps
};

// Calculate BLER (from SINR)
const bler = metrics.calculateBler(
  15,  // sinrDb
  5,   // sinrThresholdDb (default)
  2    // slope (default)
);
console.log(`BLER: ${bler.toExponential(2)}`);

// Evaluate URLLC Constraints
const result = metrics.evaluateUrllc(
  2e6,         // Actual Rate (2 Mbps)
  bler,        // BLER
  0.5,         // Latency (0.5 ms)
  requirements
);

console.log('URLLC Evaluation Result:', result);
// {
//   satisfied: true,
//   latencySatisfied: true,
//   reliabilitySatisfied: true,
//   rateSatisfied: true,
//   actualLatencyMs: 0.5,
//   actualReliability: 0.99998...,
//   actualRateBps: 2000000
// }

// Calculate Violation Rate
const violationRate = metrics.calculateViolationRate([
  result,
  { ...result, satisfied: false },
]);
console.log(`Violation Rate: ${violationRate * 100}%`);  // 50%
```

### 2.3 Constraint System (constraints)

#### Flight Constraints

```typescript
import { constraints } from 'symbion/isac';

// Flight Constraint Configuration
const flightConstraints: constraints.FlightConstraints = {
  minAltitude: 30,           // Min Altitude (m)
  maxAltitude: 150,          // Max Altitude (m)
  maxSpeed: 20,              // Max Speed (m/s)
  maxAcceleration: 5,        // Max Acceleration (m/s²)
  areaBounds: [0, 0, 1000, 1000],  // [minX, minY, maxX, maxY]
};

// Check Single Constraint
const position = { x: 500, y: 500, z: 100 };
const speed = 15;

console.log('Altitude Check:', constraints.checkAltitude(position, flightConstraints));
console.log('Area Check:', constraints.checkAreaBounds(position, flightConstraints));
console.log('Speed Check:', constraints.checkSpeed(speed, flightConstraints));

// Validate All Constraints
const validation = constraints.validateFlightConstraints(
  position,
  speed,
  flightConstraints
);
console.log('Validation Result:', validation);
// { valid: true, violations: [] }

// Position Clamping (Correct if out of bounds)
const outOfBounds = { x: 1200, y: 500, z: 200 };
const clamped = constraints.clampPosition(outOfBounds, flightConstraints);
console.log('Clamped Position:', clamped);
// { x: 1000, y: 500, z: 150 }
```

#### Communication Constraints

```typescript
import { constraints } from 'symbion/isac';

// Communication Constraint Configuration
const commConstraints: constraints.CommunicationConstraints = {
  minSinrDb: 0,              // Min SINR (dB)
  maxBler: 0.1,              // Max BLER
  minThroughputBps: 1e6,     // Min Throughput (1 Mbps)
  maxOutageProbability: 0.01, // Max Outage Probability
};

// Or use default values
const defaultConstraints = constraints.DEFAULT_COMM_CONSTRAINTS;

// Check Single Constraint
console.log('SINR Check:', constraints.checkSinr(5, commConstraints));
console.log('BLER Check:', constraints.checkBler(0.05, commConstraints));
console.log('Throughput Check:', constraints.checkThroughput(2e6, commConstraints));

// Validate All Communication Constraints
const validation = constraints.validateCommConstraints(
  5,     // sinrDb
  0.05,  // bler
  2e6,   // throughputBps
  commConstraints
);
console.log('Communication Validation:', validation);
// { valid: true, violations: [] }
```

#### ConstraintSpec Factories (Core Runner Integration)

For integration with `core.Runner` and evaluation functions like `evaluateConstraints`, ISAC provides ConstraintSpec factory functions:

```typescript
import { evaluateConstraints, constraintsToPenalty } from 'symbion/core';
import {
  // Flight constraints
  maxSpeedConstraint,
  maxAccelerationConstraint,
  altitudeConstraint,
  noFlyZoneConstraint,
  energyBudgetConstraint,
  // Communication constraints
  minSinrConstraint,
  minDistanceConstraint,
} from 'symbion/isac/constraints';

// Define state type
interface UAVState {
  velocity: number[];
  acceleration: number[];
  position: number[];
  sinrDb: number;
  energyUsed: number;
}

// Create constraints for Runner
const constraints = [
  maxSpeedConstraint<UAVState>(20, 'hard'),
  ...altitudeConstraint<UAVState>(30, 150, 'hard'),
  minSinrConstraint<UAVState>(0, 'soft'),
  energyBudgetConstraint<UAVState>(1000, 'soft'),
];

// Evaluate constraints
const state: UAVState = {
  velocity: [10, 10, 0],
  acceleration: [1, 0, 0],
  position: [100, 200, 80],
  sinrDb: 15,
  energyUsed: 500,
};

const report = evaluateConstraints(constraints, state);
console.log('Feasible:', report.feasible);
console.log('Total Penalty:', report.totalPenalty);

// Convert to optimization penalty
const penalty = constraintsToPenalty(constraints, state);
console.log('Optimization Penalty:', penalty);
```

> **Note**: The ConstraintSpec factories return objects compatible with `core.evaluateConstraint` and `core.Runner`, while the simple check functions (e.g., `checkAltitude`) are for quick validation.

### 2.4 Environment Adapter (envAdapter)

Convert ISAC problems into standard AI-style environments.

```typescript
import { 
  IsacTrajectoryEnvironment, 
  createEnvironment,
  extractInput,
  getInputSpace,
  getOutputSpace,
} from 'symbion/isac';

// Create Environment
const env = createEnvironment({
  // Scenario Configuration
  startPosition: { x: 0, y: 0, z: 50 },
  endPosition: { x: 1000, y: 1000, z: 80 },
  numWaypoints: 20,
  
  // User Configuration
  numUsers: 5,
  userDistribution: 'random',
  
  // Kinematics Constraints
  kinematics: {
    maxSpeed: 20,
    maxAcceleration: 5,
    minAltitude: 30,
    maxAltitude: 150,
  },
});

// Get Space Definitions
const inputSpace = getInputSpace();
const outputSpace = getOutputSpace();

// Reset Environment
const initialState = env.reset();
const input = extractInput(initialState);

// Environment Step
const action = { dx: 10, dy: 10, dz: 0 };
const { state, reward, done, info } = env.step(action);
```

---

## 3. Quick Start

### 3.1 Basic Trajectory Planning

```typescript
import { createPlanner, metrics, constraints } from 'symbion/isac';

// 1. Create Planner
const planner = createPlanner({
  // Start and End
  startPosition: { x: 0, y: 0, z: 50 },
  endPosition: { x: 500, y: 500, z: 80 },
  
  // Number of Waypoints
  numWaypoints: 10,
  
  // Kinematic Parameters
  kinematics: {
    maxSpeed: 20,
    maxAcceleration: 5,
    minAltitude: 30,
    maxAltitude: 150,
    hoverPower: 100,          // Hover Power (W)
    propulsionEfficiency: 0.6, // Propulsion Efficiency
  },
  
  // Waypoint Interval
  waypointDurationS: 1.0,
  
  // Random Seed (Reproducibility)
  seed: 12345,
});

// 2. Execute Planning
const result = planner.plan();

console.log('Planning Result:');
console.log(`  Waypoints: ${result.trajectory.length}`);
console.log(`  Total Distance: ${result.totalDistance.toFixed(2)} m`);
console.log(`  Est. Energy: ${result.estimatedEnergy.toFixed(2)} J`);
console.log(`  Feasible: ${result.feasible ? '' : ''}`);

// 3. Print Trajectory
console.log('\nTrajectory Waypoints:');
result.trajectory.forEach((wp, i) => {
  console.log(`  [${i}] x=${wp.x.toFixed(1)}, y=${wp.y.toFixed(1)}, z=${wp.z.toFixed(1)}`);
});

// 4. Evaluate LoS Performance
const userPositions = [
  { x: 100, y: 100, z: 0 },
  { x: 300, y: 300, z: 0 },
  { x: 500, y: 200, z: 0 },
];

console.log('\nLoS Analysis:');
result.trajectory.forEach((wp, i) => {
  const avgLos = metrics.calculateAverageLoS(wp, userPositions, 'urban');
  console.log(`  Waypoint ${i}: Avg LoS = ${(avgLos * 100).toFixed(1)}%`);
});
```

### 3.2 Constrained Planning

```typescript
import { IsacPlanner, constraints, metrics } from 'symbion/isac';

// Define Constraints
const flightConstraints: constraints.FlightConstraints = {
  minAltitude: 50,
  maxAltitude: 120,
  maxSpeed: 15,
  maxAcceleration: 3,
  areaBounds: [0, 0, 600, 600],
};

const commConstraints: constraints.CommunicationConstraints = {
  minSinrDb: 5,
  maxBler: 0.01,
  minThroughputBps: 5e6,
  maxOutageProbability: 0.001,
};

// Create Planner
const planner = new IsacPlanner({
  startPosition: { x: 50, y: 50, z: 80 },
  endPosition: { x: 550, y: 550, z: 90 },
  numWaypoints: 15,
});

// Plan Trajectory
const result = planner.plan();

// Validate Constraints
let allValid = true;
const validationResults: string[] = [];

for (let i = 0; i < result.trajectory.length - 1; i++) {
  const current = result.trajectory[i];
  const next = result.trajectory[i + 1];
  
  // Calculate Speed
  const dx = next.x - current.x;
  const dy = next.y - current.y;
  const dz = next.z - current.z;
  const speed = Math.sqrt(dx*dx + dy*dy + dz*dz);
  
  // Validate Flight Constraints
  const flightValid = constraints.validateFlightConstraints(
    current, speed, flightConstraints
  );
  
  if (!flightValid.valid) {
    allValid = false;
    validationResults.push(`Waypoint ${i}: ${flightValid.violations.join(', ')}`);
  }
}

console.log('Constraint Validation Result:');
console.log(`  Overall Valid: ${allValid ? '' : ''}`);
if (!allValid) {
  validationResults.forEach(r => console.log(`  - ${r}`));
}
```

---

## 4. Configuration Guide

### 4.1 IsacTrajectoryConfig Full Configuration

```typescript
import type { IsacTrajectoryConfig } from 'symbion/isac';

const config: IsacTrajectoryConfig = {
  // ========== Basic Settings ==========
  /** Task Name */
  taskName: 'isac-trajectory-optimization',
  
  /** Random Seed (for reproducibility) */
  seed: 42,
  
  // ========== Location Settings ==========
  /** Start Position */
  startPosition: { x: 0, y: 0, z: 50 },
  
  /** End Position (Optional) */
  endPosition: { x: 1000, y: 1000, z: 80 },
  
  /** Number of Waypoints */
  numWaypoints: 20,
  
  /** Waypoint Interval (seconds) */
  waypointDurationS: 1.0,
  
  // ========== Kinematics Constraints ==========
  kinematics: {
    /** Max Speed (m/s) */
    maxSpeed: 20,
    
    /** Max Acceleration (m/s²) */
    maxAcceleration: 5,
    
    /** Min Altitude (m) */
    minAltitude: 30,
    
    /** Max Altitude (m) */
    maxAltitude: 150,
    
    /** Hover Power (W) */
    hoverPower: 100,
    
    /** Propulsion Efficiency (0-1) */
    propulsionEfficiency: 0.6,
  },
  
  // ========== Communication Parameters ==========
  communication: {
    /** Carrier Frequency (Hz) */
    carrierFrequencyHz: 3.5e9,
    
    /** Bandwidth (Hz) */
    bandwidthHz: 20e6,
    
    /** Tx Power (dBm) */
    txPowerDbm: 23,
    
    /** Noise Power Density (dBm/Hz) */
    noisePowerDensityDbmHz: -174,
    
    /** Antenna Gain (dBi) */
    antennaGainDbi: 3,
  },
  
  // ========== Scenario Settings ==========
  scenario: {
    /** Environment Type */
    environment: 'urban',  // 'urban' | 'suburban' | 'rural'
    
    /** Number of Users */
    numUsers: 10,
    
    /** User Distribution */
    userDistribution: 'uniform',  // 'uniform' | 'clustered' | 'random'
    
    /** Area Bounds */
    areaBounds: {
      minX: 0,
      maxX: 1000,
      minY: 0,
      maxY: 1000,
    },
  },
  
  // ========== Optimization Objectives ==========
  objectives: {
    /** Maximize LoS Coverage */
    losWeight: 0.4,
    
    /** Minimize Energy Constraint */
    energyWeight: 0.3,
    
    /** Maximize Throughput */
    throughputWeight: 0.3,
  },
};
```

### 4.2 Default Configuration

```typescript
import { DEFAULT_ISAC_CONFIG, createConfig } from 'symbion/isac';

// View Default Config
console.log(DEFAULT_ISAC_CONFIG);

// Create New Config based on Default (Override specific fields)
const myConfig = createConfig({
  startPosition: { x: 100, y: 100, z: 60 },
  numWaypoints: 25,
  kinematics: {
    ...DEFAULT_ISAC_CONFIG.kinematics,
    maxSpeed: 25,
  },
});
```

---

## 5. Practical Examples

### 5.1 Multi-User Coverage Optimization

```typescript
import { createPlanner, metrics, type Position3D } from 'symbion/isac';

// User Positions
const users: Position3D[] = [
  { x: 100, y: 100, z: 0 },
  { x: 300, y: 150, z: 0 },
  { x: 500, y: 300, z: 0 },
  { x: 200, y: 400, z: 0 },
  { x: 400, y: 500, z: 0 },
];

// Create Planner
const planner = createPlanner({
  startPosition: { x: 0, y: 0, z: 80 },
  endPosition: { x: 600, y: 600, z: 80 },
  numWaypoints: 12,
});

const result = planner.plan();

// Analyze User Coverage per Waypoint
console.log('User Coverage Analysis:');
console.log('═'.repeat(60));

result.trajectory.forEach((wp, i) => {
  console.log(`\nWaypoint ${i} (${wp.x.toFixed(0)}, ${wp.y.toFixed(0)}, ${wp.z.toFixed(0)})`);
  
  users.forEach((user, j) => {
    const elevation = metrics.calculateElevationAngle(wp, user);
    const losProb = metrics.calculateLosProbability(elevation, 'urban');
    
    const status = losProb > 0.8 ? ' Good' : 
                   losProb > 0.5 ? '△ Fair' : ' Poor';
    
    console.log(`  User ${j}: Elev ${elevation.toFixed(1)}°, LoS ${(losProb * 100).toFixed(0)}% ${status}`);
  });
  
  const avgLos = metrics.calculateAverageLoS(wp, users, 'urban');
  console.log(`  → Avg Coverage: ${(avgLos * 100).toFixed(1)}%`);
});

// Calculate Overall Performance
const overallPerformance = result.trajectory.reduce((sum, wp) => {
  return sum + metrics.calculateAverageLoS(wp, users, 'urban');
}, 0) / result.trajectory.length;

console.log('\n' + '═'.repeat(60));
console.log(`Overall Average Coverage: ${(overallPerformance * 100).toFixed(1)}%`);
```

### 5.2 URLLC Constrained Trajectory

```typescript
import { 
  createPlanner, 
  metrics, 
  constraints,
  type Position3D,
} from 'symbion/isac';

// URLLC Requirements (3GPP Standard)
const urllcReq: metrics.UrllcRequirements = {
  latencyMs: 1,
  reliability: 0.99999,
  minRateBps: 1e6,
};

// Base Station Position
const baseStation: Position3D = { x: 500, y: 500, z: 30 };

// Create Planner
const planner = createPlanner({
  startPosition: { x: 100, y: 100, z: 80 },
  endPosition: { x: 900, y: 900, z: 80 },
  numWaypoints: 20,
  kinematics: {
    maxSpeed: 15,
    minAltitude: 60,
    maxAltitude: 120,
  },
});

const result = planner.plan();

// Simulate Communication Performance and Evaluate URLLC
const urllcResults: metrics.UrllcResult[] = [];

console.log('URLLC Evaluation:');
console.log('─'.repeat(70));
console.log('WP | Dist(m) | SINR(dB) | BLER     | Rate(Mbps) | Status');
console.log('─'.repeat(70));

result.trajectory.forEach((wp, i) => {
  // Calculate distance to base station
  const dx = wp.x - baseStation.x;
  const dy = wp.y - baseStation.y;
  const dz = wp.z - baseStation.z;
  const distance = Math.sqrt(dx*dx + dy*dy + dz*dz);
  
  // Simplified Channel Model
  const pathLossDb = 30 + 22 * Math.log10(distance);
  const txPowerDbm = 23;
  const noisePowerDbm = -100;
  const sinrDb = txPowerDbm - pathLossDb - noisePowerDbm;
  
  // Calculate BLER
  const bler = metrics.calculateBler(sinrDb);
  
  // Calculate Rate (Simplified Shannon)
  const bandwidth = 20e6;
  const rate = bandwidth * Math.log2(1 + Math.pow(10, sinrDb / 10));
  
  // Evaluate URLLC
  const urllcResult = metrics.evaluateUrllc(rate, bler, 0.5, urllcReq);
  urllcResults.push(urllcResult);
  
  const status = urllcResult.satisfied ? '' : '';
  console.log(
    `  ${i.toString().padStart(2)} | ` +
    `${distance.toFixed(0).padStart(7)} | ` +
    `${sinrDb.toFixed(1).padStart(8)} | ` +
    `${bler.toExponential(2).padStart(8)} | ` +
    `${(rate / 1e6).toFixed(2).padStart(10)} | ` +
    status
  );
});

console.log('─'.repeat(70));
const violationRate = metrics.calculateViolationRate(urllcResults);
console.log(`URLLC Violation Rate: ${(violationRate * 100).toFixed(1)}%`);
console.log(`Compliant Waypoints: ${urllcResults.filter(r => r.satisfied).length} / ${urllcResults.length}`);
```

### 5.3 Energy Efficiency Optimization

```typescript
import { createPlanner, type Position3D } from 'symbion/isac';

// Compare Energy Consumption of Different Altitude Configs
const altitudeConfigs = [
  { name: 'Low Altitude', minAlt: 30, maxAlt: 50 },
  { name: 'Medium Altitude', minAlt: 60, maxAlt: 100 },
  { name: 'High Altitude', minAlt: 100, maxAlt: 150 },
];

console.log('Energy Efficiency Comparison:');
console.log('═'.repeat(50));

altitudeConfigs.forEach(config => {
  const planner = createPlanner({
    startPosition: { x: 0, y: 0, z: (config.minAlt + config.maxAlt) / 2 },
    endPosition: { x: 1000, y: 1000, z: (config.minAlt + config.maxAlt) / 2 },
    numWaypoints: 15,
    kinematics: {
      maxSpeed: 20,
      minAltitude: config.minAlt,
      maxAltitude: config.maxAlt,
      hoverPower: 100,
      propulsionEfficiency: 0.6,
    },
    waypointDurationS: 1.0,
  });
  
  const result = planner.plan();
  
  console.log(`\n${config.name} (${config.minAlt}-${config.maxAlt}m):`);
  console.log(`  Total Distance: ${result.totalDistance.toFixed(1)} m`);
  console.log(`  Est. Energy: ${result.estimatedEnergy.toFixed(1)} J`);
  console.log(`  Energy Efficiency: ${(result.totalDistance / result.estimatedEnergy).toFixed(3)} m/J`);
});
```

### 5.4 Integration with AI Module

```typescript
import { 
  createEnvironment,
  extractInput,
  inputToVector,
  getInputSpace,
  getOutputSpace,
  continuousOutputToOffset,
} from 'symbion/isac';
import { UavGymEnvClient, type BoxSpace, type DictSpace } from 'symbion/ai';

// Create ISAC Environment
const env = createEnvironment({
  startPosition: { x: 0, y: 0, z: 80 },
  endPosition: { x: 1000, y: 1000, z: 80 },
  numWaypoints: 20,
  seed: 42,
});

// Get Space Definitions
const inputSpace = getInputSpace();
const outputSpace = getOutputSpace();

console.log('Input Space:', JSON.stringify(inputSpace, null, 2));
console.log('Output Space:', JSON.stringify(outputSpace, null, 2));

// Create AI Client
const aiClient = new UavGymEnvClient({
  host: 'localhost',
  port: 8765,
  observationSpace: inputSpace,
  actionSpace: outputSpace,
});

// Training Loop
async function trainLoop() {
  await aiClient.connect();
  
  const numEpisodes = 100;
  
  for (let episode = 0; episode < numEpisodes; episode++) {
    // Reset Environment
    const initialState = env.reset();
    const initialInput = extractInput(initialState);
    const inputVector = inputToVector(initialInput);
    
    let action = await aiClient.reset(inputVector, { episode });
    let totalReward = 0;
    
    for (let step = 0; step < 100; step++) {
      // Convert AI Action to Offset
      const offset = continuousOutputToOffset(action as number[]);
      
      // Execute Environment Step
      const { state, reward, done, info } = env.step(offset);
      
      totalReward += reward;
      
      if (done) {
        await aiClient.step(inputToVector(extractInput(state)), reward, true, info);
        break;
      }
      
      // Get Next Action
      const input = extractInput(state);
      action = await aiClient.step(inputToVector(input), reward, false, info);
    }
    
    console.log(`Episode ${episode}: Total Reward = ${totalReward.toFixed(2)}`);
  }
  
  aiClient.disconnect();
}

trainLoop().catch(console.error);
```

---

## 6. Advanced Topics

### 6.1 Custom Metric Composition

```typescript
import { metrics } from 'symbion/isac';
import type { Position3D } from 'symbion/isac';

// Custom Comprehensive Evaluation Function
function evaluateTrajectory(
  trajectory: Position3D[],
  users: Position3D[],
  weights = { los: 0.4, energy: 0.3, smoothness: 0.3 }
): number {
  let totalScore = 0;
  
  // 1. LoS Coverage Score
  const losScores = trajectory.map(wp => 
    metrics.calculateAverageLoS(wp, users, 'urban')
  );
  const avgLos = losScores.reduce((a, b) => a + b, 0) / losScores.length;
  
  // 2. Energy Efficiency Score (Hypothetical)
  let totalEnergy = 0;
  for (let i = 1; i < trajectory.length; i++) {
    const dx = trajectory[i].x - trajectory[i-1].x;
    const dy = trajectory[i].y - trajectory[i-1].y;
    const dz = trajectory[i].z - trajectory[i-1].z;
    const dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
    totalEnergy += dist * 10;  // Simplified Energy Model
  }
  const energyScore = 1 / (1 + totalEnergy / 10000);
  
  // 3. Smoothness Score
  let jerk = 0;
  for (let i = 2; i < trajectory.length; i++) {
    const v1 = {
      x: trajectory[i-1].x - trajectory[i-2].x,
      y: trajectory[i-1].y - trajectory[i-2].y,
      z: trajectory[i-1].z - trajectory[i-2].z,
    };
    const v2 = {
      x: trajectory[i].x - trajectory[i-1].x,
      y: trajectory[i].y - trajectory[i-1].y,
      z: trajectory[i].z - trajectory[i-1].z,
    };
    const dv = Math.sqrt(
      (v2.x - v1.x)**2 + (v2.y - v1.y)**2 + (v2.z - v1.z)**2
    );
    jerk += dv;
  }
  const smoothnessScore = 1 / (1 + jerk / 100);
  
  // Weighted Sum
  totalScore = 
    weights.los * avgLos +
    weights.energy * energyScore +
    weights.smoothness * smoothnessScore;
  
  return totalScore;
}

// Usage Example
import { createPlanner } from 'symbion/isac';

const planner = createPlanner({
  startPosition: { x: 0, y: 0, z: 80 },
  endPosition: { x: 500, y: 500, z: 80 },
  numWaypoints: 10,
});

const result = planner.plan();
const users = [
  { x: 100, y: 100, z: 0 },
  { x: 250, y: 250, z: 0 },
  { x: 400, y: 400, z: 0 },
];

const score = evaluateTrajectory(result.trajectory, users);
console.log(`Overall Score: ${(score * 100).toFixed(1)}%`);
```

### 6.2 Integration with MINCO Trajectory Optimization

```typescript
import { createPlanner } from 'symbion/isac';
import { planning } from 'symbion';

// Use ISAC Planner to Generate Initial Trajectory
const isacPlanner = createPlanner({
  startPosition: { x: 0, y: 0, z: 80 },
  endPosition: { x: 500, y: 500, z: 80 },
  numWaypoints: 8,
});

const isacResult = isacPlanner.plan();

// Convert Waypoints to MINCO Format
const waypoints = isacResult.trajectory.map((wp, i) => ({
  position: [wp.x, wp.y, wp.z] as [number, number, number],
  time: i * 1.0,  // 1 second interval
}));

// Use MINCO for Trajectory Smoothing
const mincoTrajectory = planning.minco.generateMinimumSnapTrajectory(waypoints, {
  maxVelocity: 20,
  maxAcceleration: 5,
  continuityOrder: 3,  // Continuous up to jerk
});

// Sample Smoothed Trajectory
const smoothedPoints: Position3D[] = [];
const numSamples = 50;
const totalTime = mincoTrajectory.getDuration();

for (let i = 0; i < numSamples; i++) {
  const t = (i / (numSamples - 1)) * totalTime;
  const state = mincoTrajectory.evaluate(t);
  smoothedPoints.push({
    x: state.position[0],
    y: state.position[1],
    z: state.position[2],
  });
}

console.log('MINCO Smoothed Points:', smoothedPoints.length);
console.log('Total Trajectory Time:', totalTime.toFixed(2), 's');
```

---

## 7. Research Applications

### 7.1 Typical ISAC System Designs

The ISAC module can be used in the following research scenarios:

1. **UAV Communication Coverage Optimization**
   - Maximize ground user LoS probability
   - Guarantee communication quality (SINR, BLER, Throughput)

2. **URLLC Trajectory Planning**
   - Satisfy ultra-low latency constraints
   - Ensure ultra-high reliability

3. **Energy Efficiency Optimization**
   - Minimize flight energy consumption
   - Balance communication performance and energy usage

4. **Multi-UAV Collaboration**
   - User association and handover
   - Interference management

### 7.2 Paper Reproduction

Below is an example framework for reproducing a classic paper using the Symbion ISAC module:

```typescript
/**
 * Reproduce Paper: "UAV-Enabled URLLC: A Trajectory Design Perspective"
 * Goal: Maximize URLLC reliability while satisfying energy constraints
 */

import { 
  createPlanner, 
  metrics, 
  constraints,
  type Position3D,
} from 'symbion/isac';
import { optimization } from 'symbion';

// Problem Parameters
const ENERGY_BUDGET = 5000;  // Energy Budget (J)
const URLLC_TARGET_RELIABILITY = 0.99999;
const MAX_LATENCY_MS = 1;

// Optimization Objective Function
function objectiveFunction(trajectory: Position3D[]): number {
  // Calculate URLLC Reliability
  const reliabilities = trajectory.map(wp => {
    // ... Calculate communication reliability for each waypoint
    return 0.9999;  // Example value
  });
  
  const avgReliability = reliabilities.reduce((a, b) => a + b, 0) / reliabilities.length;
  
  // Goal: Maximize reliability
  return avgReliability;
}

// Constraint Function
function constraintFunction(trajectory: Position3D[]): boolean {
  // Calculate Energy Consumption
  let energy = 0;
  for (let i = 1; i < trajectory.length; i++) {
    const dx = trajectory[i].x - trajectory[i-1].x;
    const dy = trajectory[i].y - trajectory[i-1].y;
    const dz = trajectory[i].z - trajectory[i-1].z;
    const dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
    energy += dist * 10;  // Simplified Model
  }
  
  // Check Energy Constraint
  return energy <= ENERGY_BUDGET;
}

// Run Optimization
const planner = createPlanner({
  startPosition: { x: 0, y: 0, z: 80 },
  endPosition: { x: 1000, y: 1000, z: 80 },
  numWaypoints: 20,
});

const result = planner.plan();
const objective = objectiveFunction(result.trajectory);
const feasible = constraintFunction(result.trajectory);

console.log('Optimization Result:');
console.log(`  Objective Value (Reliability): ${(objective * 100).toFixed(4)}%`);
console.log(`  Constraint Satisfied: ${feasible ? 'Yes' : 'No'}`);
```

---

## Related Links

- [API Reference](../API_REFERENCE.md#9-isac-framework-isac)
- [AI Module Guide](./ai-module-guide.md)
- [Trajectory Planning Module](../API_REFERENCE.md#13-planning-planning)
- [Example Code Base](https://github.com/symbion-io/symbion-examples)

---

*Document Version: 1.0 | Last Updated: 2025-12-27*
