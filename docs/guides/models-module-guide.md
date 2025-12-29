# Models Module Guide

> Provides core physical models and algorithms for ISAC simulation

This guide details how to use the Symbion library's Models module (`symbion/models`), which provides various physical models required for communication, sensing, and robotic system simulations.

## Table of Contents

1. [Module Overview](#1-module-overview)
2. [Channel - Channel Models](#2-channel---channel-models)
3. [Beamforming - Beamforming](#3-beamforming---beamforming)
4. [PHY - Physical Layer](#4-phy---physical-layer)
5. [Robotics - Robotics](#5-robotics---robotics)
6. [Planning - Path Planning](#6-planning---path-planning)
7. [Sensing - Sensing](#7-sensing---sensing)
8. [Numeric - Numerical Methods](#8-numeric---numerical-methods)
9. [Utils - Utility Functions](#9-utils---utility-functions)
10. [Full Example](#10-full-example)

---

## 1. Module Overview

### 1.1 Role of Models Module

The Models module is the physical model layer of Symbion, providing:

-  **Wireless Communication Models**: Channel, modulation, coding, beamforming
-  **Robotics Models**: Dynamics, control, UAV simulation
-  **Planning Algorithms**: Path planning, trajectory optimization
-  **Sensing Models**: Radar, LiDAR simulation
-  **Numerical Methods**: Mathematical operations, optimization algorithms

### 1.2 Module Architecture

```
symbion/models
├── channel/      # Channel Models
│   ├── awgn      # Additive White Gaussian Noise
│   ├── fading    # Rayleigh/Rician Fading
│   ├── twoRay    # Two-Ray Ground Reflection
│   ├── gscm      # Geometric Stochastic Channel Model
│   └── mimo      # MIMO Channel
├── beamforming/  # Beamforming
│   ├── array     # Antenna Arrays
│   ├── polarization # Polarization
│   └── mimo      # MIMO Precoding
├── phy/          # Physical Layer
│   ├── modulation # Modulation (BPSK, QAM, OFDM)
│   ├── coding    # Channel Coding
│   └── spreading # Spreading
├── robotics/     # Robotics
│   ├── dynamics  # Dynamic Models
│   ├── control   # Controllers
│   └── drone     # UAV Utilities
├── planning/     # Planning
│   ├── planning  # Path Planning (A*, RRT*)
│   └── trajectory # Trajectory Optimization (MINCO)
├── sensing/      # Sensing
├── numeric/      # Numerical Methods
│   ├── math      # Vector/Matrix Math
│   └── optimization # Optimization Algorithms
└── utils/        # Utility Functions
```

### 1.3 Import Method

```typescript
// Method 1: Import entire models module
import { models } from 'symbion';

// Usage
const capacity = models.channel.shannonCapacity(10, 20e6);
const af = models.beamforming.arrayFactor(theta, config);

// Method 2: Import specific submodules directly
import { channel, beamforming, planning } from 'symbion/models';

const H = channel.generateMIMOChannel(4, 2, 'rayleigh');
const pattern = beamforming.arrayPattern(config);
```

---

## 2. Channel - Channel Models

The Channel module provides implementations of various wireless channel models.

### 2.1 AWGN - Additive White Gaussian Noise

The most basic channel model, considering only the effect of white noise.

```typescript
import { models } from 'symbion';

// ========== Shannon Capacity ==========
// Formula: C = B × log₂(1 + SNR)
const snrDb = 10;          // 10 dB SNR
const bandwidth = 20e6;    // 20 MHz Bandwidth

const capacity = models.channel.shannonCapacity(snrDb, bandwidth);
console.log(`Shannon Capacity: ${(capacity / 1e6).toFixed(2)} Mbps`);
// Output: Shannon Capacity: 69.23 Mbps

// ========== Full AWGN Parameter Calculation ==========
const result = models.channel.awgnCalculate({
  snrDb: 10,
  bandwidth: 20e6,
});

console.log(`SNR (Linear): ${result.snrLinear.toFixed(2)}`);
console.log(`Capacity: ${(result.capacity / 1e6).toFixed(2)} Mbps`);
console.log(`Noise Power: ${result.noisePowerDbm.toFixed(2)} dBm`);

// ========== Add Noise ==========
const cleanSignal = [1, 1, -1, -1, 1, -1];
const noisySignal = models.channel.addNoise(cleanSignal, 10); // 10 dB SNR
console.log('Noisy Signal:', noisySignal);

// ========== Calculate Required SNR ==========
// Given target capacity and bandwidth, find required SNR
const targetCapacity = 100e6; // 100 Mbps
const requiredSnr = models.channel.awgn.requiredSnr(targetCapacity, bandwidth);
console.log(`Required SNR: ${requiredSnr.toFixed(2)} dB`);
```

### 2.2 Fading - Fading Channels

#### Rayleigh Fading (NLOS Scenarios)

```typescript
import { models } from 'symbion';

// Generate single Rayleigh fading coefficient
const h = models.channel.rayleighFading();
console.log(`Magnitude: |h| = ${h.magnitude.toFixed(3)}`);
console.log(`Phase: ∠h = ${(h.phase * 180 / Math.PI).toFixed(1)}°`);
console.log(`Complex: ${h.real.toFixed(3)} + j${h.imag.toFixed(3)}`);

// Batch Generation
const batch = models.channel.fading.rayleighFadingBatch(1000);
const avgMagnitude = batch.magnitude.reduce((a, b) => a + b) / batch.magnitude.length;
console.log(`Average Magnitude: ${avgMagnitude.toFixed(3)}`); // Approx 0.886
```

#### Rician Fading (LOS Scenarios)

```typescript
import { models } from 'symbion';

// K Factor: Ratio of LOS power to scattered power
const kFactorDb = 10;  // 10 dB
const kFactor = models.channel.fading.kFactorFromDb(kFactorDb);

const h = models.channel.ricianFading({ 
  kFactor: kFactor,
  losPhase: 0,  // LOS component phase
});

console.log(`Rician Fading Magnitude: ${h.magnitude.toFixed(3)}`);
// Larger K means magnitude is more stable near 1
```

#### Doppler Effect

```typescript
import { models } from 'symbion';

const velocity = 30;       // 30 m/s (approx 108 km/h)
const carrierFreq = 3.5e9; // 3.5 GHz (5G band)

// Maximum Doppler Shift
const maxDoppler = models.channel.maxDopplerShift(velocity, carrierFreq);
console.log(`Max Doppler Shift: ${maxDoppler.toFixed(1)} Hz`);

// Doppler Shift at specific angle
const angle = Math.PI / 6; // 30 degrees
const doppler = models.channel.dopplerShift(velocity, carrierFreq, angle);
console.log(`Doppler Shift (30°): ${doppler.toFixed(1)} Hz`);

// Coherence Time
const coherenceT = models.channel.coherenceTime(maxDoppler);
console.log(`Coherence Time: ${(coherenceT * 1000).toFixed(2)} ms`);

// Coherence Bandwidth (assuming 1μs delay spread)
const delaySpread = 1e-6;
const coherenceB = models.channel.coherenceBandwidth(delaySpread);
console.log(`Coherence Bandwidth: ${(coherenceB / 1e3).toFixed(1)} kHz`);
```

### 2.3 GSCM - Geometric Stochastic Channel Model

Used for simulating time-varying channels with spatial consistency.

```typescript
import { models } from 'symbion';

// Initialize Scattering Channel
const channelState = models.channel.initScatterChannel({
  numScatterers: 20,
  maxDelay: 1e-6,
  carrierFreq: 3.5e9,
  txPosition: { x: 0, y: 0, z: 10 },
  rxPosition: { x: 100, y: 50, z: 1.5 },
});

// Update Channel (Time varying)
const t = 0.1; // After 100 ms
const updatedState = models.channel.updateScatterChannel(channelState, t);

// Get Frequency Response
const frequencies = [3.5e9, 3.5001e9, 3.5002e9];
const response = models.channel.getFrequencyResponse(updatedState, frequencies);
console.log('Frequency Response:', response);
```

### 2.4 MIMO - Multiple Antenna Channels

```typescript
import { models } from 'symbion';

// Generate MIMO Channel Matrix (4x2 MIMO)
const numTx = 4;  // Number of Tx antennas
const numRx = 2;  // Number of Rx antennas
const H = models.channel.generateMIMOChannel(numTx, numRx, 'rayleigh');

console.log(`Channel Matrix Dimensions: ${H.length}x${H[0].length}`);
console.log('Channel Matrix H:');
H.forEach((row, i) => {
  const rowStr = row.map(x => `${x.toFixed(3)}`).join(', ');
  console.log(`  [${i}]: [${rowStr}]`);
});

// Calculate MIMO Capacity
const snrDb = 10;
const capacity = models.channel.calculateMIMOCapacity(H, snrDb);
console.log(`MIMO Capacity: ${capacity.toFixed(2)} bps/Hz`);

// Diversity-Multiplexing Tradeoff
const { diversity, multiplexing } = models.channel.diversityMultiplexingTradeoff(
  { numTx, numRx },
  0.5 // Multiplexing gain 0.5
);
console.log(`Diversity Gain: ${diversity.toFixed(2)}, Multiplexing Gain: ${multiplexing.toFixed(2)}`);
```

---

## 3. Beamforming - Beamforming

### 3.1 Antenna Arrays

```typescript
import { models } from 'symbion';

// ========== Array Factor ==========
const config = {
  numAntennas: 8,    // 8 Antenna ULA
  spacing: 0.5,      // Half-wavelength spacing
  steeringAngle: 0,  // Steering angle 0 (Boresight)
};

// Calculate Array Factor at specific angle
const theta = Math.PI / 6; // 30 degrees
const af = models.beamforming.arrayFactor(theta, config);
console.log(`Array Factor at θ=30°: ${af.toFixed(4)}`);

const afDb = models.beamforming.arrayFactorDb(theta, config);
console.log(`Array Factor at θ=30°: ${afDb.toFixed(2)} dB`);

// ========== Generate Pattern ==========
const pattern = models.beamforming.arrayPattern(config, 361);

// Find Main Lobe Peak and Nulls
const maxGain = Math.max(...pattern.pattern);
const maxIndex = pattern.pattern.indexOf(maxGain);
console.log(`Main Lobe Direction: ${(pattern.angles[maxIndex] * 180 / Math.PI).toFixed(1)}°`);

// ========== Half Power Beamwidth ==========
const hpbw = models.beamforming.halfPowerBeamwidth(config);
console.log(`HPBW: ${(hpbw * 180 / Math.PI).toFixed(2)}°`);

// ========== First Null Beamwidth ==========
const fnbw = models.beamforming.firstNullBeamwidth(config);
console.log(`FNBW: ${(fnbw * 180 / Math.PI).toFixed(2)}°`);

// ========== Array Gain ==========
const gain = models.beamforming.arrayGain(config);
console.log(`Array Gain: ${gain} (${10 * Math.log10(gain).toFixed(2)} dBi)`);

// ========== Steering Phases ==========
const targetAngle = Math.PI / 4; // Steer to 45 degrees
const phases = models.beamforming.steeringPhases(targetAngle, config);
console.log('Phase Shifter Settings (rad):', phases.map(p => p.toFixed(3)));
```

### 3.2 Polarization

```typescript
import { models } from 'symbion';

// Polarization Loss Factor
const polTx = { h: 1, v: 0 }; // Horizontal polarization Tx
const polRx = { h: Math.cos(Math.PI/4), v: Math.sin(Math.PI/4) }; // 45° polarization Rx

const lossFactor = models.beamforming.polarizationLossFactor(polTx, polRx);
console.log(`Polarization Matching Factor: ${lossFactor.toFixed(4)}`); // 0.5

const lossDb = models.beamforming.polarizationLossDb(polTx, polRx);
console.log(`Polarization Loss: ${lossDb.toFixed(2)} dB`); // 3 dB

// Jones Vector
const jones = models.beamforming.jonesVector(Math.PI / 4); // 45° Linear Polarization
console.log(`Jones Vector: [${jones.h.toFixed(3)}, ${jones.v.toFixed(3)}]`);
```

### 3.3 MIMO Beamforming

```typescript
import { models } from 'symbion';

// Generate Channel Matrix
const H = models.beamforming.generateChannelMatrix(4, 2);

// MRT (Maximum Ratio Transmission) SINR
const mrtSinr = models.beamforming.mrtSinr(H, 1);
console.log(`MRT SINR: ${mrtSinr.toFixed(2)}`);

// ZF (Zero Forcing) SINR
const zfSinr = models.beamforming.zfSinr(H, 1);
console.log(`ZF SINR: ${zfSinr.toFixed(2)}`);

// MIMO Capacity
const capacity = models.beamforming.mimoCapacity(H, 10);
console.log(`MIMO Capacity: ${capacity.toFixed(2)} bps/Hz`);

// Capacity vs Antennas
const capacityVsN = models.beamforming.capacityVsAntennas(16, 4, 10);
console.log('Capacity vs Number of Antennas:', capacityVsN);
```

---

## 4. PHY - Physical Layer

### 4.1 Modulation - Modulation

```typescript
import { models } from 'symbion';

// ========== QAM Modulation ==========
const bits = [0, 1, 1, 0, 1, 1, 0, 0];

// 16-QAM Modulation
const symbols = models.phy.modulation.qamModulate(bits, 16);
console.log('16-QAM Symbols:', symbols);

// Demodulation
const demodBits = models.phy.modulation.qamDemodulate(symbols, 16);
console.log('Demodulated Bits:', demodBits);

// ========== Constellation Generation ==========
const constellation = models.phy.modulation.generateConstellation(16);
console.log('16-QAM Constellation Points:', constellation);

// ========== OFDM ==========
const ofdmData = [1, -1, 1, 1, -1, -1, 1, -1];

// OFDM Modulation
const ofdmSignal = models.phy.modulation.ofdmModulate(ofdmData, {
  numSubcarriers: 64,
  cpLength: 16,
});

// OFDM Demodulation
const recoveredData = models.phy.modulation.ofdmDemodulate(ofdmSignal, {
  numSubcarriers: 64,
  cpLength: 16,
});
```

### 4.2 Coding - Channel Coding

```typescript
import { models } from 'symbion';

// ========== CRC ==========
const data = [1, 0, 1, 1, 0, 1];
const crcPoly = [1, 0, 0, 1]; // x³ + 1

const crc = models.phy.coding.crcCalculate(data, crcPoly);
console.log('CRC Checksum:', crc);

// ========== Hamming Encoding ==========
const encoded = models.phy.coding.hammingEncode(data);
console.log('Hamming Encoded:', encoded);

const decoded = models.phy.coding.hammingDecode(encoded);
console.log('Hamming Decoded:', decoded);

// ========== Convolutional Encoding ==========
const convEncoded = models.phy.coding.convolutionalEncode(data);
console.log('Convolutional Encoded:', convEncoded);

const viterbiDecoded = models.phy.coding.viterbiDecode(convEncoded);
console.log('Viterbi Decoded:', viterbiDecoded);
```

### 4.3 Spreading - Spreading

```typescript
import { models } from 'symbion';

// ========== m-Sequence ==========
const mSeq = models.phy.spreading.mSequence([1, 0, 0], 7);
console.log('m-Sequence:', mSeq);

// ========== Gold Code ==========
const goldCode = models.phy.spreading.goldSequence(5, 3);
console.log('Gold Code:', goldCode);

// ========== Walsh Code ==========
const walshCode = models.phy.spreading.walshCode(3, 8);
console.log('Walsh Code:', walshCode);

// ========== DSSS Spreading ==========
const dataBits = [1, -1, 1];
const spreadCode = [1, 1, -1, -1, 1, -1, 1];
const spreadSignal = models.phy.spreading.dsssSpread(dataBits, spreadCode);
console.log('Spread Signal Length:', spreadSignal.length);
```

---

## 5. Robotics - Robotics

### 5.1 Dynamics - Dynamics

```typescript
import { models } from 'symbion';

// ========== Quadrotor Dynamics ==========
const quadDynamics = new models.robotics.dynamics.QuadrotorDynamics({
  mass: 1.5,        // Mass (kg)
  armLength: 0.25,  // Arm Length (m)
  Ix: 0.02,         // X-axis Inertia
  Iy: 0.02,         // Y-axis Inertia
  Iz: 0.04,         // Z-axis Inertia
});

const state = {
  position: [0, 0, 10],
  velocity: [1, 0, 0],
  attitude: [0, 0.1, 0], // roll, pitch, yaw
  angularVelocity: [0, 0, 0],
};

const control = {
  thrust: 15,       // N
  torque: [0, 0.1, 0],
};

// Calculate Derivatives
const derivative = quadDynamics.derivative(state, control);
console.log('Acceleration:', derivative.acceleration);
console.log('Angular Acceleration:', derivative.angularAcceleration);

// ========== Differential Drive Kinematics ==========
const diffDrive = new models.robotics.dynamics.DifferentialDrive({
  wheelBase: 0.5,
  wheelRadius: 0.1,
});

const pose = { x: 0, y: 0, theta: 0 };
const wheelVelocities = { left: 1.0, right: 1.2 };
const newPose = diffDrive.update(pose, wheelVelocities, 0.1);
console.log('New Pose:', newPose);
```

### 5.2 Control - Control

```typescript
import { models } from 'symbion';

// ========== Differential Flatness Mapping ==========
const flatness = new models.robotics.control.FlatnessMap();

const trajectoryState = {
  position: [0, 0, 10],
  velocity: [1, 0, 0],
  acceleration: [0.5, 0, 0],
  jerk: [0, 0, 0],
};

const controlInput = flatness.forward(trajectoryState, 0); // yaw = 0
console.log('Thrust:', controlInput.thrust.toFixed(2), 'N');
console.log('Attitude:', controlInput.attitude);
console.log('Body Rates:', controlInput.bodyRates);

// ========== Cascade Controller ==========
const cascadeController = new models.robotics.control.CascadeController({
  positionGains: { kp: 1.0, ki: 0.1, kd: 0.5 },
  attitudeGains: { kp: 5.0, ki: 0.1, kd: 1.0 },
});

const currentState = {
  position: [0, 0, 9],
  velocity: [0, 0, 0],
  attitude: [0, 0, 0],
};

const targetState = {
  position: [0, 0, 10],
  velocity: [0, 0, 0],
};

const control = cascadeController.compute(currentState, targetState, 0.01);
console.log('Control Output:', control);
```

### 5.3 Drone - UAV Utilities

```typescript
import { models } from 'symbion';

// Distance Calculation
const pos1 = { x: 0, y: 0, z: 0 };
const pos2 = { x: 3, y: 4, z: 0 };
const dist = models.robotics.drone.distance(pos1, pos2);
console.log('Distance:', dist); // 5

// Angle Normalization
const angle = 5.0; // Radians
const normalized = models.robotics.drone.normalizeAngle(angle);
console.log('Normalized Angle:', normalized); // [-π, π]

// Check if Goal Reached
const currentPose = { x: 9.9, y: 9.9, z: 10 };
const goal = { x: 10, y: 10, z: 10 };
const reached = models.robotics.drone.goalReached(currentPose, goal, 0.5);
console.log('Goal Reached:', reached); // true
```

---

## 6. Planning - Path Planning

### 6.1 Path Planning Algorithms

```typescript
import { models } from 'symbion';

// ========== A* Algorithm ==========
const grid = createOccupancyGrid(100, 100); // Create Occupancy Grid
const start = { x: 0, y: 0 };
const goal = { x: 90, y: 90 };

const astarPath = models.planning.astar(start, goal, grid);
console.log('A* Waypoints:', astarPath.length);
console.log('A* Path:', astarPath);

// ========== RRT* Algorithm ==========
const obstacles = [
  { center: { x: 50, y: 50 }, radius: 10 },
  { center: { x: 30, y: 70 }, radius: 8 },
];

const rrtPath = models.planning.rrtStar({
  start: { x: 0, y: 0 },
  goal: { x: 100, y: 100 },
  obstacles,
  maxIterations: 5000,
  stepSize: 5,
  goalBias: 0.1,
});

console.log('RRT* Waypoints:', rrtPath.length);

// ========== RVO (Reciprocal Velocity Obstacles) ==========
const agents = [
  { position: { x: 0, y: 0 }, velocity: { x: 1, y: 0 }, radius: 0.5 },
  { position: { x: 10, y: 0 }, velocity: { x: -1, y: 0 }, radius: 0.5 },
];

const avoidanceVelocities = models.planning.rvo(agents, 1.0);
console.log('Avoidance Velocities:', avoidanceVelocities);
```

### 6.2 Trajectory Optimization

```typescript
import { models } from 'symbion';

// ========== MINCO Minimum Snap Trajectory ==========
const waypoints = [
  { position: [0, 0, 0], time: 0 },
  { position: [10, 5, 2], time: 2 },
  { position: [20, 0, 4], time: 4 },
  { position: [30, 5, 2], time: 6 },
];

const mincoTrajectory = models.planning.generateMinimumSnapTrajectory(waypoints, {
  maxVelocity: 5,
  maxAcceleration: 3,
});

// Sample Trajectory
const state_at_1s = mincoTrajectory.evaluate(1.0);
console.log('Position at t=1s:', state_at_1s.position);
console.log('Velocity at t=1s:', state_at_1s.velocity);
console.log('Acceleration at t=1s:', state_at_1s.acceleration);

// Get Duration
const duration = mincoTrajectory.getDuration();
console.log('Total Duration:', duration, 's');

// ========== Polynomial Trajectory ==========
const polynomial = new models.planning.trajectory.Polynomial([1, 2, 0.5, -0.1]);

// Evaluation
console.log('p(1) =', polynomial.evaluate(1));

// Derivative
const derivative = polynomial.derivative();
console.log('p\'(1) =', derivative.evaluate(1));

// ========== Piecewise Trajectory ==========
// Note: Piecewise trajectories are typically generated by MINCO or
// other optimization functions (see generateMinimumSnapTrajectory above).
```

### 6.3 Safe Corridor

```typescript
import { models } from 'symbion';

// FIRI Algorithm for Safe Flight Corridor Generation
const path = [
  { x: 0, y: 0, z: 5 },
  { x: 10, y: 5, z: 5 },
  { x: 20, y: 0, z: 5 },
];

const obstacles = [
  { center: { x: 5, y: 2 }, radius: 2 },
];

const corridor = models.planning.firi(path, obstacles);
console.log('Corridor Polyhedrons:', corridor.length);

// Convex Hull Calculation
const points = [
  { x: 0, y: 0, z: 0 },
  { x: 1, y: 0, z: 0 },
  { x: 0, y: 1, z: 0 },
  { x: 0, y: 0, z: 1 },
  { x: 0.5, y: 0.5, z: 0.5 },
];

const hull = models.planning.quickHull(points);
console.log('Hull Faces:', hull.length);
```

---

## 7. Sensing - Sensing

### 7.1 LiDAR Simulation

```typescript
import { models } from 'symbion';

// 2D LiDAR Scan Simulation
const robotPose = { x: 0, y: 0, theta: 0 };
const obstacles = [
  { type: 'circle', center: { x: 5, y: 2 }, radius: 1 },
  { type: 'line', start: { x: -3, y: 4 }, end: { x: 3, y: 4 } },
];

const lidarConfig = {
  numRays: 360,
  maxRange: 10,
  minAngle: -Math.PI,
  maxAngle: Math.PI,
};

const scan = models.sensing.simulateLiDAR(robotPose, obstacles, lidarConfig);
console.log('Scan Points:', scan.ranges.length);
console.log('Closest Distance:', Math.min(...scan.ranges.filter(r => r < Infinity)));
```

### 7.2 FOV Detection

```typescript
import { models } from 'symbion';

// Check if Target is in FOV
const sensorPose = { x: 0, y: 0, theta: 0 };
const target = { x: 5, y: 2 };
const fovAngle = Math.PI / 3; // 60 degrees

const inFov = models.sensing.isInFOV(sensorPose, target, fovAngle);
console.log('Target in FOV:', inFov);

// Ray-Circle Intersection
const rayOrigin = { x: 0, y: 0 };
const rayDir = { x: 1, y: 0 };
const circle = { center: { x: 5, y: 0 }, radius: 1 };

const intersection = models.sensing.rayCircleIntersection(rayOrigin, rayDir, circle);
console.log('Intersection Distance:', intersection);
```

### 7.3 Environment Modeling

```typescript
import { models } from 'symbion';

// Occupancy Grid Map
const occupancyGrid = new models.sensing.OccupancyGrid({
  width: 100,
  height: 100,
  resolution: 0.1, // 0.1m per cell
});

// Update Cell
occupancyGrid.updateCell(50, 50, 0.9); // Obstacle

// Query Cell
const occupancy = occupancyGrid.getCell(50, 50);
console.log('Occupancy Probability:', occupancy);

// Ray Update
occupancyGrid.rayUpdate(
  { x: 0, y: 0 },
  { x: 5, y: 5 },
  true // End is obstacle
);
```

---

## 8. Numeric - Numerical Methods

### 8.1 Vector and Matrix Math

```typescript
import { models } from 'symbion';

const { vec3, mat3 } = models.numeric.math;

// ========== 3D Vector Math ==========
const a = [1, 2, 3];
const b = [4, 5, 6];

console.log('Add:', vec3.add(a, b));       // [5, 7, 9]
console.log('Sub:', vec3.sub(a, b));       // [-3, -3, -3]
console.log('Scale:', vec3.scale(a, 2));     // [2, 4, 6]
console.log('Dot:', vec3.dot(a, b));       // 32
console.log('Cross:', vec3.cross(a, b));     // [-3, 6, -3]
console.log('Norm:', vec3.norm(a));         // 3.74...
console.log('Normalize:', vec3.normalize(a));  // [0.27, 0.53, 0.80]

// ========== 3x3 Matrix Math ==========
const M = [
  [1, 2, 3],
  [4, 5, 6],
  [7, 8, 9],
];

console.log('Transpose:', mat3.transpose(M));
console.log('Determinant:', mat3.determinant(M));

// Matrix-Vector Multiplication
const v = [1, 0, 0];
console.log('Mv:', mat3.mulVec(M, v)); // [1, 4, 7]

// Matrix-Matrix Multiplication
const N = mat3.identity();
console.log('MN:', mat3.mul(M, N));
```

### 8.2 Optimization Algorithms

```typescript
import { models } from 'symbion';

// ========== L-BFGS Optimization ==========
// Minimize f(x) = (x[0]-1)² + (x[1]-2)²

function objectiveFunction(x: number[]): { value: number; gradient: number[] } {
  const value = (x[0] - 1) ** 2 + (x[1] - 2) ** 2;
  const gradient = [
    2 * (x[0] - 1),
    2 * (x[1] - 2),
  ];
  return { value, gradient };
}

const x0 = [0, 0];
const result = models.numeric.lbfgsOptimize(x0, objectiveFunction, {
  maxIterations: 100,
  tolerance: 1e-6,
});

console.log('Optimal X:', result.x);      // Close to [1, 2]
console.log('Minimal Value:', result.value); // Close to 0
console.log('Iterations:', result.iterations);

// ========== Seidel Linear Programming ==========
// Maximize c'x, subject to Ax <= b
const c = [3, 2];
const A = [
  [1, 1],
  [2, 1],
  [-1, 0],
  [0, -1],
];
const b = [4, 5, 0, 0];

const lpResult = models.numeric.sdlp(A, b, c);
console.log('LP Optimal X:', lpResult.x);
console.log('LP Max Value:', lpResult.value);
```

---

## 9. Utils - Utility Functions

### 9.1 Unit Conversions

```typescript
import { models } from 'symbion';

const { conversion } = models.utils;

// dB <-> Linear
console.log('10 dB -> Linear:', conversion.dbToLinear(10));    // 10
console.log('100 Linear -> dB:', conversion.linearToDb(100)); // 20

// Power Conversion
console.log('1W -> dBm:', conversion.wattsToDbm(1));      // 30 dBm
console.log('30 dBm -> W:', conversion.dbmToWatts(30));   // 1 W

// Frequency <-> Wavelength
console.log('3 GHz Wavelength:', conversion.frequencyToWavelength(3e9)); // 0.1 m
console.log('0.1m Frequency:', conversion.wavelengthToFrequency(0.1)); // 3 GHz
```

### 9.2 Statistical Functions

```typescript
import { models } from 'symbion';

const { statistics } = models.utils;

// Gaussian Random Number
const sample = statistics.gaussianRandom(0, 1);
console.log('Gaussian Sample:', sample);

// Q Function
const q = statistics.qFunction(1.96);
console.log('Q(1.96):', q); // Approx 0.025

// Complex Gaussian Random Number
const complexSample = statistics.complexGaussianRandom(0, 1);
console.log('Complex Gaussian:', complexSample);
```

---

## 10. Full Example

### UAV-Ground User Communication Link Simulation

```typescript
import { models } from 'symbion';

// ========== Scenario Configuration ==========
const uavPosition = { x: 0, y: 0, z: 100 };      // UAV Position (m)
const userPosition = { x: 500, y: 300, z: 1.5 }; // Ground User Position (m)
const carrierFreq = 3.5e9;                       // Carrier Freq 3.5 GHz
const bandwidth = 20e6;                          // Bandwidth 20 MHz
const txPowerDbm = 23;                           // Tx Power 23 dBm

// ========== Calculate Distance and Angle ==========
const dx = userPosition.x - uavPosition.x;
const dy = userPosition.y - uavPosition.y;
const dz = uavPosition.z - userPosition.z;
const distance3D = Math.sqrt(dx*dx + dy*dy + dz*dz);
const elevationAngle = Math.atan2(dz, Math.sqrt(dx*dx + dy*dy));

console.log(`3D Distance: ${distance3D.toFixed(2)} m`);
console.log(`Elevation Angle: ${(elevationAngle * 180 / Math.PI).toFixed(2)}°`);

// ========== Free Space Path Loss ==========
const wavelength = 3e8 / carrierFreq;
const fspl = 20 * Math.log10(4 * Math.PI * distance3D / wavelength);
console.log(`Free Space Path Loss: ${fspl.toFixed(2)} dB`);

// ========== Channel Fading ==========
// Estimate K Factor based on Elevation Angle (Simplified Model)
const kFactorDb = 5 + 0.2 * elevationAngle * 180 / Math.PI;
const kFactor = models.channel.fading.kFactorFromDb(kFactorDb);
const fading = models.channel.ricianFading({ kFactor });

const fadingLossDb = -20 * Math.log10(fading.magnitude);
console.log(`Fading Loss: ${fadingLossDb.toFixed(2)} dB`);

// ========== SNR Calculation ==========
const noisePowerDbm = -174 + 10 * Math.log10(bandwidth); // Thermal Noise
const rxPowerDbm = txPowerDbm - fspl - fadingLossDb;
const snrDb = rxPowerDbm - noisePowerDbm;
console.log(`Rx Power: ${rxPowerDbm.toFixed(2)} dBm`);
console.log(`SNR: ${snrDb.toFixed(2)} dB`);

// ========== Capacity Calculation ==========
const capacity = models.channel.shannonCapacity(snrDb, bandwidth);
console.log(`Channel Capacity: ${(capacity / 1e6).toFixed(2)} Mbps`);

// ========== Beamforming Gain (UAV uses 4-Antenna ULA) ==========
const arrayConfig = {
  numAntennas: 4,
  spacing: 0.5,
  steeringAngle: elevationAngle,
};
const arrayGain = models.beamforming.arrayGain(arrayConfig);
const arrayGainDb = 10 * Math.log10(arrayGain);
console.log(`Array Gain: ${arrayGainDb.toFixed(2)} dB`);

// Capacity with Beamforming
const snrWithBfDb = snrDb + arrayGainDb;
const capacityWithBf = models.channel.shannonCapacity(snrWithBfDb, bandwidth);
console.log(`Capacity with Beamforming: ${(capacityWithBf / 1e6).toFixed(2)} Mbps`);
```

---

## Related Links

- [API Reference](../API_REFERENCE.md)
- [Core Module Guide](./core-module-guide.md)
- [AI Module Guide](./ai-module-guide.md)
- [ISAC Module Guide](./isac-module-guide.md)

---

*Document Version: 1.0 | Last Updated: 2025-12-27*
