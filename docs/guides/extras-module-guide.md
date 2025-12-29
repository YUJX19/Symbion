# Extras Experimental Module Guide

> ⚠ Experimental Modules - API may change, strictly for advanced research use

This guide introduces the Extras module (`symbion/extras`) of the Symbion library, which contains experimental, research-oriented function modules.

## ⚠ Important Warning

**Extras module is NOT part of the Symbion core library.** Please note before use:

-  **Experimental**: These modules are for specific research scenarios
- ⚡ **Unstable API**: May change at any time, backward compatibility not guaranteed
-  **Research Oriented**: Primarily for paper research and algorithm verification
-  **Core Alternatives**: For most scenarios, prioritize `core` and `models` modules

---

## Table of Contents

1. [Module Overview](#1-module-overview)
2. [Networking - Network Protocols](#2-networking---network-protocols)
3. [PHY-Coding - Advanced Channel Coding](#3-phy-coding---advanced-channel-coding)
4. [Sandbox2D - 2D Simulation Sandbox](#4-sandbox2d---2d-simulation-sandbox)
5. [Migration Guide](#5-migration-guide)

---

## 1. Module Overview

### 1.1 Module Structure

```
symbion/extras
├── networking/    # Network Protocols (AODV, CSMA/CA)
├── phy-coding/    # Advanced Channel Coding (Re-exports)
└── sandbox2d/     # 2D Simulation Sandbox (Educational)
```

### 1.2 Usage Scenarios

| Submodule | Usage Scenario | Core Alternative |
|-----------|----------------|------------------|
| **networking** | UAV Mesh Networks, Ad-hoc Routing Research | `models/channel` |
| **phy-coding** | LDPC/Turbo/Polar Coding Research | `models/phy` (Basic Coding) |
| **sandbox2d** | Education, Rapid Prototyping | `models/robotics` |

### 1.3 Import Method

```typescript
import { extras } from 'symbion';

// Or import submodules directly
import { networking, phyCoding, sandbox2d } from 'symbion/extras';
```

---

## 2. Networking - Network Protocols

> **Usage Scenario**: UAV Mesh Network Research, Multi-robot Collaborative Communication

### 2.1 Path Loss Models

```typescript
import { extras } from 'symbion';

// ========== Friis Free Space Path Loss ==========
const distance = 500;        // 500 meters
const frequency = 2.4;       // 2.4 GHz
const fspl = extras.networking.friisPathLoss(distance, frequency);
console.log(`Free Space Loss: ${fspl.toFixed(2)} dB`);

// ========== Two-Ray Ground Reflection Model ==========
const twoRayLoss = extras.networking.twoRayPathLoss(
  distance,    // Horizontal distance
  50,          // Tx Height (m)
  10,          // Rx Height (m)
  2.4          // Frequency (GHz)
);
console.log(`Two-Ray Loss: ${twoRayLoss.toFixed(2)} dB`);

// ========== Air-to-Ground Channel Model (3GPP) ==========
const a2gLoss = extras.networking.airToGroundPathLoss(
  distance,    // Horizontal distance
  100,         // UAV Height (m)
  1.5,         // Ground Station Height (m)
  3.5,         // Frequency (GHz)
  'urban'      // Environment Type
);
console.log(`Air-to-Ground Loss: ${a2gLoss.toFixed(2)} dB`);

// ========== Log-Distance Model ==========
const logDistLoss = extras.networking.logDistancePathLoss(
  distance,
  2.4,
  2.5,    // Path Loss Exponent
  1,      // Reference Distance
  4       // Shadowing Standard Deviation
);
console.log(`Log-Distance Loss: ${logDistLoss.toFixed(2)} dB`);
```

### 2.2 Link Quality Evaluation

```typescript
import { extras } from 'symbion';

// Calculate RSSI
const rssi = extras.networking.calculateRSSI(
  23,    // Tx Power (dBm)
  3,     // Tx Antenna Gain (dBi)
  0,     // Rx Antenna Gain (dBi)
  100    // Path Loss (dB)
);
console.log(`RSSI: ${rssi} dBm`);

// Calculate SNR
const snr = extras.networking.calculateSNR(rssi, 20); // 20 MHz Bandwidth
console.log(`SNR: ${snr.toFixed(2)} dB`);

// Evaluate Link Quality
const quality = extras.networking.evaluateLinkQuality(rssi, snr);
console.log(`Link Quality: ${quality}`); // 'excellent' | 'good' | 'fair' | 'poor' | 'disconnected'

// Estimate Data Rate (Shannon Formula)
const dataRate = extras.networking.estimateDataRate(snr, 20);
console.log(`Estimated Data Rate: ${dataRate.toFixed(2)} Mbps`);

// Estimate Latency
const latency = extras.networking.estimateLatency(500, dataRate);
console.log(`Estimated Latency: ${latency.toFixed(2)} ms`);
```

### 2.3 Complete Link Analysis

```typescript
import { extras } from 'symbion';

// Define Base Station
const baseStation: extras.networking.BaseStation = {
  id: 'bs-1',
  position: { x: 500, y: 500 },
  height: 25,
  txPower: 43,          // dBm
  frequency: 3.5,       // GHz
  bandwidth: 100,       // MHz
  antennaGain: 15,      // dBi
  coverageRadius: 1000,
  color: '#FF5733',
  type: 'ground',
};

// Define Drone Node
const drone: extras.networking.DroneNode = {
  id: 'uav-1',
  pose: { x: 100, y: 200, theta: 0 },
  height: 80,
  txPower: 23,
  antennaGain: 0,
  state: 'CONNECTED',
  connectedStation: 'bs-1',
  lastUpdateTime: Date.now(),
  batteryLevel: 0.85,
  dataRate: 0,
};

// Calculate Complete Link Parameters
const link = extras.networking.calculateCommLink(drone, baseStation, 'air-to-ground', 'urban');

console.log('Link Analysis Result:');
console.log(`  Distance: ${link.distance.toFixed(1)} m`);
console.log(`  Path Loss: ${link.pathLoss.toFixed(1)} dB`);
console.log(`  RSSI: ${link.rssi.toFixed(1)} dBm`);
console.log(`  SNR: ${link.snr.toFixed(1)} dB`);
console.log(`  Link Quality: ${link.quality}`);
console.log(`  Data Rate: ${link.dataRate.toFixed(1)} Mbps`);
console.log(`  Latency: ${link.latency.toFixed(2)} ms`);
```

### 2.4 AODV Routing Protocol

Ad-hoc On-Demand Distance Vector (AODV) routing protocol.

```typescript
import { extras } from 'symbion';

// ========== Initialize AODV Nodes ==========
let node1 = extras.networking.initAodvNode('uav-1');
let node2 = extras.networking.initAodvNode('uav-2');
let node3 = extras.networking.initAodvNode('uav-3');

// ========== Create Route Request (RREQ) ==========
const node1Pos = { x: 0, y: 0 };
const { packet: rreq, newState: state1 } = extras.networking.createRREQ(
  node1,
  'uav-3',   // Destination Node
  node1Pos
);
node1 = state1;

console.log('RREQ Sent:', rreq);

// ========== Process RREQ (Intermediate Node) ==========
const node2Pos = { x: 100, y: 0 };
const neighbors2 = ['uav-1', 'uav-3'];

const result2 = extras.networking.processRREQ(node2, rreq, node2Pos, neighbors2);
node2 = result2.newState;

if (result2.forwardPackets.length > 0) {
  console.log('Node2 forwarding RREQ to neighbors');
}

// ========== Destination Node Generates RREP ==========
const node3Pos = { x: 200, y: 0 };
const result3 = extras.networking.processRREQ(node2, rreq, node3Pos, ['uav-2']);
node3 = result3.newState;

if (result3.replyPacket) {
  console.log('Node3 generated RREP:', result3.replyPacket);
}

// ========== Find Route ==========
const route = extras.networking.findRoute(node1, 'uav-3');
if (route) {
  console.log(`Route to uav-3: Next Hop = ${route.nextHop}, Hop Count = ${route.hopCount}`);
}
```

### 2.5 CSMA/CA MAC Layer Protocol

IEEE 802.11 style Carrier Sense Multiple Access/Collision Avoidance protocol.

```typescript
import { extras } from 'symbion';

// ========== Initialize MAC Nodes ==========
let mac1 = extras.networking.initMacNode('uav-1');
let mac2 = extras.networking.initMacNode('uav-2');

// ========== Enqueue Packets ==========
mac1 = extras.networking.enqueueMacPacket(mac1, 'uav-2', 1500);
mac2 = extras.networking.enqueueMacPacket(mac2, 'uav-1', 1500);

// ========== Simulation Loop ==========
const deltaTime = 10; // 10ms step
let time = 0;

while (time < 1000) { // Simulate 1 second
  // Get Channel State
  const channel = extras.networking.getChannelState([mac1, mac2]);
  
  // Update Node 1
  const r1 = extras.networking.updateMacState(
    mac1,
    channel.busy && !channel.transmittingNodes.includes('uav-1'),
    deltaTime,
    false // Received ACK?
  );
  mac1 = r1.newState;
  
  if (r1.startTx) {
    console.log(`[${time}ms] Node1 Start Transmission`);
  }
  if (r1.collision) {
    console.log(`[${time}ms] Node1 Collision Detected, Backoff`);
  }
  if (r1.txComplete) {
    console.log(`[${time}ms] Node1 Transmission Success`);
  }
  
  // Update Node 2 (Similar logic)
  const r2 = extras.networking.updateMacState(
    mac2,
    channel.busy && !channel.transmittingNodes.includes('uav-2'),
    deltaTime,
    false
  );
  mac2 = r2.newState;
  
  // Detect Collision
  if (extras.networking.detectCollision(channel.transmittingNodes)) {
    console.log(`[${time}ms] Channel Collision!`);
  }
  
  time += deltaTime;
}
```

### 2.6 Network Coverage Analysis

```typescript
import { extras } from 'symbion';

const stations = [baseStation]; // List of Base Stations

// Calculate Network Coverage
const coverage = extras.networking.calculateCoverage(
  1000,   // Area Width
  1000,   // Area Height
  stations,
  10,     // Grid Resolution
  -80     // RSSI Threshold
);

console.log(`Network Coverage: ${(coverage * 100).toFixed(1)}%`);
```

---

## 3. PHY-Coding - Advanced Channel Coding

> **Usage Scenario**: LDPC, Turbo, Polar Code professional research

> ⚠ **Note**: For most ISAC research, it is recommended to use Link Abstraction (BLER curve, Effective SNR mapping) instead of full coding simulation. Basic coding (CRC, Hamming, Convolutional) is located in `models/phy`.

### 3.1 Current Content

Current `phy-coding` module re-exports content from `models/phy/coding`:

```typescript
import { extras } from 'symbion';

// Use Basic Coding Features
const data = [1, 0, 1, 1, 0, 1];

// CRC Calculation
const crc = extras.phyCoding.crcCalculate(data, [1, 0, 0, 1]);
console.log('CRC:', crc);

// Hamming Encoding
const encoded = extras.phyCoding.hammingEncode(data);
console.log('Hamming Encoded:', encoded);
```

### 3.2 Planned Advanced Coding

The following features are planned for future versions:

- **LDPC**: 5G NR compatible Low-Density Parity-Check codes
- **Turbo**: LTE compatible Turbo codes
- **Polar**: 5G NR control channel Polar codes

### 3.3 Recommended Alternatives

For production-grade ISAC research, recommend:

```typescript
import { models } from 'symbion';

// Use Core PHY Layer Module
const encoded = models.phy.coding.crcCalculate(data, poly);
const hamming = models.phy.coding.hammingEncode(data);
const convolved = models.phy.coding.convolutionalEncode(data);
```

---

## 4. Sandbox2D - 2D Simulation Sandbox

> **Usage Scenario**: Teaching demonstration, rapid prototyping

> ⚠ **Note**: For ISAC research, please use 3D UAV components in `models/robotics`.

### 4.1 Current Status

Sandbox2D module is currently a placeholder, planned to include:

- 2D Robot Model
- 2D Obstacles
- 2D LiDAR Simulation
- 2D Occupancy Grid

```typescript
import { extras } from 'symbion';

console.log('Sandbox2D Version:', extras.sandbox2d.SANDBOX2D_VERSION);
// Output: 0.1.0
```

### 4.2 Recommended Alternatives

```typescript
import { models } from 'symbion';

// Use 3D Robotics Module
const dynamics = new models.robotics.dynamics.QuadrotorDynamics();

// Use 3D Sensing Module
const scan = models.sensing.simulateLiDAR(robotPose, obstacles, lidarConfig);

// Use 3D Planning Module
const path = models.planning.astar(start, goal, grid);
```

---

## 5. Migration Guide

### 5.1 Migrating from Extras to Core

If you are using features in Extras, here are suggestions for migrating to core modules:

| Extras Feature | Core Alternative |
|----------------|------------------|
| `networking.friisPathLoss()` | `models.channel.awgn.calculate()` |
| `networking.calculateSNR()` | `models.channel.awgn.calculate().snrLinear` |
| `phyCoding.*` | `models.phy.coding.*` |
| `sandbox2d.Robot` | `models.robotics.dynamics.QuadrotorDynamics` |
| `sandbox2d.LiDAR` | `models.sensing.simulateLiDAR()` |

### 5.2 Upgrading Extras Features to Core

If you wish to upgrade features in Extras to core modules:

1. **Add Test Coverage**: Ensure 80%+ test coverage
2. **API Documentation**: Fully document in TypeDoc
3. **Type Safety**: Complete TypeScript type definitions
4. **Backwards Compatibility**: Commit to semver versioning
5. **Submit PR**: Request code review

---

## Related Links

- [Core Module Guide](./core-module-guide.md) - Core Framework Features
- [Models Module Guide](./models-module-guide.md) - Physics Models
- [API Reference](../API_REFERENCE.md)

---

*Document Version: 1.0 | Last Updated: 2025-12-27 | Status: Experimental*

