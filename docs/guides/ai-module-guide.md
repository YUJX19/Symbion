# AI Interface Module Guide

> Bridge connecting Symbion simulator and AI frameworks

This guide details how to use the Symbion library's AI interface module (`symbion/ai`) to integrate the TypeScript simulation environment with various AI frameworks, including Reinforcement Learning (PyTorch, RLlib, Stable-Baselines3), Large Language Models, and other AI systems.

## Table of Contents

1. [Module Overview](#1-module-overview)
2. [Core Concepts](#2-core-concepts)
3. [Quick Start](#3-quick-start)
4. [Detailed API Description](#4-detailed-api-description)
5. [Practical Examples](#5-practical-examples)
6. [FAQ & Best Practices](#6-faq-and-best-practices)

---

## 1. Module Overview

### 1.1 Purpose and Scenarios

The AI Interface Module provides a **framework-agnostic** interface for connecting Symbion to AI backends:

-  **Interactive Loop**: Use `step()` / `reset()` methods for iterative AI interactions
-  **Single Query**: Use `query()` method for one-shot inference (LLMs, Vision models)
-  **Batch Processing**: Use `batch()` method for parallel processing
-  **Streaming**: Use `stream()` method for generative models
-  **Type Safety**: Complete TypeScript type definitions ensuring consistency
-  **Easy Testing**: Provides InMemory transport layer for unit testing

### 1.2 Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                      AI Backend Server                          │
│         (RL Agent / LLM / Vision Model / Custom)                │
│                    ↑          ↓                                 │
│                 Output      Input                               │
│                    ↑          ↓                                 │
├─────────────────── WebSocket Protocol ──────────────────────┤
│                    ↑          ↓                                 │
│                 Output      Input                               │
│                    ↑          ↓                                 │
│              Symbion AI Client (TypeScript)                     │
│    ┌─────────────────────────────────────────────────────┐      │
│    │  One-shot: query() / batch() / stream()          │      │
│    │  Iterative: step() / reset()                     │      │
│    └─────────────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────────┘
```

### 1.3 Client Types

| Client Class | Use Case | Primary Methods |
|--------------|----------|-----------------|
| `AiInterfaceClient` | One-shot AI inference (LLM, Vision, etc.) | `query()`, `batch()`, `stream()` |
| `IterativeAiClient` | Iterative AI loops (step/reset pattern) | `step()`, `reset()` (+ inherits generic methods) |
| `UavGymEnvClient` | Legacy alias for IterativeAiClient | Same as above |
| `GymEnvClient` | Legacy alias for IterativeAiClient | Same as above |

### 1.4 Module Composition

```
src/ai/
├── index.ts                     # Module entry point
├── interface/
│   ├── index.ts                 # Interface exports
│   ├── interface.ts             # Type definitions (Space, Msg, etc.)
│   ├── client.ts                # Generic AI client (AiInterfaceClient)
│   ├── protocol.ts              # Protocol handler (ProtocolHandler)
│   ├── env/
│   │   ├── index.ts
│   │   ├── rlEnvClient.ts       # IterativeAiClient implementation
│   │   ├── rlObservation.ts     # Input extraction utilities
│   │   └── rlAction.ts          # Output mapping utilities
│   └── transport/
│       ├── index.ts
│       ├── transport.ts         # Transport layer interface
│       ├── ws.browser.ts        # Browser WebSocket implementation
│       └── memory.ts            # Memory transport (for testing)
└── tasks/
    ├── index.ts
    └── u2u-mcs.ts               # U2U MCS task adapter
```

---

## 2. Core Concepts

### 2.1 Space Type System

Symbion AI interface uses a standardized Space type system for defining input/output formats. Supported types:

#### DiscreteSpace

```typescript
import type { DiscreteSpace } from 'symbion/ai';

const mcsSpace: DiscreteSpace = {
  kind: 'discrete',
  n: 29,                                // 0-28 Total 29 options
  labels: ['MCS-0', 'MCS-1', ..., 'MCS-28'],  // Optional labels
};
```

#### BoxSpace - Continuous Space

```typescript
import type { BoxSpace } from 'symbion/ai';

// 3D Position Space
const positionSpace: BoxSpace = {
  kind: 'box',
  shape: [3],                          // 3D vector
  low: [0, 0, 30],                     // Min value
  high: [1000, 1000, 150],             // Max value
  dtype: 'float32',                    // Data type (optional)
};

// Multi-dimensional Observation Space
const observationSpace: BoxSpace = {
  kind: 'box',
  shape: [12],                         // 12D observation vector
  low: -Infinity,                      // Scalar auto-broadcasting
  high: Infinity,
};
```

#### MultiDiscreteSpace

```typescript
import type { MultiDiscreteSpace } from 'symbion/ai';

const actionSpace: MultiDiscreteSpace = {
  kind: 'multiDiscrete',
  nvec: [9, 9, 5, 29],                 // Number of options per dimension
  labels: [
    ['left', 'stay', 'right', ...],    // Horizontal direction
    ['back', 'stay', 'forward', ...],  // Vertical direction
    ['down', ..., 'up'],               // Altitude
    ['MCS-0', ..., 'MCS-28'],          // MCS selection
  ],
};
```

#### DictSpace

```typescript
import type { DictSpace, BoxSpace, DiscreteSpace } from 'symbion/ai';

const observationSpace: DictSpace = {
  kind: 'dict',
  spaces: {
    position: { kind: 'box', shape: [3], low: 0, high: 1000 } as BoxSpace,
    velocity: { kind: 'box', shape: [3], low: -20, high: 20 } as BoxSpace,
    mcsIndex: { kind: 'discrete', n: 29 } as DiscreteSpace,
    sinrDb: { kind: 'box', shape: [1], low: -20, high: 40 } as BoxSpace,
  },
};
```

### 2.2 Protocol Layer

The AI interface uses a JSON-based WebSocket protocol for communication. Each message contains:

```typescript
interface Msg<T> {
  v: 1;                    // Protocol version
  type: MsgType;           // Message type
  sessionId: string;       // Session ID
  seq: number;             // Sequence number
  timestamp: number;       // Timestamp
  requestId?: string;      // Request ID (for request-response pairing)
  data: T;                 // Message data
}
```

**Message Types**:

| Type | Direction | Description |
|------|-----------|-------------|
| `hello` | Client → Server | Client handshake |
| `helloAck` | Server → Client | Server handshake acknowledgment |
| `step` | Client → Server | Send input, feedback, completion status |
| `action` | Server → Client | Return output |
| `reset` | Client → Server | Reset session |
| `query` | Client → Server | Single query (one-shot mode) |
| `response` | Server → Client | Query response |
| `ping` / `pong` | Bidirectional | Heartbeat check |
| `error` | Bidirectional | Error message |

### 2.3 Transport Layer

The AI interface provides three transport implementations:

| Transport Type | Usage Scenario | Description |
|----------------|----------------|-------------|
| `BrowserWebSocketTransport` | Browser Environment | Uses browser native WebSocket |
| `NodeWebSocketTransport` | Node.js Environment | Uses `ws` library |
| `InMemoryTransport` | Unit Testing | Direct in-memory communication |

---

## 3. Quick Start

### 3.1 Basic Usage Flow

```typescript
import { 
  IterativeAiClient, 
  type BoxSpace, 
  type DiscreteSpace,
  type DictSpace 
} from 'symbion/ai';

// 1. Define Observation Space
const observationSpace: DictSpace = {
  kind: 'dict',
  spaces: {
    position: { kind: 'box', shape: [3], low: 0, high: 1000 },
    velocity: { kind: 'box', shape: [3], low: -20, high: 20 },
    sinrDb: { kind: 'box', shape: [1], low: -20, high: 40 },
  },
};

// 2. Define Action Space
const actionSpace: DictSpace = {
  kind: 'dict',
  spaces: {
    movement: { kind: 'box', shape: [3], low: -1, high: 1 },
    mcs: { kind: 'discrete', n: 29 },
  },
};

// 3. Create Client
const client = new IterativeAiClient({
  host: 'localhost',
  port: 8765,
  inputSpace: observationSpace,    // or observationSpace for RL compatibility
  outputSpace: actionSpace,        // or actionSpace for RL compatibility
});

// 4. Connect to AI Server
await client.connect();
console.log('Connected to AI Server');

// 5. Reset environment, get initial action
const initialObs = {
  position: [100, 200, 50],
  velocity: [0, 0, 0],
  sinrDb: [15.5],
};
let action = await client.reset(initialObs);

// 6. Simulation Loop
let step = 0;
let done = false;
while (!done && step < 1000) {
  // Execute action, get new observation and reward
  const { observation, reward, terminated } = simulateStep(action);
  
  // Send observation, get next action
  action = await client.step(observation, reward, terminated);
  
  done = terminated;
  step++;
}

// 7. Disconnect
client.disconnect();
```

### 3.2 Python AI Server Example

```python
# ai_server.py
import asyncio
import websockets
import json
import time
import random

async def ai_agent(websocket, path):
    """Simple AI Agent Server"""
    async for message in websocket:
        msg = json.loads(message)
        
        if msg['type'] == 'hello':
            # Handshake response
            response = {
                'v': 1,
                'type': 'helloAck',
                'sessionId': msg['sessionId'],
                'seq': 0,
                'timestamp': time.time() * 1000,
                'requestId': msg.get('requestId'),
                'data': {
                    'serverVersion': '1.0.0',
                    'accepted': True,
                },
            }
            await websocket.send(json.dumps(response))
            
        elif msg['type'] == 'step' or msg['type'] == 'reset':
            # Simple random action
            action = {
                'movement': [random.uniform(-1, 1) for _ in range(3)],
                'mcs': random.randint(0, 28),
            }
            response = {
                'v': 1,
                'type': 'action',
                'sessionId': msg['sessionId'],
                'seq': msg['seq'] + 1,
                'timestamp': time.time() * 1000,
                'requestId': msg.get('requestId'),
                'data': {'action': action},
            }
            await websocket.send(json.dumps(response))

# Start server
start_server = websockets.serve(ai_agent, 'localhost', 8765)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
```

---

## 4. Detailed API Description

### 4.1 IterativeAiClient Class

Main client class for iterative AI interactions (step/reset pattern).

#### Constructor

```typescript
constructor(config: AiInterfaceConfig, transport?: Transport)
```

**Configuration Options**:

| Option | Type | Required | Description |
|--------|------|----------|-------------|
| `host` | `string` |  | WebSocket server host (Default 'localhost') |
| `port` | `number` |  | WebSocket server port (Default 8765) |
| `observationSpace` | `Space` |  | Observation Space definition |
| `actionSpace` | `Space` |  | Action Space definition |
| `timeoutMs` | `number` |  | Request timeout (Default 30000ms) |
| `heartbeatMs` | `number` |  | Heartbeat interval (Default 10000ms, 0 to disable) |
| `autoReconnect` | `boolean` |  | Whether to auto-reconnect (Default true) |
| `reconnectDelayMs` | `number` |  | Reconnect delay in milliseconds (Default 1000) |
| `strictStep` | `boolean` |  | Strict mode to prevent concurrent steps (Default true) |

#### Core Methods

##### `connect(): Promise<void>`

Connect to AI server and complete handshake.

```typescript
await client.connect();
// Can only call step/reset after successful connection
```

##### `disconnect(): void`

Disconnect from the server.

```typescript
client.disconnect();
```

##### `step(input, feedback, done, info?): Promise<Output>`

Send current input and feedback signal, wait for AI to return output.

> **Note**: In RL contexts, `input` corresponds to observation, `feedback` to reward, and output to action.

```typescript
const output = await client.step(
  { position: [100, 200, 50], velocity: [1, 0, 0] },  // input
  1.5,                                                  // feedback (reward in RL)
  false,                                                // done
  { episodeLength: 100 }                               // info (optional)
);
```

##### `reset(initialInput, info?): Promise<Output>`

Reset session state, send initial input, wait for first output.

```typescript
const firstOutput = await client.reset(
  { position: [0, 0, 50], velocity: [0, 0, 0] },
  { seed: 12345 }
);
```

##### `onEvent(callback): void`

Subscribe to client events.

```typescript
client.onEvent((event) => {
  switch (event.type) {
    case 'connected':
      console.log('Connected');
      break;
    case 'disconnected':
      console.log('Disconnected:', event.reason);
      break;
    case 'error':
      console.error('Error:', event.error);
      break;
  }
});
```

### 4.2 ProtocolHandler Class

Low-level protocol handler for custom protocol implementation.

```typescript
import { ProtocolHandler } from 'symbion/ai';

const protocol = new ProtocolHandler('session-123', 30000);

// Create message
const msg = protocol.createMessage('step', {
  observation: [...],
  reward: 1.5,
  done: false,
});

// Register request and wait for response
const requestId = protocol.generateRequestId();
const promise = protocol.registerRequest<Action>(requestId);

// Send message...

// When response arrives
protocol.handleMessage(responseMsg);
const action = await promise;  // Auto-resolved
```

### 4.3 Helper Functions

#### Observation Space Helpers

```typescript
import { 
  defaultUavObservation,
  minimalUavObservation,
  normalizeObservation,
} from 'symbion/ai';

// Standard UAV Observation Extraction
const obs = defaultUavObservation(uavState);
// Returns: { position, velocity, acceleration, power, sinr, bler, ... }

// Minimal Observation (Motion only)
const minObs = minimalUavObservation(uavState);
// Returns: { position, velocity }

// Normalize Observation
const normalizedObs = normalizeObservation(obs, observationSpace);
// Normalize observation values to [-1, 1] or [0, 1]
```

#### Action Space Helpers

```typescript
import {
  defaultUavActionMapper,
  findOptimalMcs,
} from 'symbion/ai';

// Map AI action to UAV control command
const control = defaultUavActionMapper(action);
// Returns: { thrust, roll, pitch, yaw, mcsIndex, txPower }

// Find optimal MCS based on SNR
const optimalMcs = findOptimalMcs(15.5, 0.1);  // snrDb, blerTarget
// Returns: 12 (MCS Index)
```

---

## 5. Practical Examples

### 5.1 Basic Training Loop

```typescript
import { IterativeAiClient, type BoxSpace } from 'symbion/ai';

async function runTraining() {
  // Config
  const obsSpace: BoxSpace = {
    kind: 'box',
    shape: [12],
    low: -Infinity,
    high: Infinity,
  };
  
  const actSpace: BoxSpace = {
    kind: 'box',
    shape: [4],
    low: [-1, -1, -1, 0],
    high: [1, 1, 1, 1],
  };
  
  const client = new IterativeAiClient({
    host: 'localhost',
    port: 8765,
    inputSpace: obsSpace,
    outputSpace: actSpace,
    timeoutMs: 60000,
  });
  
  await client.connect();
  
  const numEpisodes = 100;
  const maxSteps = 500;
  
  for (let episode = 0; episode < numEpisodes; episode++) {
    // Reset Env
    const initObs = getInitialObservation();
    let action = await client.reset(initObs, { episode });
    
    let totalReward = 0;
    
    for (let step = 0; step < maxSteps; step++) {
      // Execute Simulation Step
      const result = simulationStep(action);
      
      totalReward += result.reward;
      
      if (result.done) {
        // Send final state
        await client.step(result.observation, result.reward, true);
        break;
      }
      
      // Get next action
      action = await client.step(result.observation, result.reward, false);
    }
    
    console.log(`Episode ${episode}: Total Reward = ${totalReward.toFixed(2)}`);
  }
  
  client.disconnect();
}
```

### 5.2 Unit Testing with InMemory Transport

```typescript
import { 
  IterativeAiClient, 
  InMemoryTransport,
  FakeAgentServer,
  type BoxSpace,
} from 'symbion/ai';

describe('AI Environment Tests', () => {
  let client: IterativeAiClient;
  let server: FakeAgentServer;
  let transport: InMemoryTransport;
  
  beforeEach(() => {
    // Create memory transport pair
    const [clientTransport, serverTransport] = InMemoryTransport.createPair();
    transport = clientTransport;
    
    // Create fake agent server
    server = new FakeAgentServer(serverTransport);
    server.setActionStrategy((obs) => ({
      movement: [0.1, 0.2, 0.3],
      mcs: 15,
    }));
    
    // Create client
    client = new IterativeAiClient(
      {
        host: 'memory',
        port: 0,
        inputSpace: { kind: 'box', shape: [6], low: -1, high: 1 },
        outputSpace: { kind: 'box', shape: [4], low: -1, high: 1 },
      },
      transport
    );
  });
  
  afterEach(() => {
    client.disconnect();
    server.close();
  });
  
  test('should complete step cycle', async () => {
    await client.connect();
    
    const obs = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6];
    const action = await client.reset(obs);
    
    expect(action).toBeDefined();
    expect(action.movement).toHaveLength(3);
    expect(action.mcs).toBe(15);
  });
});
```

### 5.3 Custom Observation and Action Mapping

```typescript
import { IterativeAiClient, type DictSpace } from 'symbion/ai';

// Custom Observation Mapper
function customObservationMapper(state: SimulationState): Record<string, number[]> {
  return {
    // Normalize Position to [0, 1]
    position: [
      state.position.x / 1000,
      state.position.y / 1000,
      (state.position.z - 30) / 120,  // 30-150m Normalized
    ],
    // Normalize Velocity to [-1, 1]
    velocity: [
      state.velocity.x / 20,
      state.velocity.y / 20,
      state.velocity.z / 10,
    ],
    // Communication Metrics
    communication: [
      (state.sinrDb + 20) / 60,       // -20dB to 40dB → 0-1
      1 - state.bler,                  // Reliability
      Math.log10(state.throughput + 1) / 9,  // Log Normalized Throughput
    ],
    // Target Info
    target: [
      (state.targetPosition.x - state.position.x) / 500,
      (state.targetPosition.y - state.position.y) / 500,
      (state.targetPosition.z - state.position.z) / 100,
    ],
  };
}

// Custom Action Mapper
function customActionMapper(action: Record<string, unknown>): ControlCommand {
  const movement = action.movement as number[];
  const comm = action.communication as Record<string, number>;
  
  return {
    // Motion Control
    targetVelocity: {
      x: movement[0] * 20,  // De-normalize
      y: movement[1] * 20,
      z: movement[2] * 10,
    },
    // Communication Control
    mcsIndex: Math.round(comm.mcs * 28),
    txPowerDbm: comm.power * 23,  // 0-23 dBm
  };
}

// Usage
const client = new IterativeAiClient({
  host: 'localhost',
  port: 8765,
  inputSpace: {
    kind: 'dict',
    spaces: {
      position: { kind: 'box', shape: [3], low: 0, high: 1 },
      velocity: { kind: 'box', shape: [3], low: -1, high: 1 },
      communication: { kind: 'box', shape: [3], low: 0, high: 1 },
      target: { kind: 'box', shape: [3], low: -1, high: 1 },
    },
  },
  outputSpace: {
    kind: 'dict',
    spaces: {
      movement: { kind: 'box', shape: [3], low: -1, high: 1 },
      communication: { 
        kind: 'dict',
        spaces: {
          mcs: { kind: 'box', shape: [1], low: 0, high: 1 },
          power: { kind: 'box', shape: [1], low: 0, high: 1 },
        },
      },
    },
  },
});
```

---

## 6. FAQ & Best Practices

### 6.1 FAQ

#### Q: What if connection times out?

```typescript
// Increase timeout
const client = new IterativeAiClient({
  // ...
  timeoutMs: 60000,  // 60 seconds
});

// Add error handling
client.onEvent((event) => {
  if (event.type === 'error' && event.error.code === 'E_TIMEOUT') {
    console.warn('Request timeout, attempting reconnect...');
    // Handle timeout
  }
});
```

#### Q: How to handle disconnection and reconnection?

```typescript
const client = new IterativeAiClient({
  // ...
  reconnect: true,
  maxReconnectAttempts: 10,
  reconnectDelayMs: 1000,
});

client.onEvent((event) => {
  switch (event.type) {
    case 'reconnecting':
      console.log(`Reconnecting... (Attempt ${event.attempt})`);
      break;
    case 'reconnected':
      console.log('Reconnected successfully');
      break;
    case 'reconnectFailed':
      console.error('Reconnect failed, manual handling required');
      break;
  }
});
```

#### Q: What if observation space dimensions don't match?

Ensure TypeScript client and Python server use the same space definition:

```typescript
// Print space info on client for debugging
console.log('Input Space:', JSON.stringify(client.getInputSpace(), null, 2));
console.log('Output Space:', JSON.stringify(client.getOutputSpace(), null, 2));
```

### 6.2 Best Practices

1. **Use Type-Safe Space Definitions**
   - Always specify `kind`, `shape`, `low`, `high` explicitly
   - Use TypeScript types to ensure consistency

2. **Set Reasonable Timeouts**
   - Use longer timeouts for training (e.g., 60s)
   - Use shorter timeouts for inference (e.g., 5s)

3. **Observation Normalization**
   - Normalize all observations to [-1, 1] or [0, 1]
   - Use `normalizeObservation` helper function

4. **Error Handling**
   - Always add `onEvent` callback to handle errors
   - Implement retry mechanisms

5. **Performance Optimization**
   - Avoid frequent creation/destruction of connections
   - Batch observations if supported
   - Use `InMemoryTransport` for development testing

---

## Related Links

- [API Reference](../API_REFERENCE.md#1-ai-interface-ai)
- [ISAC Module Guide](./isac-module-guide.md)

---

*Document Version: 1.0 | Last Updated: 2025-12-27*
