<div align="center">
  <img src="assets/logo.svg" height="120" alt="Symbion Logo">
  <h1>Symbion</h1>
</div>

> An open-source TypeScript library for reproducible Integrated Sensing and Communication (ISAC) research (Node.js + browser)

[![NPM Version](https://img.shields.io/npm/v/symbion)](https://www.npmjs.com/package/symbion)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![TypeScript](https://img.shields.io/badge/TypeScript-5.0+-blue.svg)](https://www.typescriptlang.org/)
[![CI](https://github.com/YUJX19/Symbion/actions/workflows/ci.yml/badge.svg)](https://github.com/YUJX19/Symbion/actions/workflows/ci.yml)

**Symbion** is an open-source TypeScript library for **reproducible end-to-end ISAC** research. It supports a “simulation-as-code” workflow that combines:

- **Communication & PHY utilities** (channel models, beamforming helpers, modulation/coding building blocks)
- **Mobility, planning, and robotics dynamics utilities** (trajectory generation, kinematics/dynamics helpers)
- **Research infrastructure** (typed spaces, objectives/constraints, deterministic RNG, structured logging, experiment runner)
- **Framework-agnostic AI agent integration** via a stable **WebSocket protocol** (for connecting external AI backends — Python, RL agents, LLMs, etc.)
- **Two reference tasks/benchmarks**:
  - **U2U-MCS**: 5G NR sidelink dynamic MCS selection (UAV-to-UAV)
  - **ISAC Trajectory**: UAV trajectory optimization with LoS persistence and URLLC-style constraints

> Symbion is designed to complement (not replace) large-scale network/PHY simulators. Its core focus is **portability + reproducibility + end-to-end ISAC workflows**.

---

## Install

```bash
npm install symbion
````

---

## Quick start

```ts
import { channel, planning, core } from 'symbion';

// 1) Shannon capacity under AWGN (Shannon limit)
const capacity = channel.shannonCapacity(10, 20e6); // 10 dB SNR, 20 MHz
console.log(`Shannon Capacity: ${(capacity / 1e6).toFixed(2)} Mbps`);

// 2) Minimum-snap trajectory (MINCO-style)
const trajectory = planning.generateMinimumSnapTrajectory(
  [
    { position: [0, 0, 0], time: 0 },
    { position: [10, 5, 2], time: 2 },
  ],
  { maxVelocity: 5, maxAcceleration: 3 }
);
console.log('Trajectory waypoints:', trajectory.waypoints.length);

// 3) Deterministic RNG (same seed => identical sequence)
const rng = core.createRng(42);
console.log('Deterministic random:', rng.random());
```

> Prefer fine-grained imports when you want explicit namespaces / smaller bundles:
>
> ```ts
> import { channel, planning } from 'symbion/models';
> import { Runner, createRng } from 'symbion/core';
> import { createPlanner } from 'symbion/isac';
> import { IterativeAiClient } from 'symbion/ai';
> ```

---

## What Symbion provides

### Modules at a glance

| Package entry    | Purpose                                                                                 | Browser | Node.js | Stability                          |
| ---------------- | --------------------------------------------------------------------------------------- | ------: | ------: | ---------------------------------- |
| `symbion/core`   | Spaces, objectives, constraints, runner, logging, reproducibility primitives            |       ✅ |       ✅ | **Stable**                         |
| `symbion/models` | Physics/model layer: channel/beamforming/PHY + robotics/planning/sensing helpers        |       ✅ |       ✅ | **Stable / evolving by submodule** |
| `symbion/isac`   | ISAC coupling layer: metric templates + planners linking comm metrics and mobility      |       ✅ |       ✅ | **Stable**                         |
| `symbion/ai`     | Framework-agnostic AI interface (typed spaces + WebSocket protocol/transport)          |       ✅ |       ✅ | **Stable**                         |
| `symbion/tasks`  | Reference research environments (U2U-MCS, ISAC Trajectory) + benchmark scripts          |      ❌* |       ✅ | **Stable (research API)**          |
| `symbion/extras` | Experimental networking + advanced coding re-exports + sandbox utilities                |       ✅ |       ✅ | ⚠️ **Experimental**                |

* `tasks` requires Node.js due to file I/O and CLI/benchmark workflows.

> **Stability note**: “Stable” describes API compatibility expectations. It does **not** claim that every model is exhaustive or that it replaces specialized simulators.

---

## Reference tasks & benchmarks

Symbion includes two research tasks demonstrating reproducible workflows:

### 1) U2U-MCS: Dynamic MCS Selection

A sequential decision-making task for 5G NR sidelink communication.

Run the simple example:

```bash
npx tsx benchmarks/u2u-mcs/simple-example.ts
```

Run a baseline benchmark:

```bash
npx tsx benchmarks/u2u-mcs/run-baseline.ts --config=baseline-high-speed.json --policy=greedy
```

### 2) ISAC Trajectory: UAV Path Optimization

A constrained optimization task for trajectory planning with communication constraints.

Run the simple example:

```bash
npx tsx benchmarks/isac-los-urllc/simple-example.ts
```

Run a baseline benchmark:

```bash
npx tsx benchmarks/isac-los-urllc/run-baseline.ts --config=baseline-suburban.json --policy=proposed
```

For detailed problem formulations, solution approaches, and baseline algorithms, see:

* [`docs/tasks-user-guide.md`](https://github.com/YUJX19/Symbion/blob/main/docs/tasks-user-guide.md)

---

## Browser vs Node.js usage

Symbion is designed to run in **both** modern browsers and Node.js:

* **Browser**: use Symbion for models/visual demos/interactive education without proprietary toolchains.
* **Node.js**: use the full stack including `tasks` benchmarks, file-based logging, and CLI workflows.

> **Important**: the `tasks` module is **Node-only** (file I/O + CLI scripts). Use the browser entry point for web apps.

### Recommended (bundlers with conditional exports)

Modern bundlers (Webpack, Vite, esbuild, etc.) automatically select the correct entry point based on your build target:

```ts
// Same import statement works in both environments:
// - Browser builds → resolves to dist/browser.mjs (excludes Node-only 'tasks')
// - Node.js builds → resolves to dist/node.mjs (full functionality)
import { channel, planning, isac } from 'symbion';
```

### Explicit entry points (when you want to force the runtime)

```ts
// Force browser-only bundle (excludes 'tasks' module)
import { channel, planning, isac } from 'symbion/browser';

// Force Node.js bundle (includes all features)
import { channel, planning, tasks } from 'symbion/node';
```

---

## Reproducibility features

Symbion’s core research infrastructure is designed for **traceable experiments**:

* **Seeded RNG** for deterministic stochasticity
* **Schema hashing** for input/output space compatibility auditing
* **Task configuration fingerprints** for tracking configuration changes
* **Structured logs** (JSONL/CSV) for step/episode/report-level analysis

Minimal reproducibility setup example:

```ts
import { Runner, ConsoleLogger, createTaskConfig, computeSchemaHash } from 'symbion/core';

const taskConfig = createTaskConfig({
  taskName: 'u2u-mcs',
  seed: 42,
  schemaHash: computeSchemaHash(env.inputSpace),
  hyperparams: { episodeLength: 1000, blerTarget: 1e-5 },
});

const runner = new Runner({
  taskConfig,
  environment: env,
  policy,
  loggers: [new ConsoleLogger('info')],
});

await runner.run();
```

---

## AI agent integration (framework-agnostic)

Symbion provides an **AI interface module** that uses a **versioned JSON WebSocket protocol** plus multiple transports (browser, Node, in-memory). This keeps the simulator decoupled from any specific AI/ML framework while supporting iterative interaction loops via a standardized step/reset pattern.

See:

* [`docs/guides/ai-module-guide.md`](https://github.com/YUJX19/Symbion/blob/main/docs/guides/ai-module-guide.md)

---

## Positioning vs related tools (high-level)

This section is meant to clarify scope (not to “rank” tools).

| Capability (out-of-the-box)                                  | MATLAB toolboxes |     ns-3 |        Sionna | **Symbion** |
| ------------------------------------------------------------ | ---------------: | -------: | ------------: | ----------: |
| Open-source core distribution                                |                ❌ |        ✅ |             ✅ |           ✅ |
| TypeScript / JavaScript native                               |                ❌ |        ❌ |             ❌ |           ✅ |
| Native **in-browser** runtime                                |                ❌ |        ❌ |             ❌ |           ✅ |
| Framework-agnostic AI control plane (e.g., via WebSocket)    |    Tool-specific | Partial* | Tool-specific |           ✅ |
| End-to-end ISAC workflow templates (mobility ↔ comm metrics) |          Partial |  Partial |       Partial |           ✅ |

\* Achievable via common integrations, but typically not a single cohesive, in-core workflow.

---

## Requirements

### Runtime

* **Node.js**: ≥ 18 (required for `tasks` and CLI/benchmarks)
* **Browser**: modern browsers with ES2020+ support

### Development

* **TypeScript**: ≥ 5.0

---

## Repository development

```bash
git clone https://github.com/YUJX19/Symbion.git
cd Symbion

npm install
npm run build
npm test
```

---

## Documentation

* **API Reference**: [`docs/API_REFERENCE.md`](https://github.com/YUJX19/Symbion/blob/main/docs/API_REFERENCE.md)
* **Module guides**: [`docs/guides/`](https://github.com/YUJX19/Symbion/tree/main/docs/guides/)
* **Tasks User Guide**: [`docs/tasks-user-guide.md`](https://github.com/YUJX19/Symbion/blob/main/docs/tasks-user-guide.md)

---


## Contributing

Contributions are welcome.

* Please read [`CONTRIBUTING.md`](https://github.com/YUJX19/Symbion/blob/main/CONTRIBUTING.md)


---

## Citation

If you use Symbion in research, please cite a versioned archive (recommended for reproducibility). A `CITATION.cff` file is included for convenience.

---

## License

MIT License — see [`LICENSE`](https://github.com/YUJX19/Symbion/blob/main/LICENSE).