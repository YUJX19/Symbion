# Symbion Usage Guides

This directory contains detailed usage guides for the core modules of the Symbion library.

## Available Guides

| Guide | Description |
|-------|-------------|
| [AI Interface Module](./ai-module-guide.md) | The bridge connecting Symbion simulator with Python AI frameworks |
| [ISAC Framework Module](./isac-module-guide.md) | Joint trajectory optimization framework integrating sensing and communication |
| [Core Framework Module](./core-module-guide.md) | Infrastructure layer for reproducible experiments (Space, Objective, Constraint, Runner, Logging, Repro, Errors) |
| [Models Module](./models-module-guide.md) | Physics model layer (Channel, Beamforming, PHY, Robotics, Planning, Sensing, Numeric) |
| [Extras Module](./extras-module-guide.md) | Experimental modules (Networking, PHY-Coding, Sandbox2D) |
| [Tasks User Guide](../tasks-user-guide.md) | Research tasks: U2U-MCS and ISAC Trajectory |

## Quick Start

### Installation

```bash
npm install symbion
```

### Module Import Patterns

| Import Pattern | Description |
|----------------|-------------|
| `import symbion from 'symbion'` | Full library (bundled) |
| `import { ... } from 'symbion/core'` | Core utilities (Space, Logging, Runner) |
| `import { ... } from 'symbion/models'` | Physics models (Channel, Beamforming, Planning) |
| `import { ... } from 'symbion/tasks'` | Research tasks (U2U-MCS, ISAC) |
| `import { ... } from 'symbion/ai'` | AI/RL interface |
| `import { ... } from 'symbion/isac'` | ISAC framework |

### Running Tests

```bash
# Full test suite (typecheck + lint + unit tests + build + examples)
npm run test:all

# Quick test (skip benchmark examples)
npm run test:quick

# Unit tests only
npm run test
```

## Quick Navigation

### AI Module

- [Module Overview](./ai-module-guide.md#1-module-overview)
- [Core Concepts](./ai-module-guide.md#2-core-concepts)
- [Quick Start](./ai-module-guide.md#3-quick-start)
- [Detailed API](./ai-module-guide.md#4-detailed-api-description)
- [Practical Examples](./ai-module-guide.md#5-practical-examples)

### ISAC Module

- [Module Overview](./isac-module-guide.md#1-module-overview)
- [Core Components](./isac-module-guide.md#2-core-components)
- [Quick Start](./isac-module-guide.md#3-quick-start)
- [Configuration Guide](./isac-module-guide.md#4-configuration-guide)
- [Practical Examples](./isac-module-guide.md#5-practical-examples)

### Core Module

- [Module Overview](./core-module-guide.md#1-module-overview)
- [Space Definition](./core-module-guide.md#2-space---space-definition)
- [Objective System](./core-module-guide.md#3-objective---metric-system)
- [Constraint System](./core-module-guide.md#4-constraint---constraint-system)
- [Runner Experiment Runner](./core-module-guide.md#5-runner---experiment-runner)
- [Logging System](./core-module-guide.md#6-logging---logging-system)
- [Repro Reproducibility](./core-module-guide.md#7-repro---reproducibility)
- [Errors Error Handling](./core-module-guide.md#8-errors---error-handling)

### Models Module

- [Module Overview](./models-module-guide.md#1-module-overview)
- [Channel Models](./models-module-guide.md#2-channel---channel-models)
- [Beamforming](./models-module-guide.md#3-beamforming---beamforming)
- [PHY Layer](./models-module-guide.md#4-phy---physical-layer)
- [Robotics](./models-module-guide.md#5-robotics---robotics)
- [Planning Path Planning](./models-module-guide.md#6-planning---path-planning)
- [Sensing](./models-module-guide.md#7-sensing---sensing)
- [Numeric Methods](./models-module-guide.md#8-numeric---numerical-methods)

### Tasks Module

- [Overview](../tasks-user-guide.md#overview)
- [U2U-MCS Task](../tasks-user-guide.md#u2u-mcs-task)
- [ISAC Trajectory Task](../tasks-user-guide.md#isac-trajectory-task)
- [Advanced Topics](../tasks-user-guide.md#advanced-topics)

### Extras Module

- [Module Overview](./extras-module-guide.md#1-module-overview)
- [Networking Protocols](./extras-module-guide.md#2-networking---network-protocols)
- [PHY-Coding Advanced Coding](./extras-module-guide.md#3-phy-coding---advanced-channel-coding)
- [Sandbox2D Simulation Sandbox](./extras-module-guide.md#4-sandbox2d---2d-simulation-sandbox)

## Other Resources

- [API Reference](../API_REFERENCE.md) - Complete API Reference
- [README](../../README.md) - Project Overview
- [Contribution Guide](../../CONTRIBUTING.md) - How to Contribute Code
- [Test Script](../../scripts/test-all.sh) - Unified test runner

---

*Last Updated: 2025-12-27 | Version 1.0.0*
