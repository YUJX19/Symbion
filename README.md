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

## Getting started

For installation instructions, quick start guides, API reference, and comprehensive tutorials, visit the **[official documentation](https://www.symbion-io.org/docs)**.

---

## Key features

- **Dual runtime**: Works in both Node.js (≥18) and modern browsers
- **Reproducible experiments**: Deterministic RNG, schema hashing, structured logging
- **Framework-agnostic AI integration**: WebSocket protocol for external agents
- **Reference tasks**: U2U-MCS sidelink and ISAC trajectory benchmarks
- **Rich model library**: Channel models, beamforming, robotics, sensing utilities

---

## Repository development

```bash
git clone https://github.com/YUJX19/Symbion.git
cd Symbion

npm install
npm run build
npm test
```

## Next steps

- [ ] **Physical flight controller integration**: Couple realistic UAV dynamics with ISAC simulation for hardware-in-the-loop validation

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