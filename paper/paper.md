---
title: "Symbion: An Open-Source TypeScript Library for Integrated Sensing and Communication Research"
tags:
  - integrated sensing and communication
  - reproducible research
  - simulation
  - UAV
authors:
  - name: Jingxiang Yu
    orcid: 0000-0001-9417-5669
    equal-contrib: true
    corresponding: true
    affiliation: 1
  - name: Hong Jiang
    orcid: 0000-0001-5994-6595
    equal-contrib: true
    affiliation: 1
affiliations:
  - name: Information College of Southwest University of Science and Technology, China
    index: 1
date: 2025-12-27
bibliography: paper.bib
---

# Summary

Symbion is an open source TypeScript library that enables reproducible, end-to-end(E2E) experimentation for integrated sensing and communication (ISAC). ISAC studies require joint evaluation of wireless communication performance, sensing metrics, and mobility or trajectory decisions under kinematic constraints. Existing research pipelines are commonly fragmented across heterogeneous tools, languages, and runtimes, which increases integration overhead and hinders reproducibility across code revisions, random seeds, and configuration changes. Symbion introduces a simulation as code workflow in JavaScript and TypeScript and provides reproducibility primitives including deterministic execution, configuration fingerprints, interface hashing, and structured logs. The library runs in Node.js and modern browsers, enabling lightweight deployment and interactive demonstrations without proprietary toolchains.

# Statement of need

ISAC is inherently cross disciplinary because communication objectives including reliability, latency, and throughput are coupled with motion feasibility, time varying LoS conditions, and system level decisions such as link adaptation and scheduling [@liuIntegratedSensingCommunications2022a]. Consequently, researchers often combine multiple ecosystems spanning network simulation, robotics and optimization [@liuObjectOrientedIntegratedSensing2025], and machine learning[@zhaoLearningBeamformIntegrated2025]. Although general purpose simulators such as ns-3 [@rileyNs3NetworkSimulator2010] and specialized domain toolkits provide substantial functionality, constructing a portable workflow that couples mobility decisions with communication and sensing metrics [@liJointDesignCommunication2024], supports controlled benchmarking and ablation, and remains easy to share and demonstrate in a browser typically requires extensive glue code and supporting infrastructure.

Symbion targets researchers and engineers who require a single portable codebase for ISAC prototyping and benchmarking. It offers a unified experiment interface that integrates mobility evaluation with communication objectives, and it records configuration fingerprints and structured logs to enable traceable comparisons across commits and parameter sweeps. Symbion further provides a framework agnostic agent interface that allows external decision makers, including Python based learning agents, to control Symbion tasks without binding the simulator to a specific learning framework or programming language.

# Approach

![Architecture overview.\label{fig:v3}](architecture.png)

Fig.\ref{fig:v3} shows an architectural overview of Symbion’s layered design and its execution environments in Node.js and modern browsers, highlighting the WebSocket-based interface to external agents and the mechanisms used to generate reproducibility artifacts, including interface and configuration hashes and structured logs. Symbion is organized into a small set of composable modules:

- **Core framework**: typed observation and action spaces with validation and reversible flattening; schema hashes that detect incompatible interface changes; a runner that standardizes environment and policy interaction; a metric and constraint system for multi objective evaluation; and structured logging at step level, episode level, and report level data.
- **ISAC coupling module**: a metric-driven coupling layer that jointly evaluates trajectory and communication performance. It supports user-defined metrics and constraints and includes reusable metric templates as reference implementations for common objectives such as LoS persistence, throughput, and reliability. The module performs constraint checking, implements LoS and propagation modeling, and enables multi-objective evaluation without requiring users to collapse objectives into a single scalar reward.
- **AI interface module**: a versioned protocol, commonly implemented over WebSocket, that allows external agents to control Symbion environments while enforcing shared space contracts. This design supports integration with the external learning framework.
- **Models**: reusable building blocks for wireless channels and physical layer processing, array and beamforming utilities, UAV kinematics and planning, sensing abstractions, and numerical utilities.

# Reference tasks and benchmark use cases

Symbion currently includes two runnable reference tasks that illustrate E2E workflows and provide reproducible baselines for controlled benchmarking and ablation, while remaining extensible to additional tasks.

**ISAC trajectory communication aware UAV mobility.**
This task couples UAV trajectory decisions with communication and LoS objectives, following the spatiotemporal trajectory design setting of Yu et al. [@yuSpatioTemporalTrajectoryDesign2024]. At each step, the agent selects a motion action subject to kinematic constraints, and Symbion evaluates LoS persistence and reliability metrics along the trajectory.


**U2U-MCS dynamic sidelink link adaptation.**
This task models a UAV-to-UAV (U2U) 5G New Radio sidelink scenario in which an agent selects an MCS at each transmission time interval to maximize throughput subject to reliability constraints. The task is inspired by the dynamic MCS scheduling setup in ns3-uavlink [@yuNs3uavlinkAIDrivenDynamic2025]. Symbion provides baseline policies including fixed, greedy, block error rate adaptive, and outer loop link adaptation, and it produces structured logs that support analysis of throughput and reliability trade offs.


# Reproducibility and quality assurance

Symbion emphasizes traceable and comparable experiments. Experiments are configured with a schema hash, random seed, and library version, and the full configuration is returned with results to support reproducibility verification. Structured logs recorded at step and episode granularity enable audits, ablation studies, and statistical analysis. Automated tests and continuous integration validate core abstractions and reference tasks across supported runtimes.

# Availability

Symbion is released under the MIT license and distributed via npm. The source code and documentation are hosted in a public repository, and versioned releases can be archived to ensure long term accessibility and reproducible citation.


# references