/**
 * @module tasks/isac-trajectory/task1/types
 * @description Shared type definitions for Task1
 * 
 * Integrates with Symbion Core framework:
 * - TaskConfig from core/repro
 * - ExperimentMetadata from core/repro
 * - ConstraintReport from core/constraint
 */

import type { PiecewiseTrajectory, Vector3Array } from '../../models/planning/trajectory/types';
import type { MinimumJerkTrajectory } from '../../models/planning/planning/minco';
import type { ConstraintReport } from '../../core/constraint';
import type { TaskConfig, ExperimentMetadata } from '../../core/repro';
import type { Task1Config, SphereObstacle } from './config';

// ==========================================
// Trajectory Sample
// ==========================================

/** Sample point with full kinematic state */
export interface TrajectorySample {
    t: number;
    position: [number, number, number];
    velocity: [number, number, number];
    acceleration: [number, number, number];
    speed: number;
    accMagnitude: number;
}

// ==========================================
// Optimization State
// ==========================================

/**
 * Complete state for cost/constraint evaluation
 */
export interface OptimizationState {
    traj: PiecewiseTrajectory;
    minco: MinimumJerkTrajectory;
    samples: TrajectorySample[];
    waypoints: Vector3Array[];
    durations: number[];
    targetPath: Vector3Array[];
    obstacles: SphereObstacle[];
    totalTime: number;
    dt: number;
}

// ==========================================
// Cost Breakdown
// ==========================================

/**
 * Detailed cost decomposition (Paper Eq. 35-50)
 * 
 * All values are time-normalized by dt for stable weights.
 */
export interface CostBreakdown {
    jerk: number;       // J_o = ∫||r^(3)(t)||² dt (analytical)
    obstacle: number;   // P1 obstacle part (×dt)
    dynamics: number;   // P1 velocity/acceleration part (×dt)
    distance: number;   // H1 distance window (×dt)
    throughput: number; // Negative for maximization (×dt)
    urllc: number;      // URLLC reliability penalty (×dt)
    total: number;      // Weighted sum
}

// ==========================================
// Performance Metrics
// ==========================================

/**
 * Task1-specific metrics for evaluation
 */
export interface Task1Metrics {
    jerkEnergy: number;
    throughputMbps: number;
    losProbPct: number;
    maxVelocity: number;
    maxAcceleration: number;
    minObstacleClearance: number;
}

// ==========================================
// Optimizer Report
// ==========================================

/**
 * Optimization result summary
 */
export interface OptimizerReport {
    status: 'converged' | 'max_iterations' | 'failed';
    iterations: number;
    costHistory: number[];
    cacheStats?: {
        hits: number;
        misses: number;
        hitRate: number;
    };
}

// ==========================================
// Task1 Result (Main Output)
// ==========================================

/**
 * Complete Task1 result with full metadata
 * 
 * **Core Integration:**
 * - `taskConfig`: Reproducibility metadata (seed, schema, version)
 * - `experimentMetadata`: Runtime environment info
 */
export interface Task1Result {
    // Configuration
    config: Task1Config;

    // Initial State
    initial: {
        cost: CostBreakdown;
        constraints: ConstraintReport;
        metrics: Task1Metrics;
    };

    // Final State
    final: {
        waypoints: Vector3Array[];
        traj: PiecewiseTrajectory;
        cost: CostBreakdown;
        constraints: ConstraintReport;
        metrics: Task1Metrics;
        sampleCount: number;
    };

    // Optimizer
    optimizer: OptimizerReport;

    // Core Framework Integration
    taskConfig: TaskConfig;
    experimentMetadata: ExperimentMetadata;
}
