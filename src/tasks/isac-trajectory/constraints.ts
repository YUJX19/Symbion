/**
 * @module tasks/isac-trajectory/task1/constraints
 * @description Constraint definitions with Core module integration
 * 
 * **Core Integration:**
 * - Uses `leConstraint`, `geConstraint` from core/constraint
 * - Uses `evaluateConstraints` for unified evaluation
 * - Separates hard (feasibility) from soft (penalty) constraints
 */

import { type ConstraintSpec, type ConstraintReport, leConstraint, geConstraint, evaluateConstraints } from '../../core/constraint';

import type { Task1Config } from './config';
import type { OptimizationState } from './types';
import { distance3D, getTargetAtTime } from './utils';
import { calculateLosProbabilityUAV } from './uav-comm-model';

// ==========================================
// Constraint Creation
// ==========================================

/**
 * Create ConstraintSpecs for Task1
 * 
 * **Core Module Value:**
 * - Declarative constraint definition with `leConstraint`/`geConstraint`
 * - Clear separation of hard (feasibility) vs soft (penalty)
 * - Unified evaluation via `evaluateConstraints`
 */
export function createConstraints(cfg: Task1Config): ConstraintSpec<OptimizationState>[] {
    return [
        // Velocity Constraint (soft)
        leConstraint<OptimizationState>(
            'max_velocity',
            (state) => Math.max(...state.samples.map(s => s.speed)),
            cfg.maxVelocity,
            'soft',
            {
                description: `Max Velocity <= ${cfg.maxVelocity} m/s`,
                unit: 'm/s',
                penaltyWeight: cfg.weights.dynamics,
            }
        ),

        // Acceleration Constraint (soft)
        leConstraint<OptimizationState>(
            'max_acceleration',
            (state) => Math.max(...state.samples.map(s => s.accMagnitude)),
            cfg.maxAcceleration,
            'soft',
            {
                description: `Max Acceleration <= ${cfg.maxAcceleration} m/s²`,
                unit: 'm/s²',
                penaltyWeight: cfg.weights.dynamics,
            }
        ),

        // Obstacle Clearance Constraint (HARD - feasibility)
        geConstraint<OptimizationState>(
            'min_obstacle_clearance',
            (state) => {
                let minClear = Infinity;
                for (const sample of state.samples) {
                    for (const obs of state.obstacles) {
                        const d = distance3D(sample.position, obs.center as [number, number, number]);
                        minClear = Math.min(minClear, d - obs.radius);
                    }
                }
                return minClear;
            },
            cfg.safetyMargin,
            'hard',
            {
                description: `Obstacle Clearance >= ${cfg.safetyMargin} m`,
                unit: 'm',
            }
        ),

        // Min Communication Distance (soft)
        geConstraint<OptimizationState>(
            'min_comm_distance',
            (state) => {
                let minDist = Infinity;
                for (const sample of state.samples) {
                    const target = getTargetAtTime(
                        state.targetPath as [number, number, number][],
                        sample.t,
                        state.totalTime
                    );
                    minDist = Math.min(minDist, distance3D(sample.position, target));
                }
                return minDist;
            },
            cfg.minCommDistance,
            'soft',
            {
                description: `Comm Distance >= ${cfg.minCommDistance} m`,
                unit: 'm',
                penaltyWeight: cfg.weights.distance,
            }
        ),

        // Max Communication Distance (soft)
        leConstraint<OptimizationState>(
            'max_comm_distance',
            (state) => {
                let maxDist = 0;
                for (const sample of state.samples) {
                    const target = getTargetAtTime(
                        state.targetPath as [number, number, number][],
                        sample.t,
                        state.totalTime
                    );
                    maxDist = Math.max(maxDist, distance3D(sample.position, target));
                }
                return maxDist;
            },
            cfg.maxCommDistance,
            'soft',
            {
                description: `Comm Distance <= ${cfg.maxCommDistance} m`,
                unit: 'm',
                penaltyWeight: cfg.weights.distance,
            }
        ),

        // Min LoS Probability (soft - URLLC)
        geConstraint<OptimizationState>(
            'min_los_probability',
            (state) => {
                let totalLoS = 0;
                for (const sample of state.samples) {
                    const target = getTargetAtTime(
                        state.targetPath as [number, number, number][],
                        sample.t,
                        state.totalTime
                    );
                    const dist = distance3D(sample.position, target);
                    totalLoS += calculateLosProbabilityUAV(dist, sample.position[2], target[2]);
                }
                return totalLoS / state.samples.length;
            },
            cfg.urllcLosThreshold,
            'soft',
            {
                description: `Avg LoS Probability >= ${cfg.urllcLosThreshold * 100}%`,
                penaltyWeight: cfg.weights.urllc,
            }
        ),
    ];
}

// ==========================================
// Constraint Evaluation
// ==========================================

/**
 * Evaluate all constraints for a state
 */
export function evaluateAllConstraints(
    cfg: Task1Config,
    state: OptimizationState
): ConstraintReport {
    const constraints = createConstraints(cfg);
    return evaluateConstraints(constraints, state);
}

/**
 * Check if all hard constraints are satisfied
 */
export function isHardFeasible(report: ConstraintReport): boolean {
    return report.feasible;
}

/**
 * Validate and throw if hard constraints violated
 */
export function validateOrThrow(report: ConstraintReport): void {
    if (!report.feasible) {
        const violated = report.results
            .filter(r => !r.satisfied && r.severity === 'hard')
            .map(r => `${r.id}: ${r.value.toFixed(3)} ${r.operator} ${r.threshold}`);

        throw new Error(
            `Hard constraint violation:\n  ${violated.join('\n  ')}`
        );
    }
}

/**
 * Format constraint report for display
 */
export function formatConstraintReport(report: ConstraintReport): string {
    const lines: string[] = [];
    lines.push(`Feasible: ${report.feasible ? 'YES' : 'NO'}`);
    lines.push(`Total Penalty: ${report.totalPenalty.toFixed(2)}`);
    lines.push('');
    lines.push('Constraints:');

    for (const r of report.results) {
        const icon = r.satisfied ? '[OK]' : '[X]';
        const status = r.satisfied ? 'OK' : `VIOLATED (${r.violation.toFixed(3)})`;
        lines.push(`  ${icon} ${r.id}: ${r.value.toFixed(3)} ${r.operator} ${r.threshold} → ${status}`);
    }

    return lines.join('\n');
}
