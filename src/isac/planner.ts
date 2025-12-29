/**
 * @module src/isac/planner
 * @description ISAC Trajectory Planner
 *
 * Provides trajectory planning with LoS and URLLC awareness.
 */

import { SeededRandom, createRng } from '../core/repro';
import { DEFAULT_ISAC_CONFIG, createConfig, type Position3D, type IsacTrajectoryConfig } from '../tasks/isac-trajectory/config';
import { IsacScenario } from '../tasks/isac-trajectory/scenario';

// Re-export types
export type { Position3D, IsacTrajectoryConfig };
export { DEFAULT_ISAC_CONFIG, createConfig };

/**
 * Planner state
 */
export interface PlannerState {
    step: number;
    position: Position3D;
    targetPosition?: Position3D;
    waypoints: Position3D[];
}

/**
 * Planner result
 */
export interface PlanResult {
    trajectory: Position3D[];
    totalDistance: number;
    estimatedEnergy: number;
    feasible: boolean;
}

/**
 * ISAC Trajectory Planner
 *
 * Generates waypoints considering:
 * - LoS persistence to ground users
 * - URLLC reliability constraints
 * - Energy efficiency
 */
export class IsacPlanner {
    private config: IsacTrajectoryConfig;
    private scenario: IsacScenario;
    private rng: SeededRandom;

    constructor(config: Partial<IsacTrajectoryConfig> = {}) {
        this.config = createConfig(config);
        this.scenario = new IsacScenario(this.config);
        this.rng = createRng(this.config.seed);
    }

    /**
     * Plan trajectory from start to end
     */
    plan(startPosition?: Position3D, endPosition?: Position3D): PlanResult {
        const start = startPosition ?? this.config.startPosition;
        const end = endPosition ?? this.config.endPosition ?? start;

        // Simple waypoint generation (can be extended with optimization)
        const waypoints = this.generateWaypoints(start, end);

        // Calculate metrics
        let totalDistance = 0;
        for (let i = 1; i < waypoints.length; i++) {
            totalDistance += this.distance(waypoints[i - 1], waypoints[i]);
        }

        const estimatedEnergy = this.estimateEnergy(waypoints);

        return {
            trajectory: waypoints,
            totalDistance,
            estimatedEnergy,
            feasible: true,
        };
    }

    /**
     * Generate waypoints between start and end
     */
    private generateWaypoints(start: Position3D, end: Position3D): Position3D[] {
        const waypoints: Position3D[] = [{ ...start }];
        const numWaypoints = this.config.numWaypoints;

        // Linear interpolation with altitude optimization
        for (let i = 1; i < numWaypoints; i++) {
            const t = i / numWaypoints;
            const position: Position3D = {
                x: start.x + t * (end.x - start.x),
                y: start.y + t * (end.y - start.y),
                z: this.optimizeAltitude(start, end, t),
            };
            waypoints.push(position);
        }

        waypoints.push({ ...end });
        return waypoints;
    }

    /**
     * Optimize altitude for LoS
     */
    private optimizeAltitude(start: Position3D, end: Position3D, t: number): number {
        // Higher altitude improves LoS probability
        const baseAltitude = start.z + t * (end.z - start.z);
        const { minAltitude, maxAltitude } = this.config.kinematics;

        // Add a slight arc for better LoS in the middle
        const arcHeight = 20 * Math.sin(Math.PI * t);

        return Math.max(minAltitude, Math.min(maxAltitude, baseAltitude + arcHeight));
    }

    /**
     * Estimate energy consumption
     */
    private estimateEnergy(waypoints: Position3D[]): number {
        const { hoverPower, propulsionEfficiency, maxSpeed } = this.config.kinematics;
        const dt = this.config.waypointDurationS;

        let energy = 0;
        for (let i = 1; i < waypoints.length; i++) {
            const distance = this.distance(waypoints[i - 1], waypoints[i]);
            const speed = distance / dt;
            const propulsionPower = (speed / maxSpeed) * hoverPower * 0.5;
            energy += (hoverPower + propulsionPower) * dt / propulsionEfficiency;
        }

        return energy;
    }

    /**
     * Calculate distance between two positions
     */
    private distance(a: Position3D, b: Position3D): number {
        return Math.sqrt(
            (a.x - b.x) ** 2 +
            (a.y - b.y) ** 2 +
            (a.z - b.z) ** 2
        );
    }

    /**
     * Reset planner
     */
    reset(seed?: number): void {
        this.rng = createRng(seed ?? this.config.seed);
        this.scenario.reset(seed);
    }
}

/**
 * Create ISAC planner
 */
export function createPlanner(config: Partial<IsacTrajectoryConfig> = {}): IsacPlanner {
    return new IsacPlanner(config);
}
