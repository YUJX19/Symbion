/**
 * @module tasks/isac-trajectory/policies
 * @description Baseline policies (planners) for ISAC trajectory task
 */

import type { DecisionMaker } from '../../core/runner';
import type { SeededRandom } from '../../core/repro';
import type { IsacInput } from './observation';
import type { IsacTrajectoryConfig, Position3D } from './config';

// ==================== Types ====================

/**
 * Planner context (additional info for planning)
 */
export interface PlannerContext {
    currentPosition: Position3D;
    targetPosition?: Position3D;
    userPositions: Position3D[];
    obstaclePositions: Position3D[];
}

// ==================== Hover Policy ====================

/**
 * Hover Policy: Stay at current position
 */
export class HoverPolicy implements DecisionMaker<IsacInput, number[]> {
    readonly name = 'hover';

    decide(_input: IsacInput, _rng: SeededRandom): number[] {
        return [0, 0, 0]; // No movement
    }

    reset(): void {
        // No state to reset
    }
}

// ==================== Random Policy ====================

/**
 * Random Policy: Random movement direction
 */
export class RandomPolicy implements DecisionMaker<IsacInput, number[]> {
    readonly name = 'random';

    decide(_input: IsacInput, rng: SeededRandom): number[] {
        return [
            rng.uniform(-1, 1),
            rng.uniform(-1, 1),
            rng.uniform(-0.5, 0.5),
        ];
    }

    reset(): void {
        // No state to reset
    }
}

// ==================== Rate-Only Policy ====================

/**
 * Rate-Only Policy: Move toward user with lowest SINR
 */
export class RateOnlyPolicy implements DecisionMaker<IsacInput, number[]> {
    readonly name = 'rate-only';
    private config: IsacTrajectoryConfig;

    constructor(config: IsacTrajectoryConfig) {
        this.config = config;
    }

    decide(input: IsacInput, _rng: SeededRandom): number[] {
        // Find user with lowest SINR
        const lowestSinrIdx = input.normalizedSinr.indexOf(
            Math.min(...input.normalizedSinr)
        );

        if (lowestSinrIdx < 0) return [0, 0, 0];

        // Move toward that user
        const _userDist = input.normalizedDistances[lowestSinrIdx];

        // Direction is encoded in distances (simplified)
        // Move toward the user by going in direction of largest distance gradient
        const targetUser = this.config.users[lowestSinrIdx];

        // Decode current position from normalized
        const [minX, minY, maxX, maxY] = this.config.areaBounds;
        const currentX = (input.normalizedPosition[0] + 1) / 2 * (maxX - minX) + minX;
        const currentY = (input.normalizedPosition[1] + 1) / 2 * (maxY - minY) + minY;

        // Direction to user
        const dx = targetUser.position.x - currentX;
        const dy = targetUser.position.y - currentY;
        const dist = Math.sqrt(dx * dx + dy * dy);

        if (dist < 1) return [0, 0, 0];

        return [
            dx / dist,
            dy / dist,
            0, // Maintain altitude
        ];
    }

    reset(): void {
        // No state to reset
    }
}

// ==================== Energy-Efficient Policy ====================

/**
 * Energy-Efficient Policy: Minimize energy while maintaining coverage
 */
export class EnergyEfficientPolicy implements DecisionMaker<IsacInput, number[]> {
    readonly name = 'energy-efficient';
    private config: IsacTrajectoryConfig;
    private hovering: boolean = true;
    private moveTimer: number = 0;

    constructor(config: IsacTrajectoryConfig) {
        this.config = config;
    }

    decide(input: IsacInput, rng: SeededRandom): number[] {
        // Hover most of the time, only move when LoS drops below threshold
        const losThreshold = 0.6;

        if (input.losPercentage < losThreshold) {
            // Need to move to improve LoS
            this.hovering = false;
            this.moveTimer = 5; // Move for 5 steps

            // Find direction that improves LoS (toward center of users with NLoS)
            let targetX = 0, targetY = 0, count = 0;
            for (let i = 0; i < input.losStatus.length; i++) {
                if (input.losStatus[i] === 0) {
                    targetX += this.config.users[i].position.x;
                    targetY += this.config.users[i].position.y;
                    count++;
                }
            }

            if (count > 0) {
                targetX /= count;
                targetY /= count;

                const [minX, minY, maxX, maxY] = this.config.areaBounds;
                const currentX = (input.normalizedPosition[0] + 1) / 2 * (maxX - minX) + minX;
                const currentY = (input.normalizedPosition[1] + 1) / 2 * (maxY - minY) + minY;

                const dx = targetX - currentX;
                const dy = targetY - currentY;
                const dist = Math.sqrt(dx * dx + dy * dy);

                if (dist > 1) {
                    // Also consider moving up for better LoS
                    return [
                        0.5 * dx / dist,
                        0.5 * dy / dist,
                        0.3, // Move up slowly
                    ];
                }
            }
        }

        if (this.moveTimer > 0) {
            this.moveTimer--;
            // Continue small movement
            return [
                rng.uniform(-0.2, 0.2),
                rng.uniform(-0.2, 0.2),
                0.1,
            ];
        }

        // Hover
        this.hovering = true;
        return [0, 0, 0];
    }

    reset(): void {
        this.hovering = true;
        this.moveTimer = 0;
    }
}

// ==================== Proposed Policy ====================

/**
 * Proposed Policy: Balance LoS, throughput, energy, and URLLC
 *
 * This is a heuristic baseline that combines multiple objectives.
 */
export class ProposedPolicy implements DecisionMaker<IsacInput, number[]> {
    readonly name = 'proposed';
    private config: IsacTrajectoryConfig;
    private stepsSinceMove: number = 0;
    private lastAction: number[] = [0, 0, 0];

    constructor(config: IsacTrajectoryConfig) {
        this.config = config;
    }

    decide(input: IsacInput, rng: SeededRandom): number[] {
        this.stepsSinceMove++;

        // Check URLLC violations
        const urllcViolations = input.urllcSatisfied.filter((s, i) =>
            this.config.users[i].isUrllc && s === 0
        ).length;

        // Priority 1: Fix URLLC violations
        if (urllcViolations > 0) {
            return this.moveTowardUrllcUsers(input);
        }

        // Priority 2: Maintain LoS
        if (input.losPercentage < 0.7) {
            return this.improveLoS(input);
        }

        // Priority 3: Maximize throughput if energy allows
        if (input.normalizedEnergy < 0.8) {
            const minSinrIdx = input.normalizedSinr.indexOf(
                Math.min(...input.normalizedSinr)
            );
            if (input.normalizedSinr[minSinrIdx] < 0.5) {
                return this.moveTowardWeakUser(input, minSinrIdx);
            }
        }

        // Priority 4: Save energy (hover with small adjustments)
        if (this.stepsSinceMove > 5) {
            this.stepsSinceMove = 0;
            // Small random exploration
            return [
                rng.uniform(-0.1, 0.1),
                rng.uniform(-0.1, 0.1),
                rng.uniform(-0.05, 0.05),
            ];
        }

        return [0, 0, 0];
    }

    private moveTowardUrllcUsers(input: IsacInput): number[] {
        let targetX = 0, targetY = 0, count = 0;

        for (let i = 0; i < this.config.users.length; i++) {
            if (this.config.users[i].isUrllc && input.urllcSatisfied[i] === 0) {
                targetX += this.config.users[i].position.x;
                targetY += this.config.users[i].position.y;
                count++;
            }
        }

        if (count === 0) return [0, 0, 0];

        targetX /= count;
        targetY /= count;

        return this.moveTowardPoint(input, targetX, targetY);
    }

    private improveLoS(_input: IsacInput): number[] {
        // Move up to improve LoS probability
        return [0, 0, 0.5];
    }

    private moveTowardWeakUser(input: IsacInput, userIdx: number): number[] {
        const user = this.config.users[userIdx];
        return this.moveTowardPoint(input, user.position.x, user.position.y);
    }

    private moveTowardPoint(input: IsacInput, targetX: number, targetY: number): number[] {
        const [minX, minY, maxX, maxY] = this.config.areaBounds;
        const currentX = (input.normalizedPosition[0] + 1) / 2 * (maxX - minX) + minX;
        const currentY = (input.normalizedPosition[1] + 1) / 2 * (maxY - minY) + minY;

        const dx = targetX - currentX;
        const dy = targetY - currentY;
        const dist = Math.sqrt(dx * dx + dy * dy);

        if (dist < 10) return [0, 0, 0];

        return [
            0.7 * dx / dist,
            0.7 * dy / dist,
            0,
        ];
    }

    reset(): void {
        this.stepsSinceMove = 0;
        this.lastAction = [0, 0, 0];
    }
}

// ==================== Factory Functions ====================

/**
 * Create a policy by name
 */
export function createPolicy(
    name: string,
    config: IsacTrajectoryConfig
): DecisionMaker<IsacInput, number[]> {
    switch (name) {
        case 'hover':
            return new HoverPolicy();
        case 'random':
            return new RandomPolicy();
        case 'rate-only':
            return new RateOnlyPolicy(config);
        case 'energy-efficient':
            return new EnergyEfficientPolicy(config);
        case 'proposed':
            return new ProposedPolicy(config);
        default:
            throw new Error(`Unknown policy: ${name}`);
    }
}

/**
 * Get list of available policies
 */
export function getAvailablePolicies(): string[] {
    return ['hover', 'random', 'rate-only', 'energy-efficient', 'proposed'];
}
