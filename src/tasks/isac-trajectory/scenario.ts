/**
 * @module tasks/isac-trajectory/scenario
 * @description ISAC scenario: channel, LoS, and mobility state generation
 */

import { SeededRandom, createRng } from '../../core/repro';
import {
    getLosProbability,
    type IsacTrajectoryConfig,
    type Position3D,
    type GroundUserConfig,
    type ObstacleConfig,
} from './config';

// ==================== Types ====================

/**
 * Channel state for a single user
 */
export interface UserChannelState {
    userId: string;
    distance: number;
    elevationAngleDeg: number;
    isLos: boolean;
    pathLossDb: number;
    channelGainDb: number;
    sinrDb: number;
    achievableRateBps: number;
}

/**
 * UAV state at a given time
 */
export interface UavState {
    position: Position3D;
    velocity: Position3D;
    heading: number; // radians
    speed: number;
    energyUsed: number;
}

/**
 * Full scenario state for a step
 */
export interface ScenarioState {
    step: number;
    uav: UavState;
    userChannels: UserChannelState[];
    losPercentage: number; // Fraction of users with LoS
    totalThroughput: number;
    urllcSatisfied: boolean[];
    sensingCoverage: number; // Fraction of area covered
}

// ==================== Scenario Class ====================

/**
 * ISAC Scenario: generates channel, LoS, and state for each step
 */
export class IsacScenario {
    private config: IsacTrajectoryConfig;
    private rng: SeededRandom;
    private currentStep: number = 0;
    private uavState: UavState;
    private totalEnergy: number = 0;

    constructor(config: IsacTrajectoryConfig) {
        this.config = config;
        this.rng = createRng(config.seed);
        this.uavState = this.createInitialUavState();
    }

    private createInitialUavState(): UavState {
        return {
            position: { ...this.config.startPosition },
            velocity: { x: 0, y: 0, z: 0 },
            heading: 0,
            speed: 0,
            energyUsed: 0,
        };
    }

    /**
     * Reset scenario
     */
    reset(seed?: number): void {
        this.rng = createRng(seed ?? this.config.seed);
        this.currentStep = 0;
        this.uavState = this.createInitialUavState();
        this.totalEnergy = 0;
    }

    /**
     * Move UAV to a new position
     */
    moveUav(targetPosition: Position3D): void {
        const dt = this.config.waypointDurationS;
        const { maxSpeed, maxAltitude, minAltitude } = this.config.kinematics;

        // Clamp target altitude
        const clampedTarget: Position3D = {
            x: targetPosition.x,
            y: targetPosition.y,
            z: Math.max(minAltitude, Math.min(maxAltitude, targetPosition.z)),
        };

        // Calculate movement
        const dx = clampedTarget.x - this.uavState.position.x;
        const dy = clampedTarget.y - this.uavState.position.y;
        const dz = clampedTarget.z - this.uavState.position.z;
        const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);

        // Clamp to max speed
        const actualDistance = Math.min(distance, maxSpeed * dt);
        const ratio = distance > 0 ? actualDistance / distance : 0;

        // Update position
        this.uavState.position = {
            x: this.uavState.position.x + dx * ratio,
            y: this.uavState.position.y + dy * ratio,
            z: this.uavState.position.z + dz * ratio,
        };

        // Update velocity and speed
        this.uavState.speed = actualDistance / dt;
        this.uavState.velocity = {
            x: (dx * ratio) / dt,
            y: (dy * ratio) / dt,
            z: (dz * ratio) / dt,
        };

        // Update heading
        if (dx !== 0 || dy !== 0) {
            this.uavState.heading = Math.atan2(dy, dx);
        }

        // Calculate energy consumption
        const energy = this.calculateEnergy(actualDistance, dt);
        this.totalEnergy += energy;
        this.uavState.energyUsed = this.totalEnergy;
    }

    /**
     * Calculate energy consumption for a movement
     */
    private calculateEnergy(distance: number, duration: number): number {
        const { hoverPower, propulsionEfficiency, maxSpeed } = this.config.kinematics;
        const speed = distance / duration;

        // Simplified model: hover + propulsion
        const propulsionPower = (speed / maxSpeed) * hoverPower * 0.5;
        return (hoverPower + propulsionPower) * duration / propulsionEfficiency;
    }

    /**
     * Generate state for current step
     */
    generateState(): ScenarioState {
        const userChannels = this.config.users.map(user =>
            this.calculateUserChannel(user)
        );

        const losCount = userChannels.filter(c => c.isLos).length;
        const losPercentage = losCount / userChannels.length;

        const totalThroughput = userChannels.reduce((sum, c) => sum + c.achievableRateBps, 0);

        const urllcSatisfied = this.config.users.map((user, i) => {
            if (!user.isUrllc) return true;
            // URLLC satisfied if achievable rate >= min rate
            return userChannels[i].achievableRateBps >= user.minRate;
        });

        const sensingCoverage = this.calculateSensingCoverage();

        const state: ScenarioState = {
            step: this.currentStep,
            uav: { ...this.uavState },
            userChannels,
            losPercentage,
            totalThroughput,
            urllcSatisfied,
            sensingCoverage,
        };

        this.currentStep++;
        return state;
    }

    /**
     * Calculate channel state for a user
     */
    private calculateUserChannel(user: GroundUserConfig): UserChannelState {
        const { comm } = this.config;

        // Distance calculation
        const dx = user.position.x - this.uavState.position.x;
        const dy = user.position.y - this.uavState.position.y;
        const dz = user.position.z - this.uavState.position.z;
        const distance3d = Math.sqrt(dx * dx + dy * dy + dz * dz);
        const distance2d = Math.sqrt(dx * dx + dy * dy);

        // Elevation angle
        const elevationAngleDeg = distance2d > 0
            ? Math.atan2(this.uavState.position.z - user.position.z, distance2d) * 180 / Math.PI
            : 90;

        // LoS probability and determination
        const losProb = this.calculateLosProbability(user, elevationAngleDeg);
        const isLos = this.rng.random() < losProb && !this.isBlockedByObstacle(user);

        // Path loss
        const plExponent = isLos ? comm.plExponentLos : comm.plExponentNlos;
        const d0 = 1; // Reference distance
        const wavelength = 3e8 / comm.carrierFrequencyHz;
        const pl0 = 20 * Math.log10(4 * Math.PI * d0 / wavelength);
        const pathLossDb = pl0 + 10 * plExponent * Math.log10(Math.max(distance3d, d0) / d0);

        // Small-scale fading
        let fadingDb = 0;
        if (isLos) {
            // Rician fading
            const K = Math.pow(10, comm.ricianKLosDb / 10);
            const sigma = 1 / Math.sqrt(2 * (K + 1));
            const mu = Math.sqrt(K / (K + 1));
            const real = this.rng.normal(mu, sigma);
            const imag = this.rng.normal(0, sigma);
            fadingDb = 20 * Math.log10(Math.sqrt(real * real + imag * imag));
        } else {
            // Rayleigh fading
            const real = this.rng.normal(0, 1);
            const imag = this.rng.normal(0, 1);
            fadingDb = 20 * Math.log10(Math.sqrt(real * real + imag * imag) / Math.sqrt(2));
        }

        // Channel gain
        const channelGainDb = -pathLossDb + fadingDb;

        // SINR
        const rxPowerDbm = comm.txPowerDbm + channelGainDb;
        const thermalNoiseDbm = -174 + 10 * Math.log10(comm.bandwidthHz);
        const noisePowerDbm = thermalNoiseDbm + comm.noiseFigureDb;
        const sinrDb = rxPowerDbm - noisePowerDbm;

        // Achievable rate (Shannon)
        const sinrLinear = Math.pow(10, sinrDb / 10);
        const achievableRateBps = comm.bandwidthHz * Math.log2(1 + sinrLinear);

        return {
            userId: user.id,
            distance: distance3d,
            elevationAngleDeg,
            isLos,
            pathLossDb,
            channelGainDb,
            sinrDb,
            achievableRateBps,
        };
    }

    /**
     * Calculate LoS probability considering environment
     */
    private calculateLosProbability(user: GroundUserConfig, elevationAngleDeg: number): number {
        return getLosProbability(elevationAngleDeg, this.config.comm.losModel);
    }

    /**
     * Check if LoS is blocked by any obstacle
     */
    private isBlockedByObstacle(user: GroundUserConfig): boolean {
        for (const obstacle of this.config.obstacles) {
            if (this.lineIntersectsBox(
                this.uavState.position,
                user.position,
                obstacle
            )) {
                return true;
            }
        }
        return false;
    }

    /**
     * Check if line segment intersects obstacle bounding box
     */
    private lineIntersectsBox(p1: Position3D, p2: Position3D, obstacle: ObstacleConfig): boolean {
        const [w, d, h] = obstacle.dimensions;
        const minX = obstacle.position.x - w / 2;
        const maxX = obstacle.position.x + w / 2;
        const minY = obstacle.position.y - d / 2;
        const maxY = obstacle.position.y + d / 2;
        const minZ = obstacle.position.z;
        const maxZ = obstacle.position.z + h;

        // Simplified ray-box intersection test
        const dx = p2.x - p1.x;
        const dy = p2.y - p1.y;
        const dz = p2.z - p1.z;

        let tmin = 0, tmax = 1;

        const checkAxis = (origin: number, dir: number, min: number, max: number) => {
            if (Math.abs(dir) < 1e-10) {
                return origin >= min && origin <= max;
            }
            let t1 = (min - origin) / dir;
            let t2 = (max - origin) / dir;
            if (t1 > t2) [t1, t2] = [t2, t1];
            tmin = Math.max(tmin, t1);
            tmax = Math.min(tmax, t2);
            return tmin <= tmax;
        };

        if (!checkAxis(p1.x, dx, minX, maxX)) return false;
        if (!checkAxis(p1.y, dy, minY, maxY)) return false;
        if (!checkAxis(p1.z, dz, minZ, maxZ)) return false;

        return true;
    }

    /**
     * Calculate sensing coverage (simplified)
     */
    private calculateSensingCoverage(): number {
        const { sensing, areaBounds } = this.config;
        const [minX, minY, maxX, maxY] = areaBounds;
        const areaSize = (maxX - minX) * (maxY - minY);

        // Circular coverage based on altitude and FoV
        const altitude = this.uavState.position.z;
        const fovRad = sensing.fieldOfViewDeg * Math.PI / 180;
        const coverageRadius = altitude * Math.tan(fovRad / 2);
        const coverageArea = Math.PI * coverageRadius * coverageRadius;

        return Math.min(1, coverageArea / areaSize);
    }

    /**
     * Get current UAV position
     */
    getCurrentPosition(): Position3D {
        return { ...this.uavState.position };
    }

    /**
     * Get current step
     */
    getCurrentStep(): number {
        return this.currentStep;
    }
}

/**
 * Create scenario from config
 */
export function createScenario(config: IsacTrajectoryConfig): IsacScenario {
    return new IsacScenario(config);
}
