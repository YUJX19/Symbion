/**
 * @module tasks/u2u-mcs/scenario
 * @description SINR, fading, and mobility generation for U2U-MCS task
 */

import { SeededRandom, createRng } from '../../core/repro';
import type { U2UMcsConfig, ChannelModelType } from './config';

// ==================== Types ====================

/**
 * Scenario state at each step
 */
export interface ScenarioState {
    /** Current step number */
    step: number;
    /** Current distance in meters */
    distance: number;
    /** Current relative speed in m/s */
    relativeSpeed: number;
    /** SINR in dB */
    sinrDb: number;
    /** Path loss in dB */
    pathLossDb: number;
    /** Small-scale fading in dB */
    fadingDb: number;
    /** Shadowing in dB */
    shadowingDb: number;
}

// ==================== Scenario Class ====================

/**
 * U2U Scenario: generates channel and mobility states
 */
export class U2UScenario {
    private config: U2UMcsConfig;
    private rng: SeededRandom;
    private currentStep: number = 0;
    private currentDistance: number;
    private currentSpeed: number;
    private shadowingState: number = 0;

    constructor(config: U2UMcsConfig) {
        this.config = config;
        this.rng = createRng(config.seed);

        // Initialize position
        const { distanceRange, relativeSpeedRange } = config.mobilityConfig;
        this.currentDistance = this.rng.uniform(distanceRange[0], distanceRange[1]);
        this.currentSpeed = this.rng.uniform(relativeSpeedRange[0], relativeSpeedRange[1]);
    }

    /**
     * Reset scenario to initial state
     */
    reset(seed?: number): void {
        this.rng = createRng(seed ?? this.config.seed);
        this.currentStep = 0;
        this.shadowingState = 0;

        const { distanceRange, relativeSpeedRange } = this.config.mobilityConfig;
        this.currentDistance = this.rng.uniform(distanceRange[0], distanceRange[1]);
        this.currentSpeed = this.rng.uniform(relativeSpeedRange[0], relativeSpeedRange[1]);
    }

    /**
     * Generate state for current step
     */
    generateState(): ScenarioState {
        // Update mobility
        this.updateMobility();

        // Calculate path loss
        const pathLossDb = this.calculatePathLoss(this.currentDistance);

        // Generate fading
        const fadingDb = this.generateFading();

        // Update shadowing (correlated)
        this.updateShadowing();

        // Calculate SINR
        const sinrDb = this.calculateSinr(pathLossDb, fadingDb, this.shadowingState);

        this.currentStep++;

        return {
            step: this.currentStep - 1,
            distance: this.currentDistance,
            relativeSpeed: this.currentSpeed,
            sinrDb,
            pathLossDb,
            fadingDb,
            shadowingDb: this.shadowingState,
        };
    }

    /**
     * Calculate path loss using log-distance model
     */
    private calculatePathLoss(distance: number): number {
        const d0 = 1; // Reference distance
        const pl0 = 40; // Path loss at reference distance (dB)
        const alpha = 2.5; // Path loss exponent for LoS

        return pl0 + 10 * alpha * Math.log10(Math.max(distance, d0) / d0);
    }

    /**
     * Generate small-scale fading based on channel model
     */
    private generateFading(): number {
        switch (this.config.channelModel) {
            case 'awgn':
                return 0; // No fading

            case 'rayleigh': {
                // Rayleigh fading (complex Gaussian magnitude)
                const real = this.rng.normal(0, 1);
                const imag = this.rng.normal(0, 1);
                const magnitude = Math.sqrt(real * real + imag * imag);
                return 20 * Math.log10(magnitude / Math.sqrt(2));
            }

            case 'rician': {
                // Rician fading
                const K = Math.pow(10, (this.config.ricianKDb ?? 6) / 10);
                const sigma = 1 / Math.sqrt(2 * (K + 1));
                const mu = Math.sqrt(K / (K + 1));

                const real = this.rng.normal(mu, sigma);
                const imag = this.rng.normal(0, sigma);
                const magnitude = Math.sqrt(real * real + imag * imag);
                return 20 * Math.log10(magnitude);
            }

            default:
                return 0;
        }
    }

    /**
     * Update correlated shadowing using Gudmundson model
     */
    private updateShadowing(): void {
        const sigmaShadow = 4; // Shadow standard deviation (dB)
        const correlationDistance = 50; // meters
        const stepDistance = this.currentSpeed * (this.config.stepDurationMs / 1000);

        // Correlation coefficient
        const rho = Math.exp(-stepDistance / correlationDistance);

        // Update shadowing
        const innovation = this.rng.normal(0, sigmaShadow * Math.sqrt(1 - rho * rho));
        this.shadowingState = rho * this.shadowingState + innovation;
    }

    /**
     * Calculate SINR from channel gains
     */
    private calculateSinr(pathLossDb: number, fadingDb: number, shadowingDb: number): number {
        const { txPowerDbm, noiseFigureDb, bandwidthHz, numRBs } = this.config;

        // Received power
        const rxPowerDbm = txPowerDbm - pathLossDb + fadingDb + shadowingDb;

        // Noise power
        const thermalNoiseDbm = -174 + 10 * Math.log10(bandwidthHz);
        const noisePowerDbm = thermalNoiseDbm + noiseFigureDb;

        // SINR (assuming no interference for now)
        return rxPowerDbm - noisePowerDbm;
    }

    /**
     * Update mobility state
     */
    private updateMobility(): void {
        const { distanceRange, relativeSpeedRange, dynamicSpeed, speedChangeRate } = this.config.mobilityConfig;
        const dt = this.config.stepDurationMs / 1000; // Convert to seconds

        // Update speed if dynamic
        if (dynamicSpeed && speedChangeRate) {
            const speedDelta = this.rng.uniform(-speedChangeRate, speedChangeRate);
            this.currentSpeed = Math.max(
                relativeSpeedRange[0],
                Math.min(relativeSpeedRange[1], this.currentSpeed + speedDelta)
            );
        }

        // Update distance (random walk with bounds)
        const direction = this.rng.random() > 0.5 ? 1 : -1;
        const distanceChange = direction * this.currentSpeed * dt;
        this.currentDistance = Math.max(
            distanceRange[0],
            Math.min(distanceRange[1], this.currentDistance + distanceChange)
        );
    }

    /**
     * Get current distance
     */
    getDistance(): number {
        return this.currentDistance;
    }

    /**
     * Get current speed
     */
    getSpeed(): number {
        return this.currentSpeed;
    }
}

/**
 * Create a scenario from config
 */
export function createScenario(config: U2UMcsConfig): U2UScenario {
    return new U2UScenario(config);
}
