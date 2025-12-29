/**
 * @module src/isac/metrics/throughput
 * @description Throughput Metrics
 */

import type { Position3D } from '../planner';

/**
 * Calculate free-space path loss (dB)
 */
export function calculatePathLoss(
    distance: number,
    frequencyHz: number
): number {
    const c = 3e8; // Speed of light
    const wavelength = c / frequencyHz;
    return 20 * Math.log10(4 * Math.PI * distance / wavelength);
}

/**
 * Calculate SINR (dB)
 */
export function calculateSinr(
    txPowerDbm: number,
    pathLossDb: number,
    noisePowerDbm: number,
    fadingDb: number = 0
): number {
    const rxPowerDbm = txPowerDbm - pathLossDb + fadingDb;
    return rxPowerDbm - noisePowerDbm;
}

/**
 * Calculate Shannon capacity (bps)
 */
export function calculateCapacity(
    bandwidthHz: number,
    sinrDb: number
): number {
    const sinrLinear = Math.pow(10, sinrDb / 10);
    return bandwidthHz * Math.log2(1 + sinrLinear);
}

/**
 * Calculate distance between two 3D points
 */
export function calculateDistance(
    pos1: Position3D,
    pos2: Position3D
): number {
    return Math.sqrt(
        (pos1.x - pos2.x) ** 2 +
        (pos1.y - pos2.y) ** 2 +
        (pos1.z - pos2.z) ** 2
    );
}

/**
 * Calculate aggregate throughput for multiple users
 */
export function calculateAggregateThroughput(
    uavPosition: Position3D,
    userPositions: Position3D[],
    config: {
        txPowerDbm: number;
        bandwidthHz: number;
        frequencyHz: number;
        noiseFigureDb: number;
    }
): number {
    const thermalNoiseDbm = -174 + 10 * Math.log10(config.bandwidthHz);
    const noisePowerDbm = thermalNoiseDbm + config.noiseFigureDb;

    let totalThroughput = 0;

    for (const userPos of userPositions) {
        const distance = calculateDistance(uavPosition, userPos);
        const pathLoss = calculatePathLoss(Math.max(1, distance), config.frequencyHz);
        const sinr = calculateSinr(config.txPowerDbm, pathLoss, noisePowerDbm);
        const capacity = calculateCapacity(config.bandwidthHz, sinr);
        totalThroughput += capacity;
    }

    return totalThroughput;
}
