/**
 * @module communication/protocols
 * @description Drone and Multi-Robot Communication Simulation Algorithms
 * 
 * Provides features for:
 * 1. Path Loss Models (Friis, Two-Ray, Log-Distance, Air-to-Ground)
 * 2. RSSI calculation and Link Quality Evaluation
 * 3. UAV-Base Station Communication Protocols (HELLO/UPDATE/ACK)
 * 4. Multi-UAV Network Topology Management
 * 5. Channel Capacity and Throughput Estimation
 * 
 * Inspired by IoD_Sim and 3GPP standards for Aerial Communication.
 */

import { Vector2, Pose } from './types';

// ==================== Entity Type Definitions ====================

/**
 * Base Station / Access Point parameters
 */
export interface BaseStation {
    id: string;
    position: Vector2;
    /** Antenna height (m) */
    height: number;
    /** Transmit power (dBm) */
    txPower: number;
    /** Carrier frequency (GHz) */
    frequency: number;
    /** Channel bandwidth (MHz) */
    bandwidth: number;
    /** Antenna gain (dBi) */
    antennaGain: number;
    /** Nominal coverage radius (m) */
    coverageRadius: number;
    color: string;
    /** Station platform type */
    type: 'ground' | 'aerial' | 'hap'; // Hap: High Altitude Platform
}

/**
 * Drone Communication Node
 */
export interface DroneNode {
    id: string;
    pose: Pose;
    height: number;
    txPower: number;
    antennaGain: number;
    state: ClientState;
    connectedStation: string | null;
    lastUpdateTime: number;
    /** Battery level [0-1] */
    batteryLevel: number;
    /** Current real-time data rate (Mbps) */
    dataRate: number;
}

/**
 * Communication Link Information
 */
export interface CommLink {
    robotId: string;
    stationId: string;
    /** Received Signal Strength Indicator (dBm) */
    rssi: number;
    /** Signal-to-Noise Ratio (dB) */
    snr: number;
    /** 3D Distance (m) */
    distance: number;
    /** Path loss (dB) */
    pathLoss: number;
    quality: LinkQuality;
    /** Estimated data rate based on Shannon capacity (Mbps) */
    dataRate: number;
    /** Estimated end-to-end latency (ms) */
    latency: number;
}

/**
 * Link Quality Classification
 */
export type LinkQuality = 'excellent' | 'good' | 'fair' | 'poor' | 'disconnected';

/**
 * Client Connection State
 */
export type ClientState = 'CLOSED' | 'HELLO_SENT' | 'CONNECTED';

/**
 * Communication Packet Types
 */
export type PacketType = 'HELLO' | 'HELLO_ACK' | 'UPDATE' | 'UPDATE_ACK' | 'DATA' | 'UNKNOWN';

/**
 * Propagation Loss Model Types
 */
export type PropagationModel = 'friis' | 'two-ray' | 'log-distance' | 'three-gpp-uma' | 'air-to-ground';

// ==================== Physical Constants ====================

const SPEED_OF_LIGHT = 299792458;  // m/s
const BOLTZMANN_CONSTANT = 1.38e-23; // J/K
const NOISE_FIGURE = 6;  // dB (Typical receiver noise figure)
const TEMPERATURE = 290; // K (Room temperature)

// ==================== Path Loss Models ====================

/**
 * Friis Free-Space Path Loss (FSPL)
 * FSPL(dB) = 20*log10(d) + 20*log10(f) - 147.55
 * 
 * @param distance - Distance (m)
 * @param frequencyGHz - Carrier frequency (GHz)
 * @returns Path loss in dB
 */
export function friisPathLoss(distance: number, frequencyGHz: number): number {
    if (distance <= 0) return 0;
    const frequencyHz = frequencyGHz * 1e9;
    // FSPL = (4 * PI * d * f / c)^2
    // In dB: 20*log10(4*PI*d*f/c) = 20*log10(d) + 20*log10(f) + 20*log10(4*PI/c)
    // 20*log10(4*PI/c) approx -147.55 for f in Hz
    const fspl = 20 * Math.log10(distance) + 20 * Math.log10(frequencyHz) - 147.55;
    return Math.max(0, fspl);
}

/**
 * Two-Ray Ground Reflection Model
 * Suitable for low-altitude ground-to-ground communication.
 * 
 * @param distance - Horizontal distance (m)
 * @param txHeight - Transmitter antenna height (m)
 * @param rxHeight - Receiver antenna height (m)
 * @param frequencyGHz - Carrier frequency (GHz)
 * @returns Path loss in dB
 */
export function twoRayPathLoss(
    distance: number,
    txHeight: number,
    rxHeight: number,
    frequencyGHz: number
): number {
    if (distance <= 0) return 0;

    // Critical breakpoint distance
    const wavelength = SPEED_OF_LIGHT / (frequencyGHz * 1e9);
    const breakpointDistance = (4 * txHeight * rxHeight) / wavelength;

    if (distance < breakpointDistance) {
        // Near field: use Friis model
        return friisPathLoss(distance, frequencyGHz);
    } else {
        // Far field: Two-Ray model
        // PL = 40*log10(d) - 20*log10(ht) - 20*log10(hr)
        return 40 * Math.log10(distance) - 20 * Math.log10(txHeight) - 20 * Math.log10(rxHeight);
    }
}

/**
 * Log-Distance Path Loss Model
 * PL(d) = PL(d0) + 10*n*log10(d/d0) + X_sigma
 * 
 * @param distance - Distance (m)
 * @param frequencyGHz - Frequency (GHz)
 * @param pathLossExponent - Decay exponent (typically 2-4)
 * @param referenceDistance - Reference distance (m, typical 1.0)
 * @param shadowingStdDev - Log-normal shadowing standard deviation (dB)
 * @returns Path loss in dB
 */
export function logDistancePathLoss(
    distance: number,
    frequencyGHz: number,
    pathLossExponent: number = 2.5,
    referenceDistance: number = 1,
    shadowingStdDev: number = 0
): number {
    if (distance <= referenceDistance) {
        return friisPathLoss(referenceDistance, frequencyGHz);
    }

    const referenceLoss = friisPathLoss(referenceDistance, frequencyGHz);
    const distanceFactor = 10 * pathLossExponent * Math.log10(distance / referenceDistance);

    // Add Shadowing (Gaussian random variable in log-scale)
    let shadowing = 0;
    if (shadowingStdDev > 0) {
        const u1 = Math.random();
        const u2 = Math.random();
        shadowing = shadowingStdDev * Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
    }

    return referenceLoss + distanceFactor + shadowing;
}

/**
 * Air-to-Ground Channel Model
 * Based on 3GPP TR 36.777.
 * Considers Probability of Line-of-Sight (LoS) based on elevation angle.
 * 
 * @param distance - Horizontal distance (m)
 * @param droneHeight - Altitude of the drone (m)
 * @param stationHeight - Height of the ground station (m)
 * @param frequencyGHz - Frequency (GHz)
 * @param environment - Type of surrounding environment
 * @returns Composite path loss in dB
 */
export function airToGroundPathLoss(
    distance: number,
    droneHeight: number,
    stationHeight: number,
    frequencyGHz: number,
    environment: 'urban' | 'suburban' | 'rural' = 'suburban'
): number {
    // 3D Distance calculation
    const heightDiff = Math.abs(droneHeight - stationHeight);
    const distance3D = Math.sqrt(distance * distance + heightDiff * heightDiff);

    // Elevation angle in degrees
    const elevationAngle = Math.atan2(heightDiff, distance) * 180 / Math.PI;

    // LoS Probability coefficients based on 3GPP environment models
    let a: number, b: number;
    switch (environment) {
        case 'urban':
            a = 9.61;
            b = 0.16;
            break;
        case 'suburban':
            a = 4.88;
            b = 0.43;
            break;
        case 'rural':
        default:
            a = 2.0;
            b = 0.6;
            break;
    }
    const losProb = 1 / (1 + a * Math.exp(-b * (elevationAngle - a)));

    // LOS Path Loss
    const losPathLoss = friisPathLoss(distance3D, frequencyGHz);

    // Additional Non-LoS loss based on environment
    let nlosExtraLoss: number;
    let envBaseLoss: number; // Global attenuation factor for urban clutter

    switch (environment) {
        case 'urban':
            nlosExtraLoss = 28;
            envBaseLoss = 15;
            break;
        case 'suburban':
            nlosExtraLoss = 18;
            envBaseLoss = 5;
            break;
        case 'rural':
        default:
            nlosExtraLoss = 8;
            envBaseLoss = 0;
            break;
    }
    const nlosPathLoss = losPathLoss + nlosExtraLoss;

    // Combined Path Loss (probabilistic weighting)
    return (losProb * losPathLoss + (1 - losProb) * nlosPathLoss) + envBaseLoss;
}

// ==================== RSSI and Link Quality ====================

/**
 * Calculates Received Signal Strength Indicator (RSSI)
 * 
 * @param txPower - Transmit power (dBm)
 * @param txGain - Transmitter antenna gain (dBi)
 * @param rxGain - Receiver antenna gain (dBi)
 * @param pathLoss - Total path loss (dB)
 * @returns RSSI in dBm
 */
export function calculateRSSI(
    txPower: number,
    txGain: number,
    rxGain: number,
    pathLoss: number
): number {
    return txPower + txGain + rxGain - pathLoss;
}

/**
 * Calculates Signal-to-Noise Ratio (SNR)
 * 
 * @param rssi - Received signal strength (dBm)
 * @param bandwidthMHz - Channel bandwidth (MHz)
 * @param noiseFigure - Receiver noise figure (dB)
 * @returns SNR in dB
 */
export function calculateSNR(
    rssi: number,
    bandwidthMHz: number = 20,
    noiseFigure: number = NOISE_FIGURE
): number {
    // Thermal noise power = -174 + 10*log10(Bandwidth_Hz) + Noise_Figure
    const noisePower = -174 + 10 * Math.log10(bandwidthMHz * 1e6) + noiseFigure;
    return rssi - noisePower;
}

/**
 * Evaluates Link Quality based on RSSI and SNR thresholds
 */
export function evaluateLinkQuality(
    rssi: number,
    snr: number,
    thresholds: {
        excellent: number;
        good: number;
        fair: number;
        poor: number;
    } = { excellent: -50, good: -65, fair: -75, poor: -85 }
): LinkQuality {
    if (rssi >= thresholds.excellent && snr > 25) return 'excellent';
    if (rssi >= thresholds.good && snr > 15) return 'good';
    if (rssi >= thresholds.fair && snr > 8) return 'fair';
    if (rssi >= thresholds.poor && snr > 3) return 'poor';
    return 'disconnected';
}

/**
 * Estimates peak data rate using Shannon-Hartley Theorem
 * C = B * log2(1 + SNR)
 * 
 * @param snr - Signal to Noise Ratio in dB
 * @param bandwidthMHz - Bandwidth in MHz
 * @returns Estimated data rate in Mbps
 */
export function estimateDataRate(snr: number, bandwidthMHz: number = 20): number {
    const snrLinear = Math.pow(10, snr / 10);
    const capacity = bandwidthMHz * Math.log2(1 + snrLinear);
    // Real-world performance is typically ~70% of theoretical limit
    return Math.max(0, capacity * 0.7);
}

/**
 * Estimates end-to-end latency
 * 
 * @param distance - 3D distance (m)
 * @param dataRate - Current data rate (Mbps)
 * @param packetSize - Packet size (bytes)
 * @returns Latency in ms
 */
export function estimateLatency(
    distance: number,
    dataRate: number,
    packetSize: number = 1500
): number {
    // Propagation delay (speed of light)
    const propagationDelay = distance / SPEED_OF_LIGHT * 1000;

    // Transmission delay (packet size / data rate)
    const transmissionDelay = dataRate > 0 ? (packetSize * 8) / (dataRate * 1e6) * 1000 : 1000;

    // Fixed processing delay overhead
    const processingDelay = 0.5;

    return propagationDelay + transmissionDelay + processingDelay;
}

// ==================== Full Link Computation ====================

/**
 * Computes a comprehensive communication link analysis between a drone and a base station
 */
export function calculateCommLink(
    drone: DroneNode | { id: string; pose: Pose; height?: number; txPower?: number; antennaGain?: number },
    station: BaseStation,
    model: PropagationModel = 'air-to-ground',
    environment: 'urban' | 'suburban' | 'rural' = 'suburban'
): CommLink {
    const dx = drone.pose.x - station.position.x;
    const dy = drone.pose.y - station.position.y;
    const horizontalDistance = Math.sqrt(dx * dx + dy * dy);

    const droneHeight = (drone as DroneNode).height ?? 50;
    const stationHeight = station.height ?? 10;

    const heightDiff = Math.abs(droneHeight - stationHeight);
    const distance3D = Math.sqrt(horizontalDistance * horizontalDistance + heightDiff * heightDiff);

    // Path Loss calculation
    let pathLoss: number;
    switch (model) {
        case 'friis':
            pathLoss = friisPathLoss(distance3D, station.frequency);
            break;
        case 'two-ray':
            pathLoss = twoRayPathLoss(horizontalDistance, droneHeight, stationHeight, station.frequency);
            break;
        case 'log-distance':
            pathLoss = logDistancePathLoss(distance3D, station.frequency, 2.5);
            break;
        case 'air-to-ground':
        default:
            pathLoss = airToGroundPathLoss(horizontalDistance, droneHeight, stationHeight, station.frequency, environment);
            break;
    }

    // Link budget calculation
    const droneTxPower = (drone as DroneNode).txPower ?? 20;
    const droneGain = (drone as DroneNode).antennaGain ?? 0;
    const rssi = calculateRSSI(station.txPower, station.antennaGain, droneGain, pathLoss);
    const snr = calculateSNR(rssi, station.bandwidth);
    const quality = evaluateLinkQuality(rssi, snr);
    const dataRate = estimateDataRate(snr, station.bandwidth);
    const latency = estimateLatency(distance3D, dataRate);

    return {
        robotId: drone.id,
        stationId: station.id,
        rssi,
        snr,
        distance: distance3D,
        pathLoss,
        quality,
        dataRate,
        latency
    };
}

// ==================== Network Topology Management ====================

/**
 * Station Selection (Handoff) Algorithm
 * Selects the best station based on RSSI with hysteresis to prevent ping-pong effect.
 */
export function selectBestStation(
    drone: DroneNode,
    stations: BaseStation[],
    currentStationId: string | null,
    hysteresis: number = 3
): string | null {
    if (stations.length === 0) return null;

    let bestStation: BaseStation | null = null;
    let bestRssi = -Infinity;

    for (const station of stations) {
        const link = calculateCommLink(drone, station);

        let effectiveRssi = link.rssi;
        if (station.id === currentStationId) {
            effectiveRssi += hysteresis; // Avoid switching unless a considerably better signal is found
        }

        if (effectiveRssi > bestRssi && link.quality !== 'disconnected') {
            bestRssi = effectiveRssi;
            bestStation = station;
        }
    }

    return bestStation?.id ?? null;
}

/**
 * Calculates aggregate network coverage area ratio
 */
export function calculateCoverage(
    worldWidth: number,
    worldHeight: number,
    stations: BaseStation[],
    gridResolution: number = 5,
    rssiThreshold: number = -80
): number {
    let coveredCount = 0;
    let totalCount = 0;

    for (let x = 0; x < worldWidth; x += gridResolution) {
        for (let y = 0; y < worldHeight; y += gridResolution) {
            totalCount++;

            for (const station of stations) {
                const testPos = { x, y, theta: 0 };
                const link = calculateCommLink({ id: 'test', pose: testPos }, station);

                if (link.rssi >= rssiThreshold) {
                    coveredCount++;
                    break;
                }
            }
        }
    }

    return totalCount > 0 ? coveredCount / totalCount : 0;
}

// ==================== Packet Simulation ====================

/**
 * Network Packet Object
 */
export interface Packet {
    id: number;
    type: PacketType;
    sourceId: string;
    destId: string;
    timestamp: number;
    payload: unknown;
    /** Packet size in bytes */
    size: number;
    /** QoS Priority [0-7] */
    priority: number;
}

/**
 * Factory for HELLO packets (Connection initialization)
 */
export function createHelloPacket(sourceId: string, destId: string): Packet {
    return {
        id: Date.now() + Math.random(),
        type: 'HELLO',
        sourceId,
        destId,
        timestamp: Date.now(),
        payload: { position: null, velocity: null },
        size: 64,
        priority: 7
    };
}

/**
 * Factory for UPDATE packets (State synchronization)
 */
export function createUpdatePacket(
    sourceId: string,
    destId: string,
    position: Pose,
    velocity: { vx: number; vy: number; vz?: number }
): Packet {
    return {
        id: Date.now() + Math.random(),
        type: 'UPDATE',
        sourceId,
        destId,
        timestamp: Date.now(),
        payload: { position, velocity },
        size: 128,
        priority: 5
    };
}

/**
 * Simulates Packet Success Probability based on Bit Error Rate (BER)
 * Simplified model assuming BPSK modulation.
 */
export function packetSuccessProbability(snr: number, packetSize: number = 1500): number {
    const snrLinear = Math.pow(10, snr / 10);
    // Approximate BPSK BER = 0.5 * exp(-SNR/2)
    const ber = 0.5 * Math.exp(-snrLinear / 2);

    // Packet Success Rate (PSR) = (1 - BER)^(Bits_per_packet)
    const bitCount = 8 * packetSize;
    return Math.pow(1 - ber, bitCount);
}

