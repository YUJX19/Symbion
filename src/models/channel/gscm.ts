/**
 * @module channel/gscm
 * @description Geometric Stochastic Channel Model (GSCM) with Scatter Clusters
 * 
 * Implements a geometric scatter-based channel model with:
 * - Log-Normal Large Scale Parameters (LSP)
 * - Cluster-based multipath propagation
 * - Ray-level fading and Doppler
 * 
 * ## References
 * - 3GPP TR 38.901 Channel Model
 * - WINNER/WINNER+ Channel Models
 */

export interface ScatterChannelParams {
    /** Carrier frequency in GHz */
    carrierFreq: number;
    /** Number of scatterer clusters */
    numClusters: number;
    /** Number of rays per cluster */
    numRaysPerCluster: number;
    /** Delay Spread Median (seconds) */
    dsMu: number;
    /** Log-std for Delay Spread */
    dsSigma: number;
    /** Azimuth Spread Departure Median (degrees) */
    asDMu: number;
    /** Log-std for ASD */
    asDSigma: number;
    /** Azimuth Spread Arrival Median (degrees) */
    asAMu: number;
    /** Log-std for ASA */
    asASigma: number;
    /** BS-UE distance (meters) */
    distance: number;
    /** UE velocity (m/s) */
    velocity: number;
}

export interface Ray {
    /** Angle of Departure (radians) */
    aoD: number;
    /** Angle of Arrival (radians) */
    aoA: number;
    /** Phase (radians) */
    phase: number;
    /** Power (linear, normalized) */
    power: number;
    /** Doppler frequency (Hz) */
    doppler: number;
}

export interface ScattererCluster {
    id: number;
    x: number;
    y: number;
    /** Delay in nanoseconds */
    delay: number;
    /** Normalized linear power 0..1 */
    power: number;
    rays: Ray[];
}

export interface ScatterChannelState {
    /** Current simulation time (seconds) */
    time: number;
    clusters: ScattererCluster[];
    uePosition: { x: number; y: number };
    bsPosition: { x: number; y: number };
    impulseResponse: { time: number; power: number }[];
}

// Standard Deviation of Log-Normal Variables (from 3GPP defaults)
const SCENARIO_DEFAULTS = {
    dsSigma: 0.4,
    asSigma: 0.4
};

/**
 * Generate random number from standard normal distribution
 */
function randn(): number {
    return Math.sqrt(-2 * Math.log(Math.random())) * Math.cos(2 * Math.PI * Math.random());
}

/**
 * Initialize Scatter Channel State (t=0)
 * Uses Log-Normal statistics for LSPs (Large Scale Parameters)
 * 
 * @param params - Channel configuration parameters
 * @returns Initial channel state
 */
export function initScatterChannel(params: ScatterChannelParams): ScatterChannelState {
    const {
        numClusters, numRaysPerCluster,
        dsMu, asDMu, asAMu, distance
    } = params;

    const c = 0.3; // m/ns

    // Generate LSPs using Log-Normal Distribution
    const actualDS = Math.pow(10, Math.log10(dsMu) + SCENARIO_DEFAULTS.dsSigma * randn());
    const actualASD = Math.pow(10, Math.log10(asDMu) + SCENARIO_DEFAULTS.asSigma * randn());
    const actualASA = Math.pow(10, Math.log10(asAMu) + SCENARIO_DEFAULTS.asSigma * randn());

    // Generate Clusters
    const clusters: ScattererCluster[] = [];
    let totalPower = 0;

    for (let i = 0; i < numClusters; i++) {
        // Delay Distribution: Exponential
        const u = Math.random();
        const minDelay = distance / c;
        const excessDelay = -actualDS * Math.log(u + 0.0001);
        const absDelay = minDelay + excessDelay;

        // Power Distribution: Exponential decay with delay + Shadowing
        const shadowingLog = randn() * 3;
        const rawPower = Math.exp(-excessDelay / actualDS) * Math.pow(10, shadowingLog / 10);
        totalPower += rawPower;

        // Angles: Wrapped Gaussian around LOS
        const aodSpread = actualASD * (Math.PI / 180);
        const aodCenter = 0;
        const aod = aodCenter + randn() * aodSpread;

        const aoaSpread = actualASA * (Math.PI / 180);
        const aoaCenter = Math.PI;
        const aoa = aoaCenter + randn() * aoaSpread;

        // Position from geometric ellipse
        const L = c * absDelay;
        const pathLen = Math.max(L, distance * 1.001);
        const num = pathLen * pathLen - distance * distance;
        const den = 2 * (pathLen - distance * Math.cos(aod));
        const r1 = num / den;

        const sX = r1 * Math.cos(aod);
        const sY = r1 * Math.sin(aod);

        // Generate Rays
        const rays: Ray[] = [];
        for (let r = 0; r < numRaysPerCluster; r++) {
            const subSpread = 2 * (Math.PI / 180);
            const r_aod = aod + (Math.random() - 0.5) * subSpread;
            const r_aoa = aoa + (Math.random() - 0.5) * subSpread;
            const phase = Math.random() * 2 * Math.PI;

            rays.push({
                aoD: r_aod,
                aoA: r_aoa,
                phase,
                power: 0,
                doppler: 0
            });
        }

        clusters.push({
            id: i,
            x: sX,
            y: sY,
            delay: absDelay,
            power: rawPower,
            rays
        });
    }

    // Normalize powers
    for (const cluster of clusters) {
        cluster.power /= totalPower;
    }
    for (const cluster of clusters) {
        for (const ray of cluster.rays) {
            ray.power = cluster.power / numRaysPerCluster;
        }
    }

    const bsPos = { x: 0, y: 0 };
    const uePos = { x: distance, y: 0 };

    return buildState(clusters, bsPos, uePos, 0, params);
}

/**
 * Update Channel State for time evolution
 * Handles UE movement and phase drift
 * 
 * @param prevState - Previous channel state
 * @param dt - Time step (seconds)
 * @param params - Channel parameters
 * @returns Updated channel state
 */
export function updateScatterChannel(
    prevState: ScatterChannelState,
    dt: number,
    params: ScatterChannelParams
): ScatterChannelState {
    const { velocity, carrierFreq } = params;

    const t = prevState.time + dt;
    const lambda = 0.3 / carrierFreq;

    // UE moves linearly along Y-axis
    const ueX = params.distance;
    const ueY = velocity * t;

    const uePos = { x: ueX, y: ueY };
    const bsPos = prevState.bsPosition;

    // Update Phases and Doppler for each cluster
    const nextClusters = prevState.clusters.map(cluster => {
        const sX = cluster.x;
        const sY = cluster.y;

        // Calculate path length change
        const d_S_UE_new = Math.sqrt((uePos.x - sX) ** 2 + (uePos.y - sY) ** 2);
        const d_S_UE_old = Math.sqrt((prevState.uePosition.x - sX) ** 2 + (prevState.uePosition.y - sY) ** 2);
        const deltaL = d_S_UE_new - d_S_UE_old;

        // Update rays
        const nextRays = cluster.rays.map(ray => {
            const phaseShift = -2 * Math.PI * (deltaL / lambda);
            const newPhase = (ray.phase + phaseShift) % (2 * Math.PI);

            // Calculate Doppler
            const rx = sX - ueX;
            const ry = sY - ueY;
            const dist = Math.sqrt(rx * rx + ry * ry);
            const cosTheta = ry / dist;
            const fd = (velocity / lambda) * cosTheta;

            return { ...ray, phase: newPhase, doppler: fd };
        });

        // Update cluster delay
        const d_BS_S = Math.sqrt(sX * sX + sY * sY);
        const totalDist = d_BS_S + d_S_UE_new;
        const newDelay = totalDist / 0.3;

        return { ...cluster, rays: nextRays, delay: newDelay };
    });

    return buildState(nextClusters, bsPos, uePos, t, params);
}

/**
 * Build complete channel state with impulse response
 */
function buildState(
    clusters: ScattererCluster[],
    bsPos: { x: number; y: number },
    uePos: { x: number; y: number },
    time: number,
    params: ScatterChannelParams
): ScatterChannelState {
    const sorted = [...clusters].sort((a, b) => a.delay - b.delay);

    // Calculate instantaneous power (with fading)
    const impulseResponse = sorted.map(c => {
        const rayNorm = c.power / params.numRaysPerCluster;
        let iSum = 0, qSum = 0;
        c.rays.forEach(r => {
            iSum += Math.cos(r.phase);
            qSum += Math.sin(r.phase);
        });
        const instPwr = rayNorm * (iSum * iSum + qSum * qSum);

        return {
            time: c.delay,
            power: 10 * Math.log10(instPwr + 1e-20)
        };
    });

    return {
        time,
        clusters,
        uePosition: uePos,
        bsPosition: bsPos,
        impulseResponse
    };
}

/**
 * Get channel frequency response at specific frequencies
 * 
 * @param state - Current channel state
 * @param frequencies - Array of frequency offsets (Hz)
 * @returns Complex frequency response { real, imag }[]
 */
export function getFrequencyResponse(
    state: ScatterChannelState,
    frequencies: number[]
): { real: number; imag: number }[] {
    return frequencies.map(f => {
        let real = 0, imag = 0;
        state.clusters.forEach(cluster => {
            const delayS = cluster.delay * 1e-9; // ns to s
            const phaseTerm = -2 * Math.PI * f * delayS;
            cluster.rays.forEach(ray => {
                const amp = Math.sqrt(ray.power);
                const totalPhase = ray.phase + phaseTerm;
                real += amp * Math.cos(totalPhase);
                imag += amp * Math.sin(totalPhase);
            });
        });
        return { real, imag };
    });
}
