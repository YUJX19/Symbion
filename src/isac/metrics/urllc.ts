/**
 * @module src/isac/metrics/urllc
 * @description URLLC (Ultra-Reliable Low-Latency Communication) Metrics
 */

/**
 * URLLC Requirements
 */
export interface UrllcRequirements {
    /** Target latency (ms) */
    latencyMs: number;
    /** Target reliability (1 - BLER) */
    reliability: number;
    /** Minimum required rate (bps) */
    minRateBps: number;
}

/**
 * URLLC evaluation result
 */
export interface UrllcResult {
    satisfied: boolean;
    latencySatisfied: boolean;
    reliabilitySatisfied: boolean;
    rateSatisfied: boolean;
    actualLatencyMs: number;
    actualReliability: number;
    actualRateBps: number;
}

/**
 * Default URLLC requirements (3GPP)
 */
export const DEFAULT_URLLC_REQUIREMENTS: UrllcRequirements = {
    latencyMs: 1,
    reliability: 0.99999,
    minRateBps: 1e6,
};

/**
 * Calculate BLER from SINR using sigmoid model
 */
export function calculateBler(
    sinrDb: number,
    sinrThresholdDb: number = 5,
    slope: number = 2
): number {
    const x = sinrDb - sinrThresholdDb;
    return 1 / (1 + Math.exp(slope * x));
}

/**
 * Evaluate URLLC constraints
 */
export function evaluateUrllc(
    achievedRateBps: number,
    bler: number,
    latencyMs: number,
    requirements: UrllcRequirements = DEFAULT_URLLC_REQUIREMENTS
): UrllcResult {
    const actualReliability = 1 - bler;

    const latencySatisfied = latencyMs <= requirements.latencyMs;
    const reliabilitySatisfied = actualReliability >= requirements.reliability;
    const rateSatisfied = achievedRateBps >= requirements.minRateBps;

    return {
        satisfied: latencySatisfied && reliabilitySatisfied && rateSatisfied,
        latencySatisfied,
        reliabilitySatisfied,
        rateSatisfied,
        actualLatencyMs: latencyMs,
        actualReliability,
        actualRateBps: achievedRateBps,
    };
}

/**
 * Calculate URLLC violation rate over multiple samples
 */
export function calculateViolationRate(
    results: UrllcResult[]
): number {
    if (results.length === 0) return 0;

    const violations = results.filter(r => !r.satisfied).length;
    return violations / results.length;
}
