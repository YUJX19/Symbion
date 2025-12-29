/**
 * @module src/isac/constraints/communication
 * @description Communication Constraints for ISAC
 */

/**
 * Communication constraints configuration
 */
export interface CommunicationConstraints {
    minSinrDb: number;
    maxBler: number;
    minThroughputBps: number;
    maxOutageProbability: number;
}

/**
 * Default communication constraints
 */
export const DEFAULT_COMM_CONSTRAINTS: CommunicationConstraints = {
    minSinrDb: 0,
    maxBler: 0.1,
    minThroughputBps: 1e6,
    maxOutageProbability: 0.01,
};

/**
 * Check SINR constraint
 */
export function checkSinr(
    sinrDb: number,
    constraints: CommunicationConstraints
): boolean {
    return sinrDb >= constraints.minSinrDb;
}

/**
 * Check BLER constraint
 */
export function checkBler(
    bler: number,
    constraints: CommunicationConstraints
): boolean {
    return bler <= constraints.maxBler;
}

/**
 * Check throughput constraint
 */
export function checkThroughput(
    throughputBps: number,
    constraints: CommunicationConstraints
): boolean {
    return throughputBps >= constraints.minThroughputBps;
}

/**
 * Validate all communication constraints
 */
export function validateCommConstraints(
    sinrDb: number,
    bler: number,
    throughputBps: number,
    constraints: CommunicationConstraints = DEFAULT_COMM_CONSTRAINTS
): { valid: boolean; violations: string[] } {
    const violations: string[] = [];

    if (!checkSinr(sinrDb, constraints)) {
        violations.push('sinr_violation');
    }
    if (!checkBler(bler, constraints)) {
        violations.push('bler_violation');
    }
    if (!checkThroughput(throughputBps, constraints)) {
        violations.push('throughput_violation');
    }

    return { valid: violations.length === 0, violations };
}
