/**
 * @module tasks/isac-trajectory/report
 * @description Report generation for ISAC trajectory task
 */

import type { EpisodeMetrics } from './reward';
import type { ScenarioState } from './scenario';
import type { IsacTrajectoryConfig, Position3D } from './config';

// ==================== Types ====================

/**
 * Full experiment report
 */
export interface IsacReport {
    /** Task name */
    taskName: string;
    /** Configuration used */
    config: IsacTrajectoryConfig;
    /** Policy name */
    policyName: string;
    /** Number of episodes */
    numEpisodes: number;
    /** Random seed */
    seed: number;
    /** Timestamp */
    timestamp: number;
    /** Per-episode metrics */
    episodeMetrics: EpisodeMetrics[];
    /** Aggregate metrics */
    aggregate: AggregateMetrics;
}

/**
 * Aggregate metrics across episodes
 */
export interface AggregateMetrics {
    avgThroughput: { mean: number; std: number };
    avgLosPersistence: { mean: number; std: number };
    totalEnergy: { mean: number; std: number };
    urllcViolationRate: { mean: number; std: number };
    avgSensingCoverage: { mean: number; std: number };
    pathLength: { mean: number; std: number };
    successRate: number;
}

// ==================== Report Generation ====================

/**
 * Calculate mean and standard deviation
 */
function meanStd(values: number[]): { mean: number; std: number } {
    if (values.length === 0) return { mean: 0, std: 0 };
    const mean = values.reduce((a, b) => a + b, 0) / values.length;
    const variance = values.reduce((sum, v) => sum + (v - mean) ** 2, 0) / values.length;
    return { mean, std: Math.sqrt(variance) };
}

/**
 * Generate aggregate metrics from episode metrics
 */
export function aggregateMetrics(episodeMetrics: EpisodeMetrics[]): AggregateMetrics {
    return {
        avgThroughput: meanStd(episodeMetrics.map(m => m.avgThroughput)),
        avgLosPersistence: meanStd(episodeMetrics.map(m => m.avgLosPersistence)),
        totalEnergy: meanStd(episodeMetrics.map(m => m.totalEnergy)),
        urllcViolationRate: meanStd(episodeMetrics.map(m => m.urllcViolationRate)),
        avgSensingCoverage: meanStd(episodeMetrics.map(m => m.avgSensingCoverage)),
        pathLength: meanStd(episodeMetrics.map(m => m.pathLength)),
        successRate: episodeMetrics.filter(m => m.missionSuccess).length / episodeMetrics.length,
    };
}

/**
 * Generate full report
 */
export function generateReport(
    config: IsacTrajectoryConfig,
    policyName: string,
    episodeMetrics: EpisodeMetrics[],
    numEpisodes: number
): IsacReport {
    return {
        taskName: 'isac-trajectory',
        config,
        policyName,
        numEpisodes,
        seed: config.seed,
        timestamp: Date.now(),
        episodeMetrics,
        aggregate: aggregateMetrics(episodeMetrics),
    };
}

// ==================== Export Formats ====================

/**
 * Export report to JSON string
 */
export function reportToJson(report: IsacReport): string {
    return JSON.stringify(report, null, 2);
}

/**
 * Export metrics to CSV string
 */
export function metricsToCSV(episodeMetrics: EpisodeMetrics[]): string {
    const headers = [
        'episode',
        'avgThroughput',
        'avgLosPersistence',
        'totalEnergy',
        'urllcViolationRate',
        'avgSensingCoverage',
        'pathLength',
        'missionSuccess',
    ];

    const rows = episodeMetrics.map((m, i) => [
        i,
        m.avgThroughput.toFixed(2),
        m.avgLosPersistence.toFixed(4),
        m.totalEnergy.toFixed(2),
        m.urllcViolationRate.toFixed(6),
        m.avgSensingCoverage.toFixed(4),
        m.pathLength.toFixed(2),
        m.missionSuccess ? 1 : 0,
    ].join(','));

    return [headers.join(','), ...rows].join('\n');
}

/**
 * Export trajectory to CSV (for plotting)
 */
export function trajectoryToCSV(states: ScenarioState[]): string {
    const headers = [
        'step',
        'x', 'y', 'z',
        'speed',
        'losPercentage',
        'totalThroughput',
        'sensingCoverage',
        'energyUsed',
    ];

    const rows = states.map(s => [
        s.step,
        s.uav.position.x.toFixed(2),
        s.uav.position.y.toFixed(2),
        s.uav.position.z.toFixed(2),
        s.uav.speed.toFixed(2),
        s.losPercentage.toFixed(4),
        s.totalThroughput.toFixed(2),
        s.sensingCoverage.toFixed(4),
        s.uav.energyUsed.toFixed(2),
    ].join(','));

    return [headers.join(','), ...rows].join('\n');
}

/**
 * Generate comparison table (for paper)
 */
export function generateComparisonTable(
    reports: IsacReport[]
): string {
    const headers = [
        'Policy',
        'Throughput (Mbps)',
        'LoS (%)',
        'Energy (J)',
        'URLLC Viol. (%)',
        'Success (%)',
    ];

    const rows = reports.map(r => {
        const a = r.aggregate;
        return [
            r.policyName,
            `${(a.avgThroughput.mean / 1e6).toFixed(2)} ± ${(a.avgThroughput.std / 1e6).toFixed(2)}`,
            `${(a.avgLosPersistence.mean * 100).toFixed(1)} ± ${(a.avgLosPersistence.std * 100).toFixed(1)}`,
            `${a.totalEnergy.mean.toFixed(0)} ± ${a.totalEnergy.std.toFixed(0)}`,
            `${(a.urllcViolationRate.mean * 100).toFixed(2)} ± ${(a.urllcViolationRate.std * 100).toFixed(2)}`,
            `${(a.successRate * 100).toFixed(1)}`,
        ].join(' | ');
    });

    const separator = headers.map(() => '---').join(' | ');

    return [
        headers.join(' | '),
        separator,
        ...rows,
    ].join('\n');
}
