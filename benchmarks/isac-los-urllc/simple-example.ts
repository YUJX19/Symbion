#!/usr/bin/env npx tsx
/**
 * @module benchmarks/isac-los-urllc/simple-example
 * @description Simple example demonstrating ISAC trajectory task usage
 * 
 * This example shows how to:
 * 1. Use the RL Environment (step-based simulation)
 * 2. Use different baseline policies
 * 3. Collect metrics and generate reports
 * 
 * Usage:
 *   npx tsx benchmarks/isac-los-urllc/simple-example.ts
 */

// Import from the isac-trajectory module
import {
    // RL Environment
    IsacTrajectoryEnvironment,
    createConfig,
    createPolicy,
    getAvailablePolicies,
    // Input helpers
    extractInput,
    // Report generation
    generateReport,
    generateComparisonTable,
    type IsacTrajectoryConfig,
    type EpisodeMetrics,
    type IsacReport,
} from '../../src/tasks/isac-trajectory';

// Import RNG from core
import { createRng } from '../../src/core/repro';

// ==========================================
// Configuration
// ==========================================

const SIMPLE_CONFIG: Partial<IsacTrajectoryConfig> = {
    seed: 42,
    numWaypoints: 10,           // Short episode
    episodeHorizon: 10,         // 10 steps per episode
    areaBounds: [-200, -200, 200, 200],
    startPosition: { x: 0, y: 0, z: 100 },
    // 2 ground users
    users: [
        { id: 'u1', position: { x: 50, y: 0, z: 0 }, minRate: 1e6, maxLatencyMs: 1, reliabilityTarget: 0.99999, isUrllc: true },
        { id: 'u2', position: { x: -50, y: 30, z: 0 }, minRate: 5e6, maxLatencyMs: 100, reliabilityTarget: 0.99, isUrllc: false },
    ],
    obstacles: [],  // No obstacles for simplicity
};

// ==========================================
// Run a single episode with a policy
// ==========================================

async function runEpisode(
    config: IsacTrajectoryConfig,
    policyName: string,
    seed: number
): Promise<EpisodeMetrics> {
    // Create environment for this episode
    const env = new IsacTrajectoryEnvironment(config);
    const policy = createPolicy(policyName, config);
    const rng = createRng(seed);

    // Reset environment
    const { info } = await env.reset(seed);
    policy.reset?.();

    console.log(`  Episode start: UAV at (${info.position.x.toFixed(0)}, ${info.position.y.toFixed(0)}, ${info.position.z.toFixed(0)})`);

    let done = false;
    let truncated = false;
    let totalReward = 0;
    let currentInfo = info;

    // Step loop
    while (!done && !truncated) {
        // Get current state for observation
        const stateHistory = env.getStateHistory();
        const currentState = stateHistory[stateHistory.length - 1];

        // Extract input in the correct format
        const input = extractInput(currentState, config);

        // Select action using policy
        const actionResult = policy.decide(input, rng);
        const action = Array.isArray(actionResult) ? actionResult : await actionResult;

        // Take step
        const result = await env.step(action);
        done = result.done;
        truncated = result.truncated;
        currentInfo = result.info;
        totalReward += result.metric;
    }

    console.log(`  Episode end: total reward = ${totalReward.toFixed(2)}, LoS = ${(currentInfo.losPercentage * 100).toFixed(1)}%`);

    return env.getEpisodeMetrics();
}

// ==========================================
// Main benchmark
// ==========================================

async function main() {
    console.log('');
    console.log('============================================================');
    console.log('     ISAC Trajectory - Simple Benchmark Example             ');
    console.log('============================================================');
    console.log('');

    // Create configuration
    const config = createConfig(SIMPLE_CONFIG);

    // List available policies
    const policies = getAvailablePolicies();
    console.log(`Available policies: ${policies.join(', ')}`);
    console.log('');

    // Test each policy
    const reports: IsacReport[] = [];
    const policiesToTest = ['hover', 'random', 'proposed'];

    for (const policyName of policiesToTest) {
        console.log(`\nRunning policy: ${policyName}`);
        console.log('-'.repeat(40));

        // Run 3 episodes
        const episodeMetrics: EpisodeMetrics[] = [];
        for (let ep = 0; ep < 3; ep++) {
            console.log(`  Episode ${ep + 1}/3:`);
            const metrics = await runEpisode(config, policyName, config.seed + ep);
            episodeMetrics.push(metrics);
        }

        // Generate report
        const report = generateReport(config, policyName, episodeMetrics, 3);
        reports.push(report);

        // Print summary
        console.log(`\n  Summary for ${policyName}:`);
        console.log(`     Avg Throughput: ${(report.aggregate.avgThroughput.mean / 1e6).toFixed(2)} Mbps`);
        console.log(`     Avg LoS: ${(report.aggregate.avgLosPersistence.mean * 100).toFixed(1)}%`);
        console.log(`     Total Energy: ${report.aggregate.totalEnergy.mean.toFixed(0)} J`);
        console.log(`     Success Rate: ${(report.aggregate.successRate * 100).toFixed(0)}%`);
    }

    // Generate comparison table
    console.log('\n');
    console.log('Comparison Table (Markdown format):');
    console.log('='.repeat(60));
    console.log(generateComparisonTable(reports));
    console.log('='.repeat(60));

    console.log('\n[OK] Benchmark completed!');
    console.log('');
}

// Run
main().catch(console.error);
