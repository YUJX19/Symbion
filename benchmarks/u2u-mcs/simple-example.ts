#!/usr/bin/env npx tsx
/**
 * @module benchmarks/u2u-mcs/simple-example
 * @description Simple example demonstrating U2U-MCS task usage
 * 
 * This example shows how to:
 * 1. Use the RL Environment (step-based simulation)
 * 2. Use different baseline policies
 * 3. Collect metrics and generate reports
 * 
 * Usage:
 *   npx tsx benchmarks/u2u-mcs/simple-example.ts
 */

// Import from u2u-mcs task
import {
    createEnvironment,
    PAPER_SCENARIO_CONFIG,
    FixedMcsPolicy,
    GreedyMcsPolicy,
    type U2UMcsConfig,
} from '../../src/tasks/u2u-mcs';

// Import RNG from core
import { createRng, type SeededRandom } from '../../src/core/repro';

// ==========================================
// Configuration
// ==========================================

// Shorter episode for quick demo
const DEMO_CONFIG: U2UMcsConfig = {
    ...PAPER_SCENARIO_CONFIG,
    episodeLength: 500,  // Shorter episode for demo
};

// ==========================================
// Episode Summary
// ==========================================

interface EpisodeSummary {
    policy: string;
    totalSteps: number;
    totalReward: number;
    avgReward: number;
    avgThroughputMbps: number;
    avgBler: number;
    urllcSuccessRate: number;
}

// ==========================================
// Run a single episode with a policy
// ==========================================

async function runEpisode(
    config: U2UMcsConfig,
    policy: { name: string; decide: (obs: any, rng: SeededRandom) => number; reset?: () => void },
    seed: number
): Promise<EpisodeSummary> {
    const env = createEnvironment(config);
    const rng = createRng(seed);

    // Reset
    const { input, info } = await env.reset(seed);
    policy.reset?.();

    let currentObs = input;
    let totalReward = 0;
    let totalThroughput = 0;
    let totalBler = 0;
    let urllcSuccess = 0;
    let step = 0;
    let done = false;
    let truncated = false;

    // Episode loop
    while (!done && !truncated) {
        // Convert normalized observation to policy input format
        const obsObj = {
            sinrDb: currentObs[0] * 30,
            distance: currentObs[1] * 200,
            relativeSpeed: currentObs[2] * 30,
            recentAckRate: currentObs[3],
            recentBler: currentObs[4],
            lastMcs: Math.round(currentObs[5] * 22),
            normalizedStep: currentObs[6],
        };

        // Select action
        const action = policy.decide(obsObj, rng);

        // Step environment
        const result = await env.step(action);
        currentObs = result.input;
        done = result.info.step >= config.episodeLength;
        truncated = false;

        // Accumulate metrics
        totalReward += result.info.rewardBreakdown.total;
        totalThroughput += result.info.throughput;
        totalBler += result.info.bler;
        if (result.info.bler <= 0.1) urllcSuccess++;
        step++;
    }

    return {
        policy: policy.name,
        totalSteps: step,
        totalReward,
        avgReward: totalReward / step,
        avgThroughputMbps: totalThroughput / step / 1e6,
        avgBler: totalBler / step,
        urllcSuccessRate: urllcSuccess / step,
    };
}

// ==========================================
// Policies to test
// ==========================================

function createPolicies(config: U2UMcsConfig) {
    return [
        new FixedMcsPolicy(10),           // Fixed MCS=10 (middle)
        new FixedMcsPolicy(3),            // Fixed MCS=3 (conservative)
        new GreedyMcsPolicy(config),      // Greedy SINR-based
    ];
}

// ==========================================
// Main benchmark
// ==========================================

async function main() {
    console.log('');
    console.log('============================================================');
    console.log('     U2U-MCS - Simple Benchmark Example                     ');
    console.log('============================================================');
    console.log('');

    console.log('Configuration:');
    console.log(`  Episode Length: ${DEMO_CONFIG.episodeLength} steps`);
    console.log(`  BLER Target: ${DEMO_CONFIG.blerTarget}`);
    console.log(`  Bandwidth: ${DEMO_CONFIG.bandwidthHz / 1e6} MHz`);
    console.log('');

    // Create policies
    const policies = createPolicies(DEMO_CONFIG);
    console.log(`Policies to test: ${policies.map(p => p.name).join(', ')}`);
    console.log('');

    // Test each policy
    const results: EpisodeSummary[] = [];

    for (const policy of policies) {
        console.log(`\nRunning policy: ${policy.name}`);
        console.log('-'.repeat(40));

        // Run 3 episodes
        const episodeResults: EpisodeSummary[] = [];
        for (let ep = 0; ep < 3; ep++) {
            const seed = 42 + ep;
            console.log(`  Episode ${ep + 1}/3 (seed=${seed})...`);
            const summary = await runEpisode(DEMO_CONFIG, policy, seed);
            episodeResults.push(summary);
            console.log(`    Reward: ${summary.totalReward.toFixed(2)}, BLER: ${summary.avgBler.toFixed(4)}, Throughput: ${summary.avgThroughputMbps.toFixed(2)} Mbps`);
        }

        // Average across episodes
        const avgResult: EpisodeSummary = {
            policy: policy.name,
            totalSteps: episodeResults[0].totalSteps,
            totalReward: episodeResults.reduce((s, r) => s + r.totalReward, 0) / 3,
            avgReward: episodeResults.reduce((s, r) => s + r.avgReward, 0) / 3,
            avgThroughputMbps: episodeResults.reduce((s, r) => s + r.avgThroughputMbps, 0) / 3,
            avgBler: episodeResults.reduce((s, r) => s + r.avgBler, 0) / 3,
            urllcSuccessRate: episodeResults.reduce((s, r) => s + r.urllcSuccessRate, 0) / 3,
        };
        results.push(avgResult);

        console.log(`\n  Summary for ${policy.name}:`);
        console.log(`     Avg Reward: ${avgResult.totalReward.toFixed(2)}`);
        console.log(`     Avg BLER: ${avgResult.avgBler.toFixed(4)}`);
        console.log(`     Avg Throughput: ${avgResult.avgThroughputMbps.toFixed(2)} Mbps`);
        console.log(`     URLLC Success: ${(avgResult.urllcSuccessRate * 100).toFixed(1)}%`);
    }

    // Print comparison table
    console.log('\n');
    console.log('Comparison Table:');
    console.log('='.repeat(80));
    console.log('| Policy          | Reward    | BLER    | Throughput | URLLC Success |');
    console.log('|-----------------|-----------|---------|------------|---------------|');
    for (const r of results) {
        const name = r.policy.padEnd(15);
        const reward = r.totalReward.toFixed(2).padStart(9);
        const bler = r.avgBler.toFixed(4).padStart(7);
        const thr = `${r.avgThroughputMbps.toFixed(2)} Mbps`.padStart(10);
        const urllc = `${(r.urllcSuccessRate * 100).toFixed(1)}%`.padStart(13);
        console.log(`| ${name} | ${reward} | ${bler} | ${thr} | ${urllc} |`);
    }
    console.log('='.repeat(80));

    console.log('\n[OK] Benchmark completed!');
    console.log('');
}

// Run
main().catch(console.error);
