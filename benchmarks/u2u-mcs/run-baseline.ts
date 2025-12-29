#!/usr/bin/env npx ts-node
/**
 * @module benchmarks/u2u-mcs/run-baseline
 * @description Run U2U-MCS baseline benchmarks
 *
 * Usage:
 *   npx ts-node benchmarks/u2u-mcs/run-baseline.ts --config=baseline-high-speed.json --policy=greedy
 *   npx ts-node benchmarks/u2u-mcs/run-baseline.ts --config=baseline-high-speed.json --all
 */

import * as fs from 'fs';
import * as path from 'path';

// Import from symbion
import { tasks, core } from '../../index';

const {
    U2UMcsEnvironment,
    createPolicy,
    getAvailablePolicies,
    DEFAULT_U2U_MCS_CONFIG,
} = tasks.u2uMcs;

const {
    createTaskConfig,
    computeSchemaHash,
} = core;

// ==================== CLI Parsing ====================

interface CliArgs {
    config: string;
    policy?: string;
    all: boolean;
    episodes: number;
    output: string;
    seed?: number;
}

function parseArgs(): CliArgs {
    const args = process.argv.slice(2);
    const result: CliArgs = {
        config: 'baseline-high-speed.json',
        all: false,
        episodes: 10,
        output: 'benchmarks/u2u-mcs/results',
    };

    for (const arg of args) {
        if (arg.startsWith('--config=')) {
            result.config = arg.split('=')[1];
        } else if (arg.startsWith('--policy=')) {
            result.policy = arg.split('=')[1];
        } else if (arg === '--all') {
            result.all = true;
        } else if (arg.startsWith('--episodes=')) {
            result.episodes = parseInt(arg.split('=')[1], 10);
        } else if (arg.startsWith('--output=')) {
            result.output = arg.split('=')[1];
        } else if (arg.startsWith('--seed=')) {
            result.seed = parseInt(arg.split('=')[1], 10);
        }
    }

    return result;
}

// ==================== Benchmark Runner ====================

interface EpisodeSummary {
    policy: string;
    episode: number;
    totalSteps: number;
    avgThroughput: number;
    p5Throughput: number;
    avgBler: number;
    urllcSuccessRate: number;
    mcsSwitchRate: number;
    totalReward: number;
}

async function runBenchmark(
    configPath: string,
    policyName: string,
    numEpisodes: number,
    outputDir: string,
    seed?: number
): Promise<EpisodeSummary[]> {
    // Load config
    const configFile = path.join('benchmarks/u2u-mcs/configs', configPath);
    const configJson = JSON.parse(fs.readFileSync(configFile, 'utf-8'));
    const config = { ...DEFAULT_U2U_MCS_CONFIG, ...configJson, seed: seed ?? configJson.seed };

    // Create environment
    const env = new U2UMcsEnvironment(config);

    // Create policy
    const policy = createPolicy(policyName, config);

    // Create output directory
    if (!fs.existsSync(outputDir)) {
        fs.mkdirSync(outputDir, { recursive: true });
    }

    // Create logger
    const timestamp = Date.now();
    const logPrefix = `${policyName}_${configPath.replace('.json', '')}_${timestamp}`;

    const summaries: EpisodeSummary[] = [];

    console.log(`\n Running ${policyName} on ${configPath} for ${numEpisodes} episodes...`);

    for (let ep = 0; ep < numEpisodes; ep++) {
        // Reset environment
        const { input } = await env.reset(config.seed + ep);

        let done = false;
        let truncated = false;
        let currentObs = input;

        // Seeded random for policy
        const rng = new core.SeededRandom(config.seed + ep);

        // Episode loop
        while (!done && !truncated) {
            const obsObj = {
                sinrDb: currentObs[0] * 30,
                distance: currentObs[1] * 200,
                relativeSpeed: currentObs[2] * 30,
                recentAckRate: currentObs[3],
                recentBler: currentObs[4],
                lastMcs: Math.round(currentObs[5] * 22),
                normalizedStep: currentObs[6],
            };

            const action = await Promise.resolve(policy.decide(obsObj, rng));

            // Step environment
            const result = await env.step(action);
            currentObs = result.input;
            done = result.done;
            truncated = result.truncated;
        }

        // Get episode summary
        const summary = env.getEpisodeSummary();

        summaries.push({
            policy: policyName,
            episode: ep,
            totalSteps: summary.totalSteps,
            avgThroughput: summary.avgThroughput,
            p5Throughput: summary.p5Throughput,
            avgBler: summary.avgBler,
            urllcSuccessRate: summary.urllcSuccessRate,
            mcsSwitchRate: summary.mcsSwitchRate,
            totalReward: summary.totalReward,
        });

        // Progress
        if ((ep + 1) % 5 === 0 || ep === numEpisodes - 1) {
            console.log(`  Episode ${ep + 1}/${numEpisodes}: reward=${summary.totalReward.toFixed(2)}, BLER=${summary.avgBler.toFixed(4)}`);
        }
    }

    // Save summaries
    const summaryPath = path.join(outputDir, `${logPrefix}_summary.json`);
    fs.writeFileSync(summaryPath, JSON.stringify(summaries, null, 2));

    // Calculate aggregate
    const avgReward = summaries.reduce((s, e) => s + e.totalReward, 0) / summaries.length;
    const avgBler = summaries.reduce((s, e) => s + e.avgBler, 0) / summaries.length;
    const avgUrllc = summaries.reduce((s, e) => s + e.urllcSuccessRate, 0) / summaries.length;

    console.log(`\n ${policyName} Results:`);
    console.log(`   Avg Reward: ${avgReward.toFixed(3)}`);
    console.log(`   Avg BLER: ${avgBler.toFixed(6)}`);
    console.log(`   URLLC Success: ${(avgUrllc * 100).toFixed(2)}%`);

    env.close();
    return summaries;
}

// ==================== Main ====================

async function main() {
    const args = parseArgs();

    console.log('╔════════════════════════════════════════════════╗');
    console.log('║       U2U-MCS Baseline Benchmark Runner        ║');
    console.log('╚════════════════════════════════════════════════╝');

    const policiesToRun = args.all
        ? getAvailablePolicies()
        : args.policy
            ? [args.policy]
            : ['greedy'];

    const allResults: Record<string, EpisodeSummary[]> = {};

    for (const policy of policiesToRun) {
        const results = await runBenchmark(
            args.config,
            policy,
            args.episodes,
            args.output,
            args.seed
        );
        allResults[policy] = results;
    }

    // Save comparison summary
    const comparisonPath = path.join(args.output, `comparison_${args.config.replace('.json', '')}_${Date.now()}.json`);
    fs.writeFileSync(comparisonPath, JSON.stringify(allResults, null, 2));

    console.log(`\n Results saved to ${args.output}/`);
    console.log(`   Comparison: ${comparisonPath}`);
}

main().catch(console.error);
