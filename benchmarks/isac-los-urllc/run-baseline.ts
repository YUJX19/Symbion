#!/usr/bin/env npx ts-node
/**
 * @module benchmarks/isac-trajectory/run-baseline
 * @description Run ISAC Trajectory baseline benchmarks
 *
 * Usage:
 *   npx ts-node benchmarks/isac-trajectory/run-baseline.ts --config=baseline-suburban.json --policy=proposed
 *   npx ts-node benchmarks/isac-trajectory/run-baseline.ts --config=baseline-suburban.json --all
 */

import * as fs from 'fs';
import * as path from 'path';

// Import from symbion
import { tasks, core } from '../../index';

const {
    IsacTrajectoryEnvironment,
    createPolicy,
    getAvailablePolicies,
    DEFAULT_ISAC_CONFIG,
    metricsToCSV,
    trajectoryToCSV,
    generateReport,
    generateComparisonTable,
} = tasks.isacTrajectory;

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
        config: 'baseline-suburban.json',
        all: false,
        episodes: 10,
        output: 'benchmarks/isac-trajectory/results',
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

async function runBenchmark(
    configPath: string,
    policyName: string,
    numEpisodes: number,
    outputDir: string,
    seed?: number
) {
    // Load config
    const configFile = path.join('benchmarks/isac-los-urllc/configs', configPath);
    const configJson = JSON.parse(fs.readFileSync(configFile, 'utf-8'));
    const config = { ...DEFAULT_ISAC_CONFIG, ...configJson, seed: seed ?? configJson.seed };

    // Create environment
    const env = new IsacTrajectoryEnvironment(config);

    // Create policy
    const policy = createPolicy(policyName, config);

    // Create output directory
    if (!fs.existsSync(outputDir)) {
        fs.mkdirSync(outputDir, { recursive: true });
    }

    const timestamp = Date.now();
    const logPrefix = `${policyName}_${configPath.replace('.json', '')}_${timestamp}`;

    const episodeMetrics: ReturnType<typeof env.getEpisodeMetrics>[] = [];
    const allTrajectories: ReturnType<typeof env.getStateHistory>[] = [];

    console.log(`\n Running ${policyName} on ${configPath} for ${numEpisodes} episodes...`);

    for (let ep = 0; ep < numEpisodes; ep++) {
        // Reset environment
        const { input } = await env.reset(config.seed + ep);

        // Reset policy
        policy.reset?.();

        let done = false;
        let truncated = false;
        let currentObs = input;

        // Seeded random for policy
        const rng = new core.SeededRandom(config.seed + ep);

        const numUsers = config.users.length;
        const baseIdx = 6; // 3 pos + 3 vel

        // Episode loop
        while (!done && !truncated) {
            // Convert observation vector to object
            const obsObj = {
                normalizedPosition: [currentObs[0], currentObs[1], currentObs[2]],
                normalizedVelocity: [currentObs[3], currentObs[4], currentObs[5]],
                losStatus: currentObs.slice(baseIdx, baseIdx + numUsers),
                normalizedSinr: currentObs.slice(baseIdx + numUsers, baseIdx + 2 * numUsers),
                normalizedDistances: currentObs.slice(baseIdx + 2 * numUsers, baseIdx + 3 * numUsers),
                losPercentage: currentObs[baseIdx + 3 * numUsers],
                urllcSatisfied: currentObs.slice(baseIdx + 3 * numUsers + 1, baseIdx + 4 * numUsers + 1),
                normalizedEnergy: currentObs[baseIdx + 4 * numUsers + 1],
                progress: currentObs[baseIdx + 4 * numUsers + 2],
            };

            // Get action from policy
            const action = await Promise.resolve(policy.decide(obsObj, rng));

            // Step environment
            const result = await env.step(action);
            currentObs = result.input;
            done = result.done;
            truncated = result.truncated;
        }

        // Get episode metrics
        const metrics = env.getEpisodeMetrics();
        episodeMetrics.push(metrics);

        // Store trajectory for first episode
        if (ep === 0) {
            allTrajectories.push(env.getStateHistory());
        }

        // Progress
        if ((ep + 1) % 5 === 0 || ep === numEpisodes - 1) {
            console.log(`  Episode ${ep + 1}/${numEpisodes}: LoS=${(metrics.avgLosPersistence * 100).toFixed(1)}%, URLLC_viol=${(metrics.urllcViolationRate * 100).toFixed(2)}%`);
        }
    }

    // Generate report
    const report = generateReport(config, policyName, episodeMetrics, numEpisodes);

    // Save report
    const reportPath = path.join(outputDir, `${logPrefix}_report.json`);
    fs.writeFileSync(reportPath, JSON.stringify(report, null, 2));

    // Save metrics CSV
    const metricsPath = path.join(outputDir, `${logPrefix}_metrics.csv`);
    fs.writeFileSync(metricsPath, metricsToCSV(episodeMetrics));

    // Save first trajectory
    if (allTrajectories.length > 0) {
        const trajPath = path.join(outputDir, `${logPrefix}_trajectory.csv`);
        fs.writeFileSync(trajPath, trajectoryToCSV(allTrajectories[0]));
    }

    // Print summary
    const a = report.aggregate;
    console.log(`\n ${policyName} Results:`);
    console.log(`   Throughput: ${(a.avgThroughput.mean / 1e6).toFixed(2)} ± ${(a.avgThroughput.std / 1e6).toFixed(2)} Mbps`);
    console.log(`   LoS: ${(a.avgLosPersistence.mean * 100).toFixed(1)} ± ${(a.avgLosPersistence.std * 100).toFixed(1)}%`);
    console.log(`   Energy: ${a.totalEnergy.mean.toFixed(0)} ± ${a.totalEnergy.std.toFixed(0)} J`);
    console.log(`   URLLC Viol: ${(a.urllcViolationRate.mean * 100).toFixed(2)}%`);
    console.log(`   Success Rate: ${(a.successRate * 100).toFixed(1)}%`);

    env.close();
    return report;
}

// ==================== Main ====================

async function main() {
    const args = parseArgs();

    console.log('================================================');
    console.log('    ISAC Trajectory Baseline Benchmark Runner   ');
    console.log('================================================');

    const policiesToRun = args.all
        ? getAvailablePolicies()
        : args.policy
            ? [args.policy]
            : ['proposed'];

    const allReports: ReturnType<typeof generateReport>[] = [];

    for (const policy of policiesToRun) {
        const report = await runBenchmark(
            args.config,
            policy,
            args.episodes,
            args.output,
            args.seed
        );
        allReports.push(report);
    }

    // Generate comparison table
    if (allReports.length > 1) {
        console.log('\n Comparison Table:');
        console.log(generateComparisonTable(allReports));

        const tablePath = path.join(args.output, `comparison_${args.config.replace('.json', '')}_${Date.now()}.md`);
        fs.writeFileSync(tablePath, generateComparisonTable(allReports));
    }

    console.log(`\n Results saved to ${args.output}/`);
}

main().catch(console.error);
