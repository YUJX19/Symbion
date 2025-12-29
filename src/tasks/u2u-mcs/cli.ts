#!/usr/bin/env npx tsx
/**
 * @module tasks/u2u-mcs/cli
 * @description Command-line interface for U2U-MCS Task2
 * 
 * Usage:
 *   npx tsx src/tasks/u2u-mcs/cli.ts
 *   npx tsx src/tasks/u2u-mcs/cli.ts --seed 123 --episodes 5
 *   npm run task:u2u2
 */

import { runTask2 } from './task2';

// ==================== Argument Parsing ====================

interface CliArgs {
    seed: number;
    episodes: number;
    help: boolean;
}

function parseArgs(): CliArgs {
    const args: CliArgs = {
        seed: 42,
        episodes: 1,
        help: false
    };

    const argv = process.argv.slice(2);

    for (let i = 0; i < argv.length; i++) {
        const arg = argv[i];

        if (arg === '--help' || arg === '-h') {
            args.help = true;
        } else if (arg === '--seed' || arg === '-s') {
            args.seed = parseInt(argv[++i], 10) || 42;
        } else if (arg === '--episodes' || arg === '-e') {
            args.episodes = parseInt(argv[++i], 10) || 1;
        }
    }

    return args;
}

function printHelp(): void {
    console.log(`
U2U-MCS Task2 - AI-Driven Dynamic MCS Selection

Usage:
  npx tsx src/tasks/u2u-mcs/cli.ts [options]

Options:
  -h, --help       Show this help message
  -s, --seed N     Random seed (default: 42)
  -e, --episodes N Number of episodes (default: 1)

Examples:
  npx tsx src/tasks/u2u-mcs/cli.ts
  npx tsx src/tasks/u2u-mcs/cli.ts --seed 123 --episodes 5
`);
}

// ==================== Main ====================

async function main(): Promise<void> {
    const args = parseArgs();

    if (args.help) {
        printHelp();
        process.exit(0);
    }

    console.log('');
    console.log('============================================================');
    console.log('     SYMBION - U2U-MCS Task2 (CLI)                         ');
    console.log('     Paper: Dynamic MCS Selection with Online LSTM         ');
    console.log('============================================================');
    console.log('');

    try {
        const result = await runTask2({
            seed: args.seed,
            totalEpisodes: args.episodes
        });

        console.log('');
        console.log('[OK] Task completed successfully');
        console.log(`     Total Steps: ${result.totalSteps}`);
        console.log(`     Avg Reward:  ${result.avgEpisodeReward.toFixed(2)}`);
        console.log('');
    } catch (error) {
        console.error('');
        console.error('[FAILED] Task failed:');
        console.error(`  ${error instanceof Error ? error.message : String(error)}`);
        console.error('');
        process.exit(1);
    }
}

// Run if executed directly
main();
