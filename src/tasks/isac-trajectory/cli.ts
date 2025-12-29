#!/usr/bin/env npx tsx
/**
 * @module tasks/isac-trajectory/cli
 * @description Command-line interface for ISAC Trajectory Task1
 * 
 * Usage:
 *   npx tsx src/tasks/isac-trajectory/cli.ts
 *   npm run task:isac1
 */

import { runTask1 } from './task1';

async function main(): Promise<void> {
    console.log('');
    console.log('============================================================');
    console.log('     SYMBION - ISAC Trajectory Task1 (CLI)                ');
    console.log('     Paper: UAV-UAV Communication-Aware Trajectory        ');
    console.log('============================================================');
    console.log('');

    try {
        // Run task with default configuration
        await runTask1();

        console.log('');
        console.log('[OK] Task completed successfully');
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
