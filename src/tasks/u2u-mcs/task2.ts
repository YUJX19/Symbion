/**
 * @module tasks/u2u-mcs/task2
 * @description Task 2: AI-Driven Dynamic MCS Selection
 * 
 * Production-grade implementation demonstrating:
 * - Core Module: Runner, TaskConfig, SeededRandom, JSONL Logging
 * - AI Module: ProtocolHandler, Transport, Schema Validation
 * 
 * ## Usage
 * ```typescript
 * import { runTask2 } from 'symbion/tasks';
 * 
 * const result = await runTask2({ seed: 42, totalEpisodes: 1 });
 * console.log(result.avgEpisodeReward);
 * ```
 */

import * as path from 'path';
import * as fs from 'fs';
import { ProtocolHandler } from '../../ai/interface/protocol';
import { Runner, type RunnerConfig } from '../../core/runner';
import { createTaskConfig } from '../../core/repro';
import { ConsoleLogger, MemoryLogger, MultiLogger } from '../../core/logging';

// Local modules
import { createEnvironment } from './environment';
import {
    PAPER_SCENARIO_CONFIG,
    DEFAULT_TASK2_RUN_CONFIG,
    type Task2RunConfig
} from './config';
import { TASK2_SCHEMA, TASK2_SCHEMA_HASH, validateSchemaHash } from './schema';
import { createHelloMsg, validateHelloAck, type HelloAckResponse } from './protocol';
import { createStdioTransport, type StdioTransport } from './transport';
import { createRemotePolicy, type RemotePolicy } from './policy';
import { U2UMcsError, schemaMismatchError, protocolError } from './errors';

// ==================== Types ====================

export interface Task2Result {
    totalSteps: number;
    totalEpisodes: number;
    avgEpisodeReward: number;
    avgEpisodeLength: number;
    successRate: number;
    durationMs: number;
    logFile?: string;
}

// ==================== Main Function ====================

/**
 * Run Task 2: AI-Driven Dynamic MCS Selection
 * 
 * @param runConfig - Runtime configuration overrides
 * @returns Experiment results
 */
export async function runTask2(
    runConfig: Partial<Task2RunConfig> = {}
): Promise<Task2Result> {
    // Merge with defaults
    const config = { ...DEFAULT_TASK2_RUN_CONFIG, ...runConfig };

    console.log('========================================');
    console.log('Task 2: AI-Driven Dynamic MCS Selection');
    console.log('========================================');
    console.log(`Seed: ${config.seed}`);
    console.log(`Episodes: ${config.totalEpisodes}`);
    console.log(`Agent: ${config.agentScript}`);
    console.log('');

    // 1. Create Environment
    const env = createEnvironment(PAPER_SCENARIO_CONFIG);

    // 2. Create TaskConfig (Core Module: repro)
    const taskConfig = createTaskConfig({
        taskName: 'u2u-mcs-task2',
        seed: config.seed,
        schemaHash: TASK2_SCHEMA_HASH,
        hyperparams: PAPER_SCENARIO_CONFIG as unknown as Record<string, unknown>
    });

    // 3. Setup Transport (AI Module)
    const agentScript = path.resolve(__dirname, config.agentScript);
    const transport = createStdioTransport(config.pythonCommand, agentScript, __dirname);

    // 4. Setup Protocol (AI Module)
    const protocol = new ProtocolHandler(`task2-${taskConfig.seed}`);

    // Wire protocol to transport
    transport.onEvent((e) => {
        if (e.type === 'message' && typeof e.data === 'string') {
            try {
                protocol.handleMessage(protocol.parseMessage(e.data));
            } catch (err) {
                console.error('[Protocol] Parse error:', err);
            }
        }
    });

    // 5. Connect and Handshake
    await transport.connect();
    console.log('[Transport] Connected');

    const handshakeOk = await performHandshake(transport, protocol, config.agentTimeout);
    if (!handshakeOk) {
        transport.disconnect();
        throw protocolError('Handshake failed');
    }
    console.log('[Protocol] Handshake successful');
    console.log(`[Schema] Hash: ${TASK2_SCHEMA_HASH.slice(0, 16)}...`);

    // 6. Create Decision Maker (AI Module)
    const decisionMaker = createRemotePolicy(protocol, transport, taskConfig, {
        timeoutMs: config.agentTimeout,
        enableLogging: config.enableConsoleOutput
    });

    // 7. Setup Loggers (Core Module: logging)
    const memoryLogger = new MemoryLogger({
        task: taskConfig.taskName,
        seed: taskConfig.seed
    });

    const loggers = config.enableConsoleOutput
        ? new MultiLogger([memoryLogger, new ConsoleLogger('info')])
        : memoryLogger;

    // 8. Create Runner (Core Module: runner)
    const runnerConfig: RunnerConfig = {
        taskConfig,
        environment: env as any,
        decisionMaker: decisionMaker as any,
        loggers: [loggers],
        maxStepsPerEpisode: PAPER_SCENARIO_CONFIG.episodeLength,
        totalEpisodes: config.totalEpisodes,
        stopOnConstraintViolation: false
    };

    const runner = new Runner(runnerConfig);

    // 9. Run Experiment
    console.log('\n[Runner] Starting experiment...\n');
    const startTime = Date.now();

    try {
        const result = await runner.run();
        const durationMs = Date.now() - startTime;

        // 10. Write JSONL Log (Core Module: logging)
        let logFile: string | undefined;
        if (config.enableJsonlFile) {
            logFile = await writeJsonlLog(memoryLogger, config.logDir, taskConfig);
        }

        // 11. Print Summary
        console.log('\n========================================');
        console.log('Experiment Complete');
        console.log('========================================');
        console.log(`Duration: ${(durationMs / 1000).toFixed(2)}s`);
        console.log(`Total Steps: ${result.totalSteps}`);
        console.log(`Avg Episode Metric: ${result.avgEpisodeMetric.toFixed(2)}`);
        console.log(`Success Rate: ${(result.successRate * 100).toFixed(1)}%`);
        if (logFile) {
            console.log(`Log File: ${logFile}`);
        }

        return {
            totalSteps: result.totalSteps,
            totalEpisodes: result.totalEpisodes,
            avgEpisodeReward: result.avgEpisodeMetric,
            avgEpisodeLength: result.avgEpisodeLength,
            successRate: result.successRate,
            durationMs,
            logFile
        };
    } finally {
        // 12. Cleanup
        await decisionMaker.close();
        transport.disconnect();
        runner.close();
    }
}

// ==================== Helpers ====================

/**
 * Perform protocol handshake with schema validation
 */
async function performHandshake(
    transport: StdioTransport,
    protocol: ProtocolHandler,
    timeoutMs: number
): Promise<boolean> {
    try {
        const reqId = protocol.generateRequestId();
        const msg = createHelloMsg(protocol, reqId);

        transport.send(protocol.serializeMessage(msg));

        const response = await protocol.registerRequest<HelloAckResponse>(reqId, timeoutMs);

        if (!validateHelloAck(response)) {
            console.error('[Handshake] Schema mismatch or rejected');
            console.error(`[Handshake] Expected: ${TASK2_SCHEMA_HASH}`);
            console.error(`[Handshake] Received: ${response.schemaHash}`);
            return false;
        }

        return true;
    } catch (e) {
        console.error('[Handshake] Error:', e);
        return false;
    }
}

/**
 * Write JSONL log to file
 */
async function writeJsonlLog(
    logger: MemoryLogger,
    logDir: string,
    taskConfig: { taskName: string; seed: number }
): Promise<string> {
    // Ensure directory exists
    const fullLogDir = path.resolve(__dirname, logDir);
    if (!fs.existsSync(fullLogDir)) {
        fs.mkdirSync(fullLogDir, { recursive: true });
    }

    // Generate filename
    const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
    const filename = `${taskConfig.taskName}_seed${taskConfig.seed}_${timestamp}.jsonl`;
    const filepath = path.join(fullLogDir, filename);

    // Write JSONL
    const jsonl = logger.toJSONL();
    fs.writeFileSync(filepath, jsonl);

    return filepath;
}

