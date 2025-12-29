/**
 * @module core/logging
 * @description Unified logging output for reproducible experiments
 *
 * Provides JSONL and CSV loggers with fixed field schemas (versioned, append-only).
 * Supports step-level, episode-level, and report-level structured export.
 *
 * Browser-compatible: ConsoleLogger and MemoryLogger work in all environments.
 * Node.js only: Use `symbion/src/core/logging-node` for file-based loggers.
 */

// ==================== Types ====================

/**
 * Log level for console output
 */
export type LogLevel = 'debug' | 'info' | 'warn' | 'error';

/**
 * Base log entry structure (all logs must include these fields)
 */
export interface BaseLogEntry {
    /** Schema version for compatibility */
    schemaVersion: string;
    /** Task identifier */
    task: string;
    /** Random seed for reproducibility */
    seed: number;
    /** Timestamp in milliseconds */
    timestamp: number;
}

/**
 * Step-level log entry
 */
export interface StepLogEntry extends BaseLogEntry {
    logType: 'step';
    episode: number;
    step: number;
    observation: unknown;
    action: unknown;
    reward: number;
    done: boolean;
    truncated: boolean;
    info?: Record<string, unknown>;
    metrics?: Record<string, number>;
}

/**
 * Episode-level summary log entry
 */
export interface EpisodeLogEntry extends BaseLogEntry {
    logType: 'episode';
    episode: number;
    totalSteps: number;
    totalReward: number;
    avgReward: number;
    success: boolean;
    metrics?: Record<string, number>;
    constraintViolations?: number;
}

/**
 * Report-level log entry (final summary)
 */
export interface ReportLogEntry extends BaseLogEntry {
    logType: 'report';
    totalEpisodes: number;
    totalSteps: number;
    avgEpisodeReward: number;
    avgEpisodeLength: number;
    successRate: number;
    objectiveBreakdown?: Record<string, number>;
    config: Record<string, unknown>;
}

/**
 * Union of all log entry types
 */
export type LogEntry = StepLogEntry | EpisodeLogEntry | ReportLogEntry;

/**
 * Logger interface
 */
export interface Logger {
    /** Log a step */
    logStep(entry: Omit<StepLogEntry, 'logType' | 'schemaVersion' | 'timestamp'>): void;
    /** Log episode summary */
    logEpisode(entry: Omit<EpisodeLogEntry, 'logType' | 'schemaVersion' | 'timestamp'>): void;
    /** Log final report */
    logReport(entry: Omit<ReportLogEntry, 'logType' | 'schemaVersion' | 'timestamp'>): void;
    /** Flush pending writes */
    flush(): void;
    /** Close the logger */
    close(): void;
}

/**
 * Logger configuration
 */
export interface LoggerConfig {
    /** Output directory (Node.js only) */
    outputDir?: string;
    /** Task name (used for file naming) */
    task: string;
    /** Random seed */
    seed: number;
    /** Schema version */
    schemaVersion?: string;
    /** Whether to include step-level logs (can be verbose) */
    logSteps?: boolean;
    /** Buffer size before flushing */
    bufferSize?: number;
}

// ==================== Constants ====================

const DEFAULT_SCHEMA_VERSION = '1.0.0';

// ==================== Console Logger (Browser-compatible) ====================

/**
 * Console Logger: Print to console (for debugging)
 * Works in both browser and Node.js environments.
 */
export class ConsoleLogger implements Logger {
    private level: LogLevel;
    private config: { task: string; seed: number; schemaVersion: string };

    constructor(levelOrConfig: LogLevel | LoggerConfig = 'info') {
        if (typeof levelOrConfig === 'string') {
            this.level = levelOrConfig;
            this.config = { task: 'unknown', seed: 0, schemaVersion: DEFAULT_SCHEMA_VERSION };
        } else {
            this.level = 'info';
            this.config = {
                task: levelOrConfig.task,
                seed: levelOrConfig.seed,
                schemaVersion: levelOrConfig.schemaVersion ?? DEFAULT_SCHEMA_VERSION,
            };
        }
    }

    logStep(entry: Omit<StepLogEntry, 'logType' | 'schemaVersion' | 'timestamp'>): void {
        if (this.level === 'debug') {
            console.log(`[STEP] E${entry.episode} S${entry.step}: reward=${entry.reward.toFixed(3)}`);
        }
    }

    logEpisode(entry: Omit<EpisodeLogEntry, 'logType' | 'schemaVersion' | 'timestamp'>): void {
        if (this.level === 'debug' || this.level === 'info') {
            console.log(
                `[EPISODE] E${entry.episode}: steps=${entry.totalSteps}, ` +
                `reward=${entry.totalReward.toFixed(3)}, success=${entry.success}`
            );
        }
    }

    logReport(entry: Omit<ReportLogEntry, 'logType' | 'schemaVersion' | 'timestamp'>): void {
        console.log(
            `[REPORT] Episodes=${entry.totalEpisodes}, ` +
            `AvgReward=${entry.avgEpisodeReward.toFixed(3)}, ` +
            `SuccessRate=${(entry.successRate * 100).toFixed(1)}%`
        );
    }

    flush(): void { /* no-op */ }
    close(): void { /* no-op */ }
}

// ==================== Memory Logger (Browser-compatible) ====================

/**
 * Memory Logger: Store logs in memory
 * Works in both browser and Node.js environments.
 * Useful for testing and browser-based applications.
 */
export class MemoryLogger implements Logger {
    private config: { task: string; seed: number; schemaVersion: string };
    public steps: StepLogEntry[] = [];
    public episodes: EpisodeLogEntry[] = [];
    public reports: ReportLogEntry[] = [];

    constructor(config: LoggerConfig) {
        this.config = {
            task: config.task,
            seed: config.seed,
            schemaVersion: config.schemaVersion ?? DEFAULT_SCHEMA_VERSION,
        };
    }

    private createBaseEntry(): Pick<BaseLogEntry, 'schemaVersion' | 'task' | 'seed' | 'timestamp'> {
        return {
            schemaVersion: this.config.schemaVersion,
            task: this.config.task,
            seed: this.config.seed,
            timestamp: Date.now(),
        };
    }

    logStep(entry: Omit<StepLogEntry, 'logType' | 'schemaVersion' | 'timestamp'>): void {
        this.steps.push({
            ...this.createBaseEntry(),
            logType: 'step',
            ...entry,
        });
    }

    logEpisode(entry: Omit<EpisodeLogEntry, 'logType' | 'schemaVersion' | 'timestamp'>): void {
        this.episodes.push({
            ...this.createBaseEntry(),
            logType: 'episode',
            ...entry,
        });
    }

    logReport(entry: Omit<ReportLogEntry, 'logType' | 'schemaVersion' | 'timestamp'>): void {
        this.reports.push({
            ...this.createBaseEntry(),
            logType: 'report',
            ...entry,
        });
    }

    /** Get all logs */
    getAllLogs(): LogEntry[] {
        return [...this.steps, ...this.episodes, ...this.reports];
    }

    /** Export to JSON string */
    toJSON(): string {
        return JSON.stringify({
            steps: this.steps,
            episodes: this.episodes,
            reports: this.reports,
        }, null, 2);
    }

    /** Export to JSONL string */
    toJSONL(): string {
        return this.getAllLogs().map(entry => JSON.stringify(entry)).join('\n');
    }

    clear(): void {
        this.steps = [];
        this.episodes = [];
        this.reports = [];
    }

    flush(): void { /* no-op for memory logger */ }
    close(): void { /* no-op for memory logger */ }
}

// ==================== Multi-Logger (Browser-compatible) ====================

/**
 * Multi-Logger: Write to multiple loggers simultaneously
 */
export class MultiLogger implements Logger {
    private loggers: Logger[];

    constructor(loggers: Logger[]) {
        this.loggers = loggers;
    }

    logStep(entry: Omit<StepLogEntry, 'logType' | 'schemaVersion' | 'timestamp'>): void {
        for (const logger of this.loggers) {
            logger.logStep(entry);
        }
    }

    logEpisode(entry: Omit<EpisodeLogEntry, 'logType' | 'schemaVersion' | 'timestamp'>): void {
        for (const logger of this.loggers) {
            logger.logEpisode(entry);
        }
    }

    logReport(entry: Omit<ReportLogEntry, 'logType' | 'schemaVersion' | 'timestamp'>): void {
        for (const logger of this.loggers) {
            logger.logReport(entry);
        }
    }

    flush(): void {
        for (const logger of this.loggers) {
            logger.flush();
        }
    }

    close(): void {
        for (const logger of this.loggers) {
            logger.close();
        }
    }
}

// ==================== Type aliases for Node.js loggers ====================
// Actual implementations are in logging-node.ts (Node.js only)

/**
 * JSONL Logger type - Node.js only
 * Import from 'symbion/src/core/logging-node' for actual implementation
 */
export type JsonlLogger = Logger;

/**
 * CSV Logger type - Node.js only
 * Import from 'symbion/src/core/logging-node' for actual implementation
 */
export type CsvLogger = Logger;

// ==================== Factory Functions ====================

/**
 * Create a logger based on format (browser-compatible)
 */
export function createLogger(
    format: 'console' | 'memory',
    config: LoggerConfig
): Logger {
    switch (format) {
        case 'console':
            return new ConsoleLogger(config);
        case 'memory':
            return new MemoryLogger(config);
    }
}

/**
 * Create a combined logger for browser environment
 */
export function createBrowserLogger(config: LoggerConfig): Logger {
    return new MultiLogger([
        new MemoryLogger(config),
        new ConsoleLogger('info'),
    ]);
}


