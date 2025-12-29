/**
 * @module core/repro
 * @description Reproducibility guarantees for experiments
 *
 * Provides TaskConfig serialization, schema hash computation, and version tracking
 * for ensuring long-term reproducibility of experiments.
 */

// Core version - should match package.json
const CORE_VERSION = '2.0.2';

// ==================== Browser-compatible Hash ====================

/**
 * Simple hash function that works in both browser and Node.js
 * Uses djb2 algorithm for fast, consistent hashing
 */
function simpleHash(str: string): string {
    let hash = 5381;
    for (let i = 0; i < str.length; i++) {
        hash = ((hash << 5) + hash) ^ str.charCodeAt(i);
    }
    // Convert to hex string
    return (hash >>> 0).toString(16).padStart(8, '0');
}

/**
 * Create a hash from a string (browser-compatible)
 */
function createHash(data: string): string {
    // Use multiple rounds for better distribution
    const h1 = simpleHash(data);
    const h2 = simpleHash(data + h1);
    const h3 = simpleHash(h1 + data);
    const h4 = simpleHash(h2 + h3);
    return h1 + h2 + h3 + h4;
}

// ==================== Types ====================

/**
 * Complete task configuration for reproducibility
 */
export interface TaskConfig {
    /** Task name identifier */
    taskName: string;
    /** Task version (e.g., '1.0.0') */
    version: string;
    /** Random seed for reproducibility */
    seed: number;
    /** Schema hash of observation/action spaces */
    schemaHash: string;
    /** Library version */
    libraryVersion: string;
    /** Timestamp when config was created */
    createdAt: number;
    /** Task-specific hyperparameters */
    hyperparams: Record<string, unknown>;
    /** Optional description */
    description?: string;
    /** Optional tags for categorization */
    tags?: string[];
}

/**
 * Minimal config for creating a TaskConfig
 */
export interface TaskConfigInput {
    taskName: string;
    version?: string;
    seed: number;
    schemaHash: string;
    hyperparams: Record<string, unknown>;
    description?: string;
    tags?: string[];
}

/**
 * Validation result for TaskConfig
 */
export interface ValidationResult {
    valid: boolean;
    errors: string[];
    warnings: string[];
}

// ==================== Factory Functions ====================

/**
 * Create a TaskConfig with defaults
 */
export function createTaskConfig(input: TaskConfigInput): TaskConfig {
    return {
        taskName: input.taskName,
        version: input.version ?? '1.0.0',
        seed: input.seed,
        schemaHash: input.schemaHash,
        libraryVersion: CORE_VERSION,
        createdAt: Date.now(),
        hyperparams: input.hyperparams,
        description: input.description,
        tags: input.tags,
    };
}

// ==================== Serialization ====================

/**
 * Serialize TaskConfig to a canonical JSON string
 *
 * Uses deterministic key ordering for consistent hashing.
 */
export function serializeTaskConfig(config: TaskConfig): string {
    // Create a copy with sorted keys
    const sorted = sortObjectKeys(config);
    return JSON.stringify(sorted, null, 2);
}

/**
 * Deserialize JSON string to TaskConfig
 */
export function deserializeTaskConfig(json: string): TaskConfig {
    const parsed = JSON.parse(json);
    return validateAndNormalize(parsed);
}

/**
 * Compute a hash of the TaskConfig for quick comparison
 */
export function computeConfigHash(config: TaskConfig): string {
    // Exclude timestamp and non-essential fields
    const essential = {
        taskName: config.taskName,
        version: config.version,
        seed: config.seed,
        schemaHash: config.schemaHash,
        hyperparams: sortObjectKeys(config.hyperparams),
    };
    const canonical = JSON.stringify(essential);
    return createHash(canonical);
}

// ==================== Validation ====================

/**
 * Validate a TaskConfig
 */
export function validateTaskConfig(config: TaskConfig): ValidationResult {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Required fields
    if (!config.taskName || typeof config.taskName !== 'string') {
        errors.push('taskName is required and must be a string');
    }

    if (!config.version || typeof config.version !== 'string') {
        errors.push('version is required and must be a string');
    }

    if (typeof config.seed !== 'number' || !Number.isInteger(config.seed)) {
        errors.push('seed must be an integer');
    }

    if (!config.schemaHash || typeof config.schemaHash !== 'string') {
        errors.push('schemaHash is required and must be a string');
    }

    if (typeof config.hyperparams !== 'object' || config.hyperparams === null) {
        errors.push('hyperparams must be an object');
    }

    // Warnings
    if (config.libraryVersion !== CORE_VERSION) {
        warnings.push(
            `libraryVersion mismatch: config was created with ${config.libraryVersion}, ` +
            `current version is ${CORE_VERSION}`
        );
    }

    return {
        valid: errors.length === 0,
        errors,
        warnings,
    };
}

/**
 * Check if two TaskConfigs are compatible (same schema)
 */
export function areConfigsCompatible(a: TaskConfig, b: TaskConfig): boolean {
    return (
        a.taskName === b.taskName &&
        a.version === b.version &&
        a.schemaHash === b.schemaHash
    );
}

// ==================== Seeded Random ====================

/**
 * Seeded random number generator (Mulberry32)
 *
 * Use this instead of Math.random() for reproducibility.
 */
export class SeededRandom {
    private state: number;

    constructor(seed: number) {
        this.state = seed >>> 0;
    }

    /**
     * Generate a random float in [0, 1)
     */
    random(): number {
        this.state = (this.state + 0x6D2B79F5) >>> 0;
        let t = this.state;
        t = Math.imul(t ^ (t >>> 15), t | 1);
        t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    }

    /**
     * Generate a random integer in [min, max)
     */
    randint(min: number, max: number): number {
        return Math.floor(this.random() * (max - min)) + min;
    }

    /**
     * Generate a random float in [min, max)
     */
    uniform(min: number, max: number): number {
        return this.random() * (max - min) + min;
    }

    /**
     * Generate a random sample from a normal distribution
     */
    normal(mean: number = 0, std: number = 1): number {
        // Box-Muller transform
        const u1 = this.random();
        const u2 = this.random();
        const z = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
        return mean + std * z;
    }

    /**
     * Shuffle an array in place
     */
    shuffle<T>(array: T[]): T[] {
        for (let i = array.length - 1; i > 0; i--) {
            const j = this.randint(0, i + 1);
            [array[i], array[j]] = [array[j], array[i]];
        }
        return array;
    }

    /**
     * Choose n random elements from an array
     */
    sample<T>(array: T[], n: number): T[] {
        const shuffled = [...array];
        this.shuffle(shuffled);
        return shuffled.slice(0, n);
    }

    /**
     * Get the current state (for saving/restoring)
     */
    getState(): number {
        return this.state;
    }

    /**
     * Set the state (for restoring)
     */
    setState(state: number): void {
        this.state = state >>> 0;
    }
}

/**
 * Create a seeded random number generator
 */
export function createRng(seed: number): SeededRandom {
    return new SeededRandom(seed);
}

// ==================== Utility Functions ====================

/**
 * Sort object keys recursively for deterministic serialization
 */
function sortObjectKeys(obj: unknown): unknown {
    if (obj === null || typeof obj !== 'object') {
        return obj;
    }

    if (Array.isArray(obj)) {
        return obj.map(sortObjectKeys);
    }

    const sorted: Record<string, unknown> = {};
    for (const key of Object.keys(obj as object).sort()) {
        sorted[key] = sortObjectKeys((obj as Record<string, unknown>)[key]);
    }
    return sorted;
}

/**
 * Validate and normalize a parsed TaskConfig
 */
function validateAndNormalize(parsed: unknown): TaskConfig {
    if (typeof parsed !== 'object' || parsed === null) {
        throw new Error('Invalid TaskConfig: must be an object');
    }

    const obj = parsed as Record<string, unknown>;

    return {
        taskName: String(obj.taskName ?? ''),
        version: String(obj.version ?? '1.0.0'),
        seed: Number(obj.seed ?? 0),
        schemaHash: String(obj.schemaHash ?? ''),
        libraryVersion: String(obj.libraryVersion ?? CORE_VERSION),
        createdAt: Number(obj.createdAt ?? Date.now()),
        hyperparams: (typeof obj.hyperparams === 'object' && obj.hyperparams !== null)
            ? obj.hyperparams as Record<string, unknown>
            : {},
        description: obj.description ? String(obj.description) : undefined,
        tags: Array.isArray(obj.tags) ? obj.tags.map(String) : undefined,
    };
}

// ==================== Experiment Metadata ====================

/**
 * Generate experiment metadata for logging
 */
export interface ExperimentMetadata {
    config: TaskConfig;
    configHash: string;
    startedAt: number;
    hostname?: string;
    platform?: string;
    nodeVersion?: string;
}

/**
 * Create experiment metadata
 */
export function createExperimentMetadata(config: TaskConfig): ExperimentMetadata {
    return {
        config,
        configHash: computeConfigHash(config),
        startedAt: Date.now(),
        hostname: typeof process !== 'undefined' ? process.env.HOSTNAME : undefined,
        platform: typeof process !== 'undefined' ? process.platform : undefined,
        nodeVersion: typeof process !== 'undefined' ? process.version : undefined,
    };
}
