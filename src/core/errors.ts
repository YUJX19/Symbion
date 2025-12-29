/**
 * @module core/errors
 * @description Unified error types and error codes for consistent cross-task exception handling
 *
 * Provides standardized error codes to ensure consistent, controllable exception behavior
 * across all tasks and protocols.
 */

// ==================== Error Codes ====================

/**
 * Standard error codes for the Symbion framework
 */
export const ErrorCodes = {
    // Protocol & Communication Errors
    /** Request timeout */
    TIMEOUT: 'TIMEOUT',
    /** Protocol version mismatch or invalid message format */
    PROTOCOL_ERROR: 'PROTOCOL_ERROR',
    /** Unknown or mismatched request ID */
    UNKNOWN_REQUEST_ID: 'UNKNOWN_REQUEST_ID',
    /** Schema hash mismatch (observation/action space changed) */
    SCHEMA_MISMATCH: 'SCHEMA_MISMATCH',
    /** Connection error (disconnected, failed to connect) */
    CONNECTION_ERROR: 'CONNECTION_ERROR',
    /** Concurrent step rejected in strict mode */
    CONCURRENT_STEP: 'CONCURRENT_STEP',

    // Validation Errors
    /** Generic validation failure */
    VALIDATION_ERROR: 'VALIDATION_ERROR',
    /** Action validation failed (out of range, wrong type) */
    INVALID_ACTION: 'INVALID_ACTION',
    /** Observation validation failed */
    INVALID_OBSERVATION: 'INVALID_OBSERVATION',
    /** Configuration validation failed */
    INVALID_CONFIG: 'INVALID_CONFIG',

    // Constraint & Optimization Errors
    /** Hard constraint violation */
    CONSTRAINT_VIOLATION: 'CONSTRAINT_VIOLATION',
    /** Optimization failed to converge */
    OPTIMIZATION_FAILED: 'OPTIMIZATION_FAILED',
    /** Infeasible problem (no valid solution exists) */
    INFEASIBLE: 'INFEASIBLE',

    // Task & Runtime Errors
    /** Task not found or not registered */
    TASK_NOT_FOUND: 'TASK_NOT_FOUND',
    /** Episode terminated unexpectedly */
    EPISODE_TERMINATED: 'EPISODE_TERMINATED',
    /** Runner step called before reset */
    NOT_INITIALIZED: 'NOT_INITIALIZED',

    // Remote & External Errors
    /** Remote agent error (Python side) */
    REMOTE_ERROR: 'REMOTE_ERROR',
    /** Internal server/framework error */
    INTERNAL_ERROR: 'INTERNAL_ERROR',
} as const;

export type ErrorCode = (typeof ErrorCodes)[keyof typeof ErrorCodes];

// ==================== Error Classes ====================

/**
 * Base error class for Symbion framework
 */
export class SymbionError extends Error {
    readonly code: ErrorCode;
    readonly details?: unknown;
    readonly timestamp: number;

    constructor(code: ErrorCode, message: string, details?: unknown) {
        super(message);
        this.name = 'SymbionError';
        this.code = code;
        this.details = details;
        this.timestamp = Date.now();

        // Maintain proper stack trace in V8
        if (Error.captureStackTrace) {
            Error.captureStackTrace(this, SymbionError);
        }
    }

    /**
     * Convert to JSON-serializable object
     */
    toJSON(): {
        name: string;
        code: ErrorCode;
        message: string;
        details: unknown;
        timestamp: number;
    } {
        return {
            name: this.name,
            code: this.code,
            message: this.message,
            details: this.details,
            timestamp: this.timestamp,
        };
    }
}

/**
 * Timeout error
 */
export class TimeoutError extends SymbionError {
    constructor(message = 'Operation timed out', details?: unknown) {
        super(ErrorCodes.TIMEOUT, message, details);
        this.name = 'TimeoutError';
    }
}

/**
 * Protocol error (version mismatch, invalid message format)
 */
export class ProtocolError extends SymbionError {
    constructor(message: string, details?: unknown) {
        super(ErrorCodes.PROTOCOL_ERROR, message, details);
        this.name = 'ProtocolError';
    }
}

/**
 * Validation error (invalid action, observation, or config)
 */
export class ValidationError extends SymbionError {
    constructor(message: string, details?: unknown) {
        super(ErrorCodes.VALIDATION_ERROR, message, details);
        this.name = 'ValidationError';
    }
}

/**
 * Constraint violation error
 */
export class ConstraintViolationError extends SymbionError {
    readonly violations: ConstraintViolationDetail[];

    constructor(message: string, violations: ConstraintViolationDetail[]) {
        super(ErrorCodes.CONSTRAINT_VIOLATION, message, violations);
        this.name = 'ConstraintViolationError';
        this.violations = violations;
    }
}

/**
 * Detail of a single constraint violation
 */
export interface ConstraintViolationDetail {
    constraintId: string;
    value: number;
    threshold: number;
    severity: 'hard' | 'soft';
}

/**
 * Schema mismatch error
 */
export class SchemaMismatchError extends SymbionError {
    readonly expectedHash: string;
    readonly actualHash: string;

    constructor(expectedHash: string, actualHash: string) {
        super(
            ErrorCodes.SCHEMA_MISMATCH,
            `Schema hash mismatch: expected ${expectedHash}, got ${actualHash}`,
            { expectedHash, actualHash }
        );
        this.name = 'SchemaMismatchError';
        this.expectedHash = expectedHash;
        this.actualHash = actualHash;
    }
}

/**
 * Connection error
 */
export class ConnectionError extends SymbionError {
    constructor(message: string, details?: unknown) {
        super(ErrorCodes.CONNECTION_ERROR, message, details);
        this.name = 'ConnectionError';
    }
}

/**
 * Not initialized error (step called before reset)
 */
export class NotInitializedError extends SymbionError {
    constructor(message = 'Runner not initialized. Call reset() first.') {
        super(ErrorCodes.NOT_INITIALIZED, message);
        this.name = 'NotInitializedError';
    }
}

// ==================== Error Utilities ====================

/**
 * Check if an error is a SymbionError
 */
export function isSymbionError(error: unknown): error is SymbionError {
    return error instanceof SymbionError;
}

/**
 * Check if an error has a specific error code
 */
export function hasErrorCode(error: unknown, code: ErrorCode): boolean {
    return isSymbionError(error) && error.code === code;
}

/**
 * Wrap any error into a SymbionError
 */
export function wrapError(error: unknown, defaultCode: ErrorCode = ErrorCodes.INTERNAL_ERROR): SymbionError {
    if (isSymbionError(error)) {
        return error;
    }

    if (error instanceof Error) {
        return new SymbionError(defaultCode, error.message, {
            originalName: error.name,
            originalStack: error.stack,
        });
    }

    return new SymbionError(defaultCode, String(error));
}

/**
 * Create a timeout promise that rejects after specified milliseconds
 */
export function createTimeout(ms: number, message?: string): Promise<never> {
    return new Promise((_, reject) => {
        setTimeout(() => {
            reject(new TimeoutError(message ?? `Timeout after ${ms}ms`));
        }, ms);
    });
}

/**
 * Race a promise against a timeout
 */
export async function withTimeout<T>(
    promise: Promise<T>,
    ms: number,
    message?: string
): Promise<T> {
    return Promise.race([promise, createTimeout(ms, message)]);
}
