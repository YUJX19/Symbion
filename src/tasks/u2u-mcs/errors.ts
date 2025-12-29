/**
 * @module tasks/u2u-mcs/errors
 * @description Unified error codes for U2U-MCS tasks
 * 
 * Error codes are aligned between TypeScript and Python for consistent handling.
 */

// ==================== Error Codes ====================

/**
 * Error code definitions for U2U-MCS protocol
 */
export const U2U_MCS_ERROR_CODES = {
    /** Schema hash mismatch between agent and environment */
    SCHEMA_MISMATCH: 'SCHEMA_MISMATCH',
    /** Invalid observation or action format */
    VALIDATION_ERROR: 'VALIDATION_ERROR',
    /** Protocol message format error */
    PROTOCOL_ERROR: 'PROTOCOL_ERROR',
    /** Internal error in agent or environment */
    INTERNAL_ERROR: 'INTERNAL_ERROR',
    /** Request timeout */
    TIMEOUT: 'TIMEOUT',
    /** Agent not initialized */
    NOT_INITIALIZED: 'NOT_INITIALIZED'
} as const;

export type U2UMcsErrorCode = typeof U2U_MCS_ERROR_CODES[keyof typeof U2U_MCS_ERROR_CODES];

// ==================== Error Class ====================

/**
 * U2U-MCS specific error with code and details
 */
export class U2UMcsError extends Error {
    readonly name = 'U2UMcsError';

    constructor(
        public readonly code: U2UMcsErrorCode,
        message: string,
        public readonly details?: unknown
    ) {
        super(`[${code}] ${message}`);
    }

    /**
     * Convert to protocol error response format
     */
    toProtocolError(): { code: string; message: string; details?: unknown } {
        return {
            code: this.code,
            message: this.message,
            details: this.details
        };
    }
}

// ==================== Error Factories ====================

/**
 * Create schema mismatch error
 */
export function schemaMismatchError(expected: string, received: string): U2UMcsError {
    return new U2UMcsError(
        'SCHEMA_MISMATCH',
        `Schema hash mismatch: expected ${expected}, received ${received}`,
        { expected, received }
    );
}

/**
 * Create validation error
 */
export function validationError(message: string, details?: unknown): U2UMcsError {
    return new U2UMcsError('VALIDATION_ERROR', message, details);
}

/**
 * Create protocol error
 */
export function protocolError(message: string, details?: unknown): U2UMcsError {
    return new U2UMcsError('PROTOCOL_ERROR', message, details);
}

/**
 * Create timeout error
 */
export function timeoutError(requestId: string, timeoutMs: number): U2UMcsError {
    return new U2UMcsError(
        'TIMEOUT',
        `Request ${requestId} timed out after ${timeoutMs}ms`,
        { requestId, timeoutMs }
    );
}

/**
 * Create internal error
 */
export function internalError(message: string, details?: unknown): U2UMcsError {
    return new U2UMcsError('INTERNAL_ERROR', message, details);
}
