/**
 * @module tasks/u2u-mcs/transport
 * @description StdioTransport for Python agent communication
 * 
 * Reusable transport implementation for spawning and communicating with
 * Python agents via stdin/stdout.
 */

import { spawn, type ChildProcess } from 'child_process';
import * as readline from 'readline';
import { BaseTransport, type TransportOptions } from '../../ai/interface/transport/transport';

// ==================== Types ====================

export interface StdioTransportOptions extends TransportOptions {
    /** Python command (e.g., 'python3') */
    command: string;
    /** Script arguments */
    args: string[];
    /** Working directory */
    cwd?: string;
    /** Environment variables */
    env?: Record<string, string>;
}

// ==================== StdioTransport ====================

/**
 * Transport that spawns a subprocess and communicates via stdio
 * 
 * - stdout: JSON protocol messages (one per line)
 * - stderr: Logs (inherited to parent's stderr)
 * - stdin: JSON protocol messages from TypeScript
 */
export class StdioTransport extends BaseTransport {
    private process: ChildProcess | null = null;
    private rl: readline.Interface | null = null;
    private readonly command: string;
    private readonly args: string[];
    private readonly cwd: string;
    private readonly env?: Record<string, string>;

    constructor(options: StdioTransportOptions) {
        super({ host: 'localhost', port: 0 }); // BaseTransport requires these but we don't use them
        this.command = options.command;
        this.args = options.args;
        this.cwd = options.cwd ?? process.cwd();
        this.env = options.env;
    }

    /**
     * Spawn the subprocess and set up communication
     */
    async connect(): Promise<void> {
        if (this.process) return;
        this.setState('connecting');

        try {
            this.process = spawn(this.command, this.args, {
                stdio: ['pipe', 'pipe', 'inherit'],
                cwd: this.cwd,
                env: this.env ? { ...process.env, ...this.env } : process.env,
            });

            if (!this.process.stdout || !this.process.stdin) {
                throw new Error('Failed to create stdio pipes');
            }

            // Set up line-based reading from stdout
            this.rl = readline.createInterface({
                input: this.process.stdout,
                terminal: false
            });

            this.rl.on('line', (line) => {
                const trimmed = line.trim();
                if (!trimmed) return;
                this.emit({ type: 'message', data: trimmed });
            });

            // Handle process events
            this.process.on('close', (code) => {
                this.setState('disconnected');
                this.emit({ type: 'disconnected', data: code });
            });

            this.process.on('error', (err) => {
                this.setState('error');
                this.emit({ type: 'error', error: err });
            });

            this.setState('connected');
            this.emit({ type: 'connected' });
        } catch (error) {
            this.setState('error');
            this.emit({ type: 'error', error: error as Error });
            throw error;
        }
    }

    /**
     * Send a message to the subprocess via stdin
     */
    send(message: string): void {
        if (!this.process || !this.process.stdin) {
            throw new Error('Transport not connected');
        }
        this.process.stdin.write(message + '\n');
    }

    /**
     * Disconnect and kill the subprocess
     */
    disconnect(): void {
        if (this.rl) {
            this.rl.close();
            this.rl = null;
        }
        if (this.process) {
            this.process.kill();
            this.process = null;
        }
        this.setState('disconnected');
    }

    /**
     * Check if the subprocess is still running
     */
    isProcessAlive(): boolean {
        return this.process !== null && !this.process.killed;
    }
}

// ==================== Factory ====================

/**
 * Create a StdioTransport for a Python agent
 */
export function createStdioTransport(
    pythonCommand: string,
    scriptPath: string,
    cwd?: string
): StdioTransport {
    return new StdioTransport({
        command: pythonCommand,
        args: [scriptPath],
        cwd,
        host: 'localhost',
        port: 0
    });
}
