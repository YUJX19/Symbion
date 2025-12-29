import { defineConfig } from 'vitest/config';
import { resolve } from 'path';

export default defineConfig({
    test: {
        globals: true,
        environment: 'node',
        include: ['__tests__/**/*.test.ts', 'ai/__tests__/**/*.test.ts'],
        exclude: [
            'node_modules',
            'dist',
            // Exclude script-style tests that don't use Vitest describe/it format
            '__tests__/flight-control.test.ts',
            '__tests__/trajectory-tracking.test.ts',
        ],
        coverage: {
            provider: 'v8',
            reporter: ['text', 'json', 'html'],
            include: [
                '**/*.ts',
            ],
            exclude: [
                '__tests__/**',
                'node_modules/**',
                'dist/**',
                '*.config.ts',
                'types.ts',
            ],
            thresholds: {
                lines: 60,
                functions: 60,
                branches: 50,
                statements: 60,
            },
        },
        testTimeout: 30000,
        hookTimeout: 10000,
        pool: 'forks',
        reporters: ['default'],
        passWithNoTests: false,
    },
    resolve: {
        alias: {
            '@': resolve(__dirname, '.'),
        },
    },
});
