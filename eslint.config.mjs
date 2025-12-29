/**
 * Symbion ESLint Configuration (Layered)
 *
 * Purpose:
 * - Framework-agnostic (no React/Next rules)
 * - Library-first: keep Public API strict, allow gradual cleanup internally
 * - Browser + Node globals
 * - Prettier-compatible (disables formatting rules that conflict with Prettier)
 *
 * Layers:
 * 1) Public API files: error on any, warn/error on boundary return types
 * 2) Internal source: warn on any, allow gradual adoption
 * 3) Tests: relaxed rules (dev ergonomics)
 *
 * Project Structure:
 * - index.ts (main entry)
 * - <module>/index.ts (module entries)
 * - <module>/types.ts (type definitions)
 * - <module>/*.ts (internal implementation)
 * - __tests__/*.test.ts (tests)
 */

import eslint from '@eslint/js';
import tseslint from 'typescript-eslint';
import prettierConfig from 'eslint-config-prettier';
import globals from 'globals';

export default tseslint.config(
    // ==================== Global Ignores (build artifacts, deps, etc.) ====================
    {
        ignores: [
            // Build outputs
            'dist/**',
            'build/**',
            'out/**',

            // Dependencies
            'node_modules/**',

            // Coverage & reports
            'coverage/**',
            '.nyc_output/**',

            // Fixtures & mocks
            '__fixtures__/**',
            '__mocks__/**',

            // Generated or vendor files
            '**/*.min.*',

            // Config files
            '*.config.js',
            '*.config.mjs',
            '*.config.cjs',
            '*.config.ts',

            // Type declarations (often generated)
            '*.d.ts',

            // Other
            '.git/**',
            '.cache/**',
            'tmp/**',
            '*.tgz',
        ],
    },

    // ==================== Base Configurations ====================
    eslint.configs.recommended,
    ...tseslint.configs.recommended,
    prettierConfig,

    // ==================== Shared Language Options ====================
    {
        languageOptions: {
            ecmaVersion: 2022,
            sourceType: 'module',
            globals: {
                ...globals.browser,
                ...globals.node,
            },
        },
    },

    // ==================== Layer 1: Public API (STRICT) ====================
    // Main entry point and module index files that define the public interface
    {
        files: [
            'index.ts',           // Main library entry
            '*/index.ts',         // Module entries (beamforming/index.ts, etc.)
            '*/types.ts',         // Exported type definitions
            '**/public*.ts',      // Any file prefixed with 'public'
        ],
        rules: {
            // Strongly protect the external contract
            '@typescript-eslint/no-explicit-any': 'error',

            // Enforce explicit types at the boundary
            '@typescript-eslint/explicit-module-boundary-types': 'warn',

            // Keep public API clean
            '@typescript-eslint/no-unused-vars': ['error', {
                argsIgnorePattern: '^_',
                varsIgnorePattern: '^_',
                caughtErrorsIgnorePattern: '^_',
            }],

            // Library best-practice baseline
            'prefer-const': 'error',
            'no-var': 'error',
            'eqeqeq': ['error', 'always'],
            'no-duplicate-imports': 'error',
            'no-loss-of-precision': 'error',
        },
    },

    // ==================== Layer 2: Internal Source (GRADUAL) ====================
    // All other TypeScript files (implementation details)
    {
        files: ['**/*.ts'],
        rules: {
            // Gradual type-safety adoption: discourage any without blocking iteration
            '@typescript-eslint/no-explicit-any': 'warn',

            // Typed "no-unsafe-*" can be very noisy in math-heavy code.
            // Keep off initially; plan to enable module-by-module later.
            '@typescript-eslint/no-unsafe-assignment': 'off',
            '@typescript-eslint/no-unsafe-call': 'off',
            '@typescript-eslint/no-unsafe-member-access': 'off',
            '@typescript-eslint/no-unsafe-return': 'off',
            '@typescript-eslint/no-unsafe-argument': 'off',

            // Encourage safer patterns
            '@typescript-eslint/no-non-null-assertion': 'warn',

            // Warning for unused variables (allow gradual cleanup)
            '@typescript-eslint/no-unused-vars': ['warn', {
                argsIgnorePattern: '^_',
                varsIgnorePattern: '^_',
                caughtErrorsIgnorePattern: '^_',
            }],

            // Stylistic / consistency
            '@typescript-eslint/array-type': ['warn', { default: 'array' }],

            // Allow console for algorithm debugging
            'no-console': 'off',

            // General best practices
            'prefer-const': 'error',
            'no-var': 'error',
            'eqeqeq': ['error', 'always'],
            'no-duplicate-imports': 'error',
            'no-loss-of-precision': 'error',
        },
    },

    // ==================== Layer 3: Tests (RELAXED) ====================
    {
        files: ['__tests__/**/*.ts', '**/*.test.ts', '**/*.spec.ts'],
        rules: {
            // Tests often need flexibility
            '@typescript-eslint/no-explicit-any': 'off',
            '@typescript-eslint/no-non-null-assertion': 'off',

            // Allow unused vars in tests (common for placeholders)
            '@typescript-eslint/no-unused-vars': ['warn', {
                argsIgnorePattern: '^_',
                varsIgnorePattern: '^_',
                caughtErrorsIgnorePattern: '^_',
            }],

            // Allow console output for diagnostics
            'no-console': 'off',

            // Keep basic safety rules
            'eqeqeq': ['error', 'always'],
            'no-loss-of-precision': 'error',
        },
    },
);
