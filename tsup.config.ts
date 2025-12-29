import { defineConfig } from 'tsup'

/**
 * Optimized tsup configuration for symbion library
 * 
 * Key optimizations:
 * - splitting: true → Extracts shared code to chunks, reducing duplication
 * - minify: true → Production-ready compressed output
 * - Unified build → All entries share common chunks
 * 
 * Expected output:
 * - Main bundles link to shared chunk-*.js files
 * - Total size reduced from ~1.7M to ~800K
 */
export default defineConfig({
    name: 'symbion',

    entry: {
        // ==================== Main Entries ====================
        // Default entry (includes all modules, suitable for Node.js)
        index: 'index.ts',

        // Explicit Node.js entry (full functionality)
        node: 'node.ts',

        // Browser-safe entry (excludes Node.js-dependent modules)
        browser: 'browser.ts',

        // ==================== Sub-path Exports ====================
        'src/core': 'src/core/index.ts',
        'src/ai': 'src/ai/index.ts',
        'src/isac': 'src/isac/index.ts',
        'src/models': 'src/models/index.ts',
        'src/tasks': 'src/tasks/index.ts',
        'src/extras': 'src/extras/index.ts',
    },

    format: ['cjs', 'esm'],
    dts: true,

    // ==================== Key Optimizations ====================
    splitting: true,        // ✅ Extract shared dependencies to chunks
    minify: true,          // ✅ Production compression
    treeshake: true,       // ✅ Remove unused code

    sourcemap: false,      // No sourcemaps for release builds
    clean: true,           // Clean dist before build

    // ==================== External Dependencies ====================
    // Node.js built-ins should NOT be bundled
    // They will be resolved at runtime in Node.js environment
    // In browser context, imports using these will fail at build time (as intended)
    external: [
        'fs',
        'path',
        'child_process',
        'readline',
    ],

    outDir: 'dist',
    target: 'es2020',

    // Neutral platform: let the bundler/runtime resolve environment-specific modules
    platform: 'neutral',
})
