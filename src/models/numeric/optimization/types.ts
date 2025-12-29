/**
 * @module optimization/types
 * @description Type definitions for optimization algorithms
 */

/**
 * L-BFGS optimization configuration
 */
export interface LBFGSConfig {
    /** Maximum number of iterations */
    maxIterations: number;
    /** Memory size for L-BFGS history */
    memorySize: number;
    /** Convergence tolerance for gradient norm */
    tolerance: number;
    /** Small epsilon for numerical stability */
    epsilon: number;
    /** Relative cost decrease tolerance */
    relCostTol?: number;
}

/**
 * Optimization result
 */
export interface OptimizationResult {
    /** Optimal solution vector */
    solution: number[];
    /** Final cost value */
    cost: number;
    /** Number of iterations performed */
    iterations: number;
    /** Whether optimization converged */
    converged: boolean;
    /** Final gradient norm */
    gradientNorm: number;
}

/**
 * Cost function signature
 */
export type CostFunction = (x: number[]) => { cost: number; gradient: number[] };
