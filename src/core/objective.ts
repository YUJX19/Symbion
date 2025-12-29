/**
 * @module core/objective
 * @description MetricSpec and objective function composition with tracking
 *
 * Provides unified objective function and metric interfaces for reproducible research.
 * Supports combining multiple metrics with per-component tracking for paper-level breakdowns.
 */

// ==================== Types ====================

/**
 * Optimization direction for a metric
 */
export type OptimizationDirection = 'maximize' | 'minimize';

/**
 * Metric category for grouping and filtering
 */
export type MetricCategory =
    | 'communication'  // Rate, throughput, BLER
    | 'sensing'        // Coverage, detection, LoS
    | 'energy'         // Power, efficiency
    | 'mobility'       // Speed, acceleration
    | 'reliability'    // URLLC, outage
    | 'custom';

/**
 * Metadata for a metric specification
 */
export interface MetricMetadata {
    /** Unique identifier for the metric */
    id: string;
    /** Human-readable description */
    description: string;
    /** Optimization direction */
    direction: OptimizationDirection;
    /** Unit of measurement (e.g., 'bps', 'dB', 'Joule') */
    unit?: string;
    /** Category for grouping */
    category?: MetricCategory;
    /** Weight in combined objective (default: 1.0) */
    weight?: number;
    /** Version for tracking changes */
    version?: string;
}

/**
 * Metric evaluation function
 * @param state - Current state to evaluate
 * @returns Metric value (higher is better for maximize, lower for minimize)
 */
export type MetricEvaluator<S = unknown> = (state: S) => number;

/**
 * MetricSpec: Metric with metadata for tracking and composition
 */
export interface MetricSpec<S = unknown> {
    /** Metric metadata */
    metadata: MetricMetadata;
    /** Evaluation function */
    evaluate: MetricEvaluator<S>;
}

/**
 * Result of evaluating a combined metric
 */
export interface MetricEvaluationResult {
    /** Combined objective value */
    total: number;
    /** Per-metric breakdown */
    breakdown: Record<string, {
        raw: number;
        weight: number;
        weighted: number;
        direction: OptimizationDirection;
    }>;
    /** Timestamp of evaluation */
    timestamp: number;
}

/**
 * Metric composition configuration
 */
export interface MetricComposition<S = unknown> {
    /** Metrics to combine */
    metrics: MetricSpec<S>[];
    /** Combination method */
    method: 'weighted_sum' | 'product' | 'min' | 'max';
    /** Whether to normalize metrics before combining */
    normalize?: boolean;
    /** Normalization bounds (if normalize is true) */
    normBounds?: Record<string, { min: number; max: number }>;
}

// ==================== Factory Functions ====================

/**
 * Create a MetricSpec from an evaluation function and metadata
 */
export function asMetricSpec<S = unknown>(
    evaluate: MetricEvaluator<S>,
    metadata: MetricMetadata
): MetricSpec<S> {
    return {
        metadata: {
            weight: 1.0,
            version: '1.0',
            ...metadata,
        },
        evaluate,
    };
}

/**
 * Create a simple metric with minimal metadata
 */
export function simpleMetric<S = unknown>(
    id: string,
    evaluate: MetricEvaluator<S>,
    direction: OptimizationDirection = 'maximize'
): MetricSpec<S> {
    return asMetricSpec(evaluate, {
        id,
        description: id,
        direction,
    });
}

// ==================== Metric Transformations ====================

/**
 * Normalize a metric value to [0, 1] range
 */
export function normalizeValue(
    value: number,
    min: number,
    max: number,
    direction: OptimizationDirection
): number {
    if (max === min) return 0.5;
    const normalized = (value - min) / (max - min);
    const clamped = Math.max(0, Math.min(1, normalized));
    // For minimize, invert so that 1 is best
    return direction === 'minimize' ? 1 - clamped : clamped;
}

/**
 * Create a normalized version of a metric
 */
export function normalizeMetric<S>(
    metric: MetricSpec<S>,
    min: number,
    max: number
): MetricSpec<S> {
    return asMetricSpec(
        (state: S) => normalizeValue(
            metric.evaluate(state),
            min,
            max,
            metric.metadata.direction
        ),
        {
            ...metric.metadata,
            id: `${metric.metadata.id}_normalized`,
            description: `${metric.metadata.description} (normalized)`,
            direction: 'maximize', // After normalization, always maximize
        }
    );
}

/**
 * Clamp a metric value to a range
 */
export function clampMetric<S>(
    metric: MetricSpec<S>,
    min: number,
    max: number
): MetricSpec<S> {
    return asMetricSpec(
        (state: S) => Math.max(min, Math.min(max, metric.evaluate(state))),
        {
            ...metric.metadata,
            id: `${metric.metadata.id}_clamped`,
            description: `${metric.metadata.description} (clamped to [${min}, ${max}])`,
        }
    );
}

/**
 * Apply exponential smoothing to a metric (requires history tracking)
 */
export function createSmoothMetric<S>(
    metric: MetricSpec<S>,
    alpha: number = 0.9
): { metric: MetricSpec<S>; reset: () => void } {
    let smoothedValue: number | null = null;

    const smoothMetric = asMetricSpec(
        (state: S) => {
            const raw = metric.evaluate(state);
            if (smoothedValue === null) {
                smoothedValue = raw;
            } else {
                smoothedValue = alpha * smoothedValue + (1 - alpha) * raw;
            }
            return smoothedValue;
        },
        {
            ...metric.metadata,
            id: `${metric.metadata.id}_smooth`,
            description: `${metric.metadata.description} (smoothed, α=${alpha})`,
        }
    );

    return {
        metric: smoothMetric,
        reset: () => { smoothedValue = null; },
    };
}

// ==================== Metric Composition ====================

/**
 * Combine multiple metrics with per-component tracking
 *
 * This is the core function for creating composite objectives
 * with detailed breakdown for paper-level analysis.
 */
export function combineMetricsWithTracking<S>(
    metrics: MetricSpec<S>[],
    method: 'weighted_sum' | 'product' | 'min' | 'max' = 'weighted_sum'
): {
    evaluate: (state: S) => MetricEvaluationResult;
    getMetricIds: () => string[];
} {
    const metricIds = metrics.map(m => m.metadata.id);

    const evaluate = (state: S): MetricEvaluationResult => {
        const breakdown: MetricEvaluationResult['breakdown'] = {};
        const values: number[] = [];

        for (const metric of metrics) {
            const raw = metric.evaluate(state);
            const weight = metric.metadata.weight ?? 1.0;
            const direction = metric.metadata.direction;

            // For minimize metrics, negate the value for combination
            const signedRaw = direction === 'minimize' ? -raw : raw;
            const weighted = signedRaw * weight;

            breakdown[metric.metadata.id] = {
                raw,
                weight,
                weighted,
                direction,
            };

            values.push(weighted);
        }

        // Combine values based on method
        let total: number;
        switch (method) {
            case 'weighted_sum':
                total = values.reduce((sum, v) => sum + v, 0);
                break;
            case 'product':
                total = values.reduce((prod, v) => prod * v, 1);
                break;
            case 'min':
                total = Math.min(...values);
                break;
            case 'max':
                total = Math.max(...values);
                break;
        }

        return {
            total,
            breakdown,
            timestamp: Date.now(),
        };
    };

    return {
        evaluate,
        getMetricIds: () => metricIds,
    };
}

/**
 * Simple metric combination without tracking (returns just the value)
 */
export function combineMetrics<S>(
    metrics: MetricSpec<S>[],
    method: 'weighted_sum' | 'product' | 'min' | 'max' = 'weighted_sum'
): MetricEvaluator<S> {
    const combined = combineMetricsWithTracking(metrics, method);
    return (state: S) => combined.evaluate(state).total;
}

// ==================== Metric Registry ====================

/**
 * Registry for managing and looking up metrics
 */
export class MetricRegistry<S = unknown> {
    private metrics: Map<string, MetricSpec<S>> = new Map();

    /**
     * Register a metric
     */
    register(metric: MetricSpec<S>): this {
        this.metrics.set(metric.metadata.id, metric);
        return this;
    }

    /**
     * Get a metric by ID
     */
    get(id: string): MetricSpec<S> | undefined {
        return this.metrics.get(id);
    }

    /**
     * Check if a metric exists
     */
    has(id: string): boolean {
        return this.metrics.has(id);
    }

    /**
     * List all registered metric IDs
     */
    list(): string[] {
        return Array.from(this.metrics.keys());
    }

    /**
     * List metrics by category
     */
    listByCategory(category: MetricCategory): MetricSpec<S>[] {
        return Array.from(this.metrics.values())
            .filter(m => m.metadata.category === category);
    }

    /**
     * Create a composition from registered metrics
     */
    compose(
        ids: string[],
        method: 'weighted_sum' | 'product' | 'min' | 'max' = 'weighted_sum'
    ): ReturnType<typeof combineMetricsWithTracking<S>> {
        const metrics = ids.map(id => {
            const metric = this.get(id);
            if (!metric) {
                throw new Error(`Metric not found: ${id}`);
            }
            return metric;
        });
        return combineMetricsWithTracking(metrics, method);
    }

    /**
     * Export registry to JSON (for reproducibility)
     */
    toJSON(): Record<string, MetricMetadata> {
        const result: Record<string, MetricMetadata> = {};
        this.metrics.forEach((metric, id) => {
            result[id] = metric.metadata;
        });
        return result;
    }
}

/**
 * Create a new metric registry
 */
export function createMetricRegistry<S = unknown>(): MetricRegistry<S> {
    return new MetricRegistry<S>();
}

/**
 * Global metric registry (convenience for simple use cases)
 */
export const globalMetricRegistry = new MetricRegistry();
