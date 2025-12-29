/**
 * @module planning/optimization-history
 * @description Stores optimization process history for real-time visualization
 * 
 * Captures L-BFGS iterations, MINCO cost evolution, and other optimization metrics
 * that can be displayed in the UI during simulation.
 */

export interface LBFGSIterationData {
    iteration: number;
    cost: number;
    gradientNorm: number;
    step: number;
    time: number;
}

export interface MINCOOptimizationData {
    timestamp: number;
    numPieces: number;
    durations: number[];
    totalTime: number;
    // Cost components
    positionCost: number;
    velocityCost: number;
    accelerationCost: number;
    totalCost: number;
}

export interface CorridorGenerationData {
    segmentIndex: number;
    firiIterations: number;
    ellipsoidVolume: number;
    constraintCount: number;
    time: number;
}

export interface OptimizationHistory {
    robotId: string;
    timestamp: number;

    // L-BFGS optimization history
    lbfgsHistory: LBFGSIterationData[];

    // MINCO trajectory optimization
    mincoData: MINCOOptimizationData | null;

    // Corridor generation
    corridorHistory: CorridorGenerationData[];

    // Summary statistics
    totalPlanningTime: number;
    aStarIterations: number;
    pathLength: number;
    waypointCount: number;
}

// Global cache for optimization history
const optimizationHistoryCache = new Map<string, OptimizationHistory>();

/**
 * Initialize a new optimization history for a robot
 */
export function initOptimizationHistory(robotId: string): OptimizationHistory {
    const history: OptimizationHistory = {
        robotId,
        timestamp: Date.now(),
        lbfgsHistory: [],
        mincoData: null,
        corridorHistory: [],
        totalPlanningTime: 0,
        aStarIterations: 0,
        pathLength: 0,
        waypointCount: 0
    };
    optimizationHistoryCache.set(robotId, history);
    return history;
}

/**
 * Get optimization history for a robot
 */
export function getOptimizationHistory(robotId: string): OptimizationHistory | null {
    return optimizationHistoryCache.get(robotId) || null;
}

/**
 * Record an L-BFGS iteration
 */
export function recordLBFGSIteration(
    robotId: string,
    iteration: number,
    cost: number,
    gradientNorm: number,
    step: number
): void {
    let history = optimizationHistoryCache.get(robotId);
    if (!history) {
        history = initOptimizationHistory(robotId);
    }

    history.lbfgsHistory.push({
        iteration,
        cost,
        gradientNorm,
        step,
        time: Date.now()
    });

    // Limit history size to avoid memory issues
    if (history.lbfgsHistory.length > 100) {
        history.lbfgsHistory = history.lbfgsHistory.slice(-100);
    }
}

/**
 * Record MINCO optimization result
 */
export function recordMINCOData(
    robotId: string,
    numPieces: number,
    durations: number[],
    totalTime: number,
    costs: { position: number; velocity: number; acceleration: number; total: number }
): void {
    let history = optimizationHistoryCache.get(robotId);
    if (!history) {
        history = initOptimizationHistory(robotId);
    }

    history.mincoData = {
        timestamp: Date.now(),
        numPieces,
        durations,
        totalTime,
        positionCost: costs.position,
        velocityCost: costs.velocity,
        accelerationCost: costs.acceleration,
        totalCost: costs.total
    };
}

/**
 * Record corridor generation for a segment
 */
export function recordCorridorGeneration(
    robotId: string,
    segmentIndex: number,
    firiIterations: number,
    ellipsoidVolume: number,
    constraintCount: number
): void {
    let history = optimizationHistoryCache.get(robotId);
    if (!history) {
        history = initOptimizationHistory(robotId);
    }

    history.corridorHistory.push({
        segmentIndex,
        firiIterations,
        ellipsoidVolume,
        constraintCount,
        time: Date.now()
    });
}

/**
 * Record planning summary statistics
 */
export function recordPlanningSummary(
    robotId: string,
    totalPlanningTime: number,
    aStarIterations: number,
    pathLength: number,
    waypointCount: number
): void {
    let history = optimizationHistoryCache.get(robotId);
    if (!history) {
        history = initOptimizationHistory(robotId);
    }

    history.totalPlanningTime = totalPlanningTime;
    history.aStarIterations = aStarIterations;
    history.pathLength = pathLength;
    history.waypointCount = waypointCount;
}

/**
 * Clear optimization history for a robot
 */
export function clearOptimizationHistory(robotId?: string): void {
    if (robotId) {
        optimizationHistoryCache.delete(robotId);
    } else {
        optimizationHistoryCache.clear();
    }
}

/**
 * Get all robot IDs with optimization history
 */
export function getOptimizationRobotIds(): string[] {
    return Array.from(optimizationHistoryCache.keys());
}
