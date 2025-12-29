/**
 * @module utils/statistics
 * @description Statistical functions module
 */

/**
 * Compute the probability density function of a Gaussian (normal) distribution
 * 
 * @param x - Variable value
 * @param mean - Mean (default 0)
 * @param std - Standard deviation (default 1)
 * @returns PDF value
 * 
 * @example
 * ```typescript
 * gaussianPdf(0, 0, 1);  // Standard normal distribution value at x=0
 * gaussianPdf(1, 0, 2);  // Normal distribution with mean=0, std=2
 * ```
 */
export function gaussianPdf(x: number, mean: number = 0, std: number = 1): number {
    if (std <= 0) {
        throw new Error('Standard deviation must be positive');
    }
    const exponent = -0.5 * Math.pow((x - mean) / std, 2);
    return (1 / (std * Math.sqrt(2 * Math.PI))) * Math.exp(exponent);
}

/**
 * Generate a random number following Gaussian distribution (Box-Muller transform)
 * 
 * @param mean - Mean (default 0)
 * @param std - Standard deviation (default 1)
 * @returns Gaussian random number
 * 
 * @example
 * ```typescript
 * const noise = gaussianRandom(0, 0.1);  // Generate zero-mean noise with std=0.1
 * ```
 */
export function gaussianRandom(mean: number = 0, std: number = 1): number {
    const u1 = Math.random();
    const u2 = Math.random();
    const z0 = Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
    return mean + z0 * std;
}

/**
 * Generate complex Gaussian random number (real and imaginary parts are i.i.d.)
 * 
 * @param std - Standard deviation of each component (default 1/sqrt(2), so complex magnitude variance is 1)
 * @returns [real part, imaginary part]
 */
export function complexGaussianRandom(std: number = 1 / Math.sqrt(2)): [number, number] {
    return [gaussianRandom(0, std), gaussianRandom(0, std)];
}

/**
 * Compute the mean of an array
 * 
 * @param arr - Numeric array
 * @returns Mean value
 */
export function mean(arr: number[]): number {
    if (arr.length === 0) {
        throw new Error('Array cannot be empty');
    }
    return arr.reduce((sum, val) => sum + val, 0) / arr.length;
}

/**
 * Compute the variance of an array
 * 
 * @param arr - Numeric array
 * @param isSample - Whether to compute sample variance (default true, uses n-1)
 * @returns Variance
 */
export function variance(arr: number[], isSample: boolean = true): number {
    if (arr.length === 0) {
        throw new Error('Array cannot be empty');
    }
    const m = mean(arr);
    const squaredDiffs = arr.map(val => Math.pow(val - m, 2));
    const divisor = isSample ? arr.length - 1 : arr.length;
    return squaredDiffs.reduce((sum, val) => sum + val, 0) / divisor;
}

/**
 * Compute the standard deviation of an array
 * 
 * @param arr - Numeric array
 * @param isSample - Whether to compute sample standard deviation (default true)
 * @returns Standard deviation
 */
export function standardDeviation(arr: number[], isSample: boolean = true): number {
    return Math.sqrt(variance(arr, isSample));
}

/**
 * Q-function (complementary cumulative distribution function)
 * Used for computing bit error rate, etc.
 * 
 * @param x - Input value
 * @returns Q(x) value
 * 
 * @see https://en.wikipedia.org/wiki/Q-function
 */
export function qFunction(x: number): number {
    // Using erfc approximation formula
    return 0.5 * erfc(x / Math.sqrt(2));
}

/**
 * Approximate implementation of complementary error function
 * 
 * @param x - Input value
 * @returns erfc(x) value
 */
export function erfc(x: number): number {
    // Abramowitz and Stegun approximation formula
    const a1 = 0.254829592;
    const a2 = -0.284496736;
    const a3 = 1.421413741;
    const a4 = -1.453152027;
    const a5 = 1.061405429;
    const p = 0.3275911;

    const sign = x < 0 ? -1 : 1;
    x = Math.abs(x);

    const t = 1.0 / (1.0 + p * x);
    const y = 1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * Math.exp(-x * x);

    return sign === 1 ? 1 - y : 1 + y;
}
