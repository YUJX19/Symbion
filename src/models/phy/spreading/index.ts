/**
 * @module spreading
 * @description Spreading Sequence Generation Algorithms
 * 
 * ## Supported Sequence Types
 * - PN Sequence (m-sequence)
 * - Gold Code
 * - Walsh-Hadamard Code
 * - Kasami Sequence
 * - Frequency Hopping Sequence
 * 
 * ## References
 * - Peterson, R. L. (1995). Introduction to Spread Spectrum Communications
 */

// ==================== PN Sequence (m-sequence) ====================

/**
 * Linear Feedback Shift Register (LFSR) m-sequence generation
 * 
 * @param taps - Feedback tap positions (polynomial coefficients)
 * @param length - Output sequence length
 * @param initialState - Initial state (default all 1s)
 * @returns PN Sequence (represented as ±1)
 */
export function mSequence(taps: number[], length: number, initialState?: number[]): number[] {
    const n = Math.max(...taps)  // Register length
    const state = initialState ? [...initialState] : new Array(n).fill(1)

    const sequence: number[] = []

    for (let i = 0; i < length; i++) {
        // Output last bit
        const output = state[n - 1]
        sequence.push(output === 0 ? -1 : 1)  // 0 -> -1, 1 -> +1

        // Calculate feedback
        let feedback = 0
        for (const tap of taps) {
            feedback ^= state[tap - 1]
        }

        // Shift
        for (let j = n - 1; j > 0; j--) {
            state[j] = state[j - 1]
        }
        state[0] = feedback
    }

    return sequence
}

/**
 * Generate common PN sequences
 * 
 * @param degree - Polynomial degree (3-10)
 * @returns Full period m-sequence
 */
export function pnSequence(degree: number): number[] {
    // Taps for common primitive polynomials
    const primitivePolynomials: Record<number, number[]> = {
        3: [3, 2],           // x³ + x² + 1
        4: [4, 3],           // x⁴ + x³ + 1
        5: [5, 3],           // x⁵ + x³ + 1
        6: [6, 5],           // x⁶ + x⁵ + 1
        7: [7, 6],           // x⁷ + x⁶ + 1
        8: [8, 6, 5, 4],     // x⁸ + x⁶ + x⁵ + x⁴ + 1
        9: [9, 5],           // x⁹ + x⁵ + 1
        10: [10, 7]          // x¹⁰ + x⁷ + 1
    }

    const taps = primitivePolynomials[degree]
    if (!taps) {
        throw new Error(`Unsupported degree: ${degree}. Use 3-10.`)
    }

    const period = (1 << degree) - 1  // 2^n - 1
    return mSequence(taps, period)
}

// ==================== Gold Code ====================

/**
 * Generate Gold Code
 * Gold codes are formed by the modulo-2 sum of two m-sequences, offering balanced cross-correlation properties
 * 
 * @param degree - Polynomial degree
 * @param shift - Phase shift of the second sequence
 * @returns Gold Code Sequence
 */
export function goldSequence(degree: number, shift: number = 0): number[] {
    // Use two different primitive polynomials
    const poly1: Record<number, number[]> = {
        5: [5, 3],
        6: [6, 5],
        7: [7, 6],
        10: [10, 7]
    }

    const poly2: Record<number, number[]> = {
        5: [5, 4, 3, 2],
        6: [6, 5, 2, 1],
        7: [7, 4],
        10: [10, 9, 8, 6, 5, 4, 3, 2]
    }

    if (!poly1[degree] || !poly2[degree]) {
        throw new Error(`Gold sequence not available for degree ${degree}`)
    }

    const period = (1 << degree) - 1
    const seq1 = mSequence(poly1[degree], period)
    const seq2 = mSequence(poly2[degree], period)

    // Modulo-2 sum with shift
    const gold: number[] = []
    for (let i = 0; i < period; i++) {
        const idx2 = (i + shift) % period
        gold.push(seq1[i] * seq2[idx2])  // Since values are ±1, multiplication acts like XOR
    }

    return gold
}

// ==================== Walsh-Hadamard Code ====================

/**
 * Generate Walsh-Hadamard Matrix
 * 
 * @param n - Matrix size (must be a power of 2)
 * @returns n×n Walsh-Hadamard Matrix
 */
export function hadamardMatrix(n: number): number[][] {
    if (n === 1) {
        return [[1]]
    }

    if ((n & (n - 1)) !== 0) {
        throw new Error('n must be a power of 2')
    }

    const half = hadamardMatrix(n / 2)
    const result: number[][] = []

    for (let i = 0; i < n / 2; i++) {
        result.push([...half[i], ...half[i]])
    }
    for (let i = 0; i < n / 2; i++) {
        result.push([...half[i], ...half[i].map(x => -x)])
    }

    return result
}

/**
 * Get Walsh Code Sequence
 * 
 * @param index - Code index (0 ~ n-1)
 * @param n - Code length (must be a power of 2)
 * @returns Walsh Code Sequence
 */
export function walshCode(index: number, n: number): number[] {
    const matrix = hadamardMatrix(n)
    if (index < 0 || index >= n) {
        throw new Error(`Index must be 0 to ${n - 1}`)
    }
    return matrix[index]
}

/**
 * Calculate Walsh Sequence Correlation
 */
export function walshCorrelation(seq1: number[], seq2: number[]): number {
    if (seq1.length !== seq2.length) {
        throw new Error('Sequences must have same length')
    }

    let sum = 0
    for (let i = 0; i < seq1.length; i++) {
        sum += seq1[i] * seq2[i]
    }
    return sum / seq1.length
}

// ==================== Spreading & Despreading ====================

/**
 * Direct Sequence Spread Spectrum (DSSS)
 * 
 * @param data - Data symbols (±1)
 * @param spreadingCode - Spreading code (±1)
 * @returns Spread signal
 */
export function dsssSpread(data: number[], spreadingCode: number[]): number[] {
    const chipRate = spreadingCode.length
    const result: number[] = []

    for (const symbol of data) {
        for (const chip of spreadingCode) {
            result.push(symbol * chip)
        }
    }

    return result
}

/**
 * DSSS Despreading
 * 
 * @param received - Received signal
 * @param spreadingCode - Spreading code
 * @returns Despread symbols
 */
export function dsssDesread(received: number[], spreadingCode: number[]): number[] {
    const chipRate = spreadingCode.length
    const numSymbols = Math.floor(received.length / chipRate)
    const result: number[] = []

    for (let i = 0; i < numSymbols; i++) {
        let sum = 0
        for (let j = 0; j < chipRate; j++) {
            sum += received[i * chipRate + j] * spreadingCode[j]
        }
        result.push(sum > 0 ? 1 : -1)
    }

    return result
}

/**
 * Calculate Processing Gain
 * 
 * @param chipRate - Chip rate
 * @param symbolRate - Symbol rate
 * @returns Processing Gain (dB)
 */
export function processingGain(chipRate: number, symbolRate: number): number {
    return 10 * Math.log10(chipRate / symbolRate)
}

// ==================== Frequency Hopping Sequence ====================

/**
 * PN Sequence based Frequency Hopping Pattern Generation
 * 
 * @param numChannels - Number of available channels
 * @param hopCount - Number of hops
 * @param seed - PN sequence seed
 * @returns Frequency hopping channel index sequence
 */
export function frequencyHoppingPattern(
    numChannels: number,
    hopCount: number,
    seed: number = 1
): number[] {
    // Generate pseudo-random sequence using LFSR
    const bitsPerHop = Math.ceil(Math.log2(numChannels))
    const pnBits = mSequence([7, 6], hopCount * bitsPerHop,
        seed.toString(2).padStart(7, '0').split('').map(Number))

    const pattern: number[] = []

    for (let i = 0; i < hopCount; i++) {
        let channel = 0
        for (let j = 0; j < bitsPerHop; j++) {
            const bit = pnBits[i * bitsPerHop + j] > 0 ? 1 : 0
            channel = (channel << 1) | bit
        }
        pattern.push(channel % numChannels)
    }

    return pattern
}

/**
 * Calculate FHSS Bandwidth Expansion
 * 
 * @param hopBandwidth - Single hop bandwidth (Hz)
 * @param numChannels - Number of channels
 * @returns Total spread bandwidth (Hz)
 */
export function fhssBandwidth(hopBandwidth: number, numChannels: number): number {
    return hopBandwidth * numChannels
}

/**
 * FHSS Jamming Margin
 * 
 * @param totalBandwidth - Total bandwidth
 * @param signalBandwidth - Signal bandwidth
 * @returns Jamming Margin (dB)
 */
export function fhssJammingMargin(totalBandwidth: number, signalBandwidth: number): number {
    return 10 * Math.log10(totalBandwidth / signalBandwidth)
}

// ==================== Kasami Sequence ====================

/**
 * Generate Small Set Kasami Sequence
 * Applicable when n is even
 * 
 * @param degree - Polynomial degree (must be even)
 * @param shift - Phase shift
 * @returns Kasami Sequence
 */
export function kasamiSequence(degree: number, shift: number = 0): number[] {
    if (degree % 2 !== 0) {
        throw new Error('Kasami sequence requires even degree')
    }

    const period = (1 << degree) - 1
    const halfDegree = degree / 2
    const subPeriod = (1 << halfDegree) + 1

    // Generate m-sequence
    const m = pnSequence(degree)

    // Generate decimated sequence (sample every subPeriod)
    const decimated: number[] = []
    for (let i = 0; i < period; i += subPeriod) {
        decimated.push(m[i % period])
    }

    // Extend to full period
    const w: number[] = []
    for (let i = 0; i < period; i++) {
        w.push(decimated[i % decimated.length])
    }

    // Combine
    const kasami: number[] = []
    for (let i = 0; i < period; i++) {
        const idx = (i + shift) % period
        kasami.push(m[i] * w[idx])
    }

    return kasami
}
