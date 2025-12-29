/**
 * OTFS (Orthogonal Time Frequency Space) Modulation
 * 
 * OTFS is a 2D modulation technique designed for high-mobility channels.
 * It operates in the Delay-Doppler domain instead of the Time-Frequency domain.
 * 
 * Key advantages over OFDM:
 * - Robust against high Doppler shifts
 * - Sparse channel representation
 * - Better performance in V2X, HSR (High-Speed Rail), and satellite scenarios
 * 
 * Reference: Hadani et al., "Orthogonal Time Frequency Space Modulation", 2017
 */

import { Complex, fft, ifft } from '../signal-processing'

// ==================== Types ====================

/**
 * OTFS System Configuration Parameters
 */
export interface OTFSParams {
    /** Number of delay bins (equivalent to subcarriers in OFDM) */
    M: number;
    /** Number of Doppler bins (equivalent to time slots in OFDM) */
    N: number;
    /** Subcarrier spacing (Hz) */
    deltaF: number;
    /** Symbol duration (s) */
    T: number;
    /** Carrier frequency (Hz) */
    carrierFreq: number;
    /** Modulation order (e.g., 4 for QPSK, 16 for 16QAM) */
    modOrder: number;
}

/**
 * Single path property in Delay-Doppler domain
 */
export interface DelayDopplerPath {
    /** Path delay in seconds */
    delay: number;
    /** Doppler shift in Hz */
    doppler: number;
    /** Complex path gain */
    gain: Complex;
}

/**
 * Delay-Doppler Channel Model
 */
export interface OTFSChannel {
    /** List of multi-path components */
    paths: DelayDopplerPath[];
    /** Maximum delay in the channel */
    maxDelay: number;
    /** Maximum Doppler shift in the channel */
    maxDoppler: number;
}

// ==================== Constants ====================

const DEFAULT_PARAMS: OTFSParams = {
    M: 32,                 // 32 delay bins
    N: 16,                 // 16 Doppler bins
    deltaF: 15e3,          // 15 kHz subcarrier spacing
    T: 66.67e-6,           // Symbol duration
    carrierFreq: 28e9,     // 28 GHz (mmWave)
    modOrder: 4            // QPSK
}

// ==================== QAM Modulation ====================

/**
 * Generate QAM constellation points
 * @param order - Modulation order
 */
export function generateQAMConstellation(order: number): Complex[] {
    const sqrtM = Math.sqrt(order)
    if (!Number.isInteger(sqrtM)) {
        throw new Error('Modulation order must be a perfect square (4, 16, 64, ...)')
    }

    const constellation: Complex[] = []
    const levels = Array.from({ length: sqrtM }, (_, i) => 2 * i - sqrtM + 1)

    for (const real of levels) {
        for (const imag of levels) {
            constellation.push(new Complex(real, imag))
        }
    }

    // Normalize power to unity
    const avgPower = constellation.reduce((sum, c) => sum + c.magnitude() ** 2, 0) / order
    const normFactor = 1 / Math.sqrt(avgPower)

    return constellation.map(c => c.scale(normFactor))
}

/**
 * QAM modulation: map bits to complex symbols
 */
export function qamModulate(bits: number[], order: number): Complex[] {
    const constellation = generateQAMConstellation(order)
    const bitsPerSymbol = Math.log2(order)
    const symbols: Complex[] = []

    for (let i = 0; i < bits.length; i += bitsPerSymbol) {
        let index = 0
        for (let j = 0; j < bitsPerSymbol; j++) {
            index = (index << 1) | (bits[i + j] || 0)
        }
        symbols.push(constellation[index % constellation.length])
    }

    return symbols
}

/**
 * QAM demodulation: complex symbols to bits (using hard decision)
 */
export function qamDemodulate(symbols: Complex[], order: number): number[] {
    const constellation = generateQAMConstellation(order)
    const bitsPerSymbol = Math.log2(order)
    const bits: number[] = []

    for (const symbol of symbols) {
        // Find nearest constellation point
        let minDist = Infinity
        let minIndex = 0

        for (let i = 0; i < constellation.length; i++) {
            const dist = symbol.subtract(constellation[i]).magnitude()
            if (dist < minDist) {
                minDist = dist
                minIndex = i
            }
        }

        // Convert index back to bit sequence
        for (let j = bitsPerSymbol - 1; j >= 0; j--) {
            bits.push((minIndex >> j) & 1)
        }
    }

    return bits
}

// ==================== OTFS Core Functions ====================

/**
 * Reshape a 1D symbol array into a 2D Delay-Doppler grid (M x N)
 */
export function reshapeToGrid(data: Complex[], M: number, N: number): Complex[][] {
    const grid: Complex[][] = []
    for (let m = 0; m < M; m++) {
        grid[m] = []
        for (let n = 0; n < N; n++) {
            const idx = m * N + n
            grid[m][n] = idx < data.length ? data[idx] : new Complex(0, 0)
        }
    }
    return grid
}

/**
 * Flatten a 2D grid back into a 1D array
 */
export function flattenGrid(grid: Complex[][]): Complex[] {
    return grid.flat()
}

/**
 * Inverse Symplectic Finite Fourier Transform (ISFFT)
 * Transforms from the Delay-Doppler domain to the Time-Frequency domain.
 * 
 * Formula: X_tf[n,m] = (1/sqrt(MN)) * sum_k sum_l x_dd[k,l] * exp(j2π(nk/N - ml/M))
 */
export function isfft2d(ddGrid: Complex[][]): Complex[][] {
    const M = ddGrid.length
    const N = ddGrid[0]?.length || 0
    const tfGrid: Complex[][] = []

    // Step 1: IFFT along the Doppler dimension (columns -> N)
    const temp: Complex[][] = []
    for (let m = 0; m < M; m++) {
        temp[m] = ifft(ddGrid[m])
    }

    // Step 2: FFT along the Delay dimension (rows -> M)
    for (let n = 0; n < N; n++) {
        const column = temp.map(row => row[n])
        const fftColumn = fft(column)
        if (!tfGrid[0]) {
            for (let m = 0; m < M; m++) tfGrid[m] = []
        }
        for (let m = 0; m < M; m++) {
            tfGrid[m][n] = fftColumn[m]
        }
    }

    // Normalize by sqrt(M*N)
    const norm = 1 / Math.sqrt(M * N)
    for (let m = 0; m < M; m++) {
        for (let n = 0; n < N; n++) {
            tfGrid[m][n] = tfGrid[m][n].scale(norm)
        }
    }

    return tfGrid
}

/**
 * Symplectic Finite Fourier Transform (SFFT)
 * Transforms from the Time-Frequency domain back to the Delay-Doppler domain.
 * 
 * Formula: x_dd[k,l] = (1/sqrt(MN)) * sum_n sum_m X_tf[n,m] * exp(-j2π(nk/N - ml/M))
 */
export function sfft2d(tfGrid: Complex[][]): Complex[][] {
    const M = tfGrid.length
    const N = tfGrid[0]?.length || 0
    const ddGrid: Complex[][] = []

    // Step 1: FFT along the Doppler dimension (columns -> N)
    const temp: Complex[][] = []
    for (let m = 0; m < M; m++) {
        temp[m] = fft(tfGrid[m])
    }

    // Step 2: IFFT along the Delay dimension (rows -> M)
    for (let n = 0; n < N; n++) {
        const column = temp.map(row => row[n])
        const ifftColumn = ifft(column)
        if (!ddGrid[0]) {
            for (let m = 0; m < M; m++) ddGrid[m] = []
        }
        for (let m = 0; m < M; m++) {
            ddGrid[m][n] = ifftColumn[m]
        }
    }

    // Normalize by sqrt(M*N)
    const norm = 1 / Math.sqrt(M * N)
    for (let m = 0; m < M; m++) {
        for (let n = 0; n < N; n++) {
            ddGrid[m][n] = ddGrid[m][n].scale(norm)
        }
    }

    return ddGrid
}

/**
 * Heisenberg Transform: Time-Frequency grid to time-domain signal.
 * Converts TF grid to a transmit signal using an OFDM-like basis.
 */
export function heisenbergTransform(tfGrid: Complex[][]): Complex[] {
    const M = tfGrid.length
    const N = tfGrid[0]?.length || 0
    const signal: Complex[] = []

    // Each column in TF grid becomes one period of the time-domain signal
    for (let n = 0; n < N; n++) {
        const subcarriers = tfGrid.map(row => row[n])
        const timeSamples = ifft(subcarriers)
        signal.push(...timeSamples)
    }

    return signal
}

/**
 * Wigner Transform: Time-domain signal to Time-Frequency grid.
 * This is the inverse of the Heisenberg Transform.
 */
export function wignerTransform(signal: Complex[], M: number, N: number): Complex[][] {
    const tfGrid: Complex[][] = Array.from({ length: M }, () => [])

    for (let n = 0; n < N; n++) {
        const start = n * M
        const timeSamples = signal.slice(start, start + M)

        // Zero-padding if signal is truncated
        while (timeSamples.length < M) {
            timeSamples.push(new Complex(0, 0))
        }

        const subcarriers = fft(timeSamples)
        for (let m = 0; m < M; m++) {
            tfGrid[m][n] = subcarriers[m]
        }
    }

    return tfGrid
}

// ==================== OTFS Modulator/Demodulator ====================

/**
 * OTFS Modulation Pipeline: Data bits to transmit signal
 */
export function otfsModulate(bits: number[], params: OTFSParams = DEFAULT_PARAMS): Complex[] {
    // Step 1: Map bits to QAM symbols
    const symbols = qamModulate(bits, params.modOrder)

    // Step 2: Map symbols to Delay-Doppler grid
    const ddGrid = reshapeToGrid(symbols, params.M, params.N)

    // Step 3: Transform to Time-Frequency domain (ISFFT)
    const tfGrid = isfft2d(ddGrid)

    // Step 4: Transform to Time domain (Heisenberg)
    const txSignal = heisenbergTransform(tfGrid)

    return txSignal
}

/**
 * OTFS Demodulation Pipeline: Received signal to data bits
 */
export function otfsDemodulate(
    rxSignal: Complex[],
    params: OTFSParams = DEFAULT_PARAMS
): number[] {
    // Step 1: Recover Time-Frequency grid (Wigner)
    const tfGrid = wignerTransform(rxSignal, params.M, params.N)

    // Step 2: Recover Delay-Doppler grid (SFFT)
    const ddGrid = sfft2d(tfGrid)

    // Step 3: Extract symbols from grid
    const symbols = flattenGrid(ddGrid)

    // Step 4: Recover bits using QAM demodulation
    const bits = qamDemodulate(symbols, params.modOrder)

    return bits
}

// ==================== Channel Model ====================

/**
 * Generate a statistical Delay-Doppler channel based on mobility parameters
 */
export function generateDDChannel(
    velocityKmh: number,  // Mobility in km/h
    carrierFreqHz: number, // Carrier frequency
    numPaths: number = 4
): OTFSChannel {
    const c = 3e8 // Speed of light
    const v = velocityKmh / 3.6 // Convert to m/s
    const maxDoppler = carrierFreqHz * v / c
    const maxDelay = 2e-6 // 2 microseconds typical for wireless channels

    const paths: DelayDopplerPath[] = []

    // Line-of-Sight (LOS) path
    paths.push({
        delay: 0,
        doppler: maxDoppler,
        gain: new Complex(1, 0)
    })

    // Non-Line-of-Sight (NLOS) paths with random scattering
    for (let i = 1; i < numPaths; i++) {
        const delayFactor = Math.random()
        const dopplerFactor = (Math.random() - 0.5) * 2
        const gainMag = 0.3 + Math.random() * 0.4
        const gainPhase = Math.random() * 2 * Math.PI

        paths.push({
            delay: delayFactor * maxDelay,
            doppler: dopplerFactor * maxDoppler,
            gain: Complex.fromPolar(gainMag, gainPhase)
        })
    }

    return { paths, maxDelay, maxDoppler }
}

/**
 * Apply a multi-path Delay-Doppler channel to the transmit signal
 */
export function applyDDChannel(
    signal: Complex[],
    channel: OTFSChannel,
    sampleRate: number
): Complex[] {
    const output: Complex[] = signal.map(() => new Complex(0, 0))
    const N = signal.length

    for (const path of channel.paths) {
        const delaySamples = Math.round(path.delay * sampleRate)

        for (let n = 0; n < N; n++) {
            const srcIdx = n - delaySamples
            if (srcIdx >= 0 && srcIdx < N) {
                // Apply Doppler shift as a progressive phase rotation in time
                const phase = 2 * Math.PI * path.doppler * n / sampleRate
                const phaseRotation = Complex.fromPolar(1, phase)
                const sample = signal[srcIdx].multiply(path.gain).multiply(phaseRotation)
                output[n] = output[n].add(sample)
            }
        }
    }

    return output
}

/**
 * Add Additive White Gaussian Noise (AWGN) to a complex signal
 */
export function addAWGN(signal: Complex[], snrDb: number): Complex[] {
    // Calculate signal power
    const signalPower = signal.reduce((sum, s) => sum + s.magnitude() ** 2, 0) / signal.length

    // Calculate required noise power based on SNR
    const snrLinear = Math.pow(10, snrDb / 10)
    const noisePower = signalPower / snrLinear
    const noiseStd = Math.sqrt(noisePower / 2) // Divided by 2 for I and Q channels

    // Generate complex Gaussian noise
    return signal.map(s => {
        const noiseReal = gaussianRandom() * noiseStd
        const noiseImag = gaussianRandom() * noiseStd
        return s.add(new Complex(noiseReal, noiseImag))
    })
}

/**
 * Helper: Box-Muller transform for generating standard Gaussian random numbers
 */
function gaussianRandom(): number {
    const u1 = Math.random()
    const u2 = Math.random()
    return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2)
}

// ==================== OFDM Comparison ====================

/**
 * Simplified OFDM modulation for performance comparison
 */
export function ofdmModulate(bits: number[], M: number, modOrder: number): Complex[] {
    const symbols = qamModulate(bits, modOrder)
    const numSymbols = Math.ceil(symbols.length / M)
    const signal: Complex[] = []

    for (let n = 0; n < numSymbols; n++) {
        const subcarriers = symbols.slice(n * M, (n + 1) * M)
        while (subcarriers.length < M) {
            subcarriers.push(new Complex(0, 0))
        }
        const timeSamples = ifft(subcarriers)
        signal.push(...timeSamples)
    }

    return signal
}

/**
 * Simplified OFDM demodulation for performance comparison
 */
export function ofdmDemodulate(signal: Complex[], M: number, modOrder: number): number[] {
    const numSymbols = Math.floor(signal.length / M)
    const symbols: Complex[] = []

    for (let n = 0; n < numSymbols; n++) {
        const timeSamples = signal.slice(n * M, (n + 1) * M)
        const subcarriers = fft(timeSamples)
        symbols.push(...subcarriers)
    }

    return qamDemodulate(symbols, modOrder)
}

/**
 * Apply equivalent channel effect to OFDM signals.
 * Incorporates Inter-Carrier Interference (ICI) results from frequency-dispersive channels.
 */
export function applyOFDMChannel(
    signal: Complex[],
    channel: OTFSChannel,
    sampleRate: number,
    M: number  // Subcarriers (FFT size)
): Complex[] {
    // Apply standard time-varying delay channel
    const output = applyDDChannel(signal, channel, sampleRate)

    // Simulate Inter-Carrier Interference (ICI) degradation
    // The ICI power is approximated based on maximum Doppler shift
    const numSymbols = Math.floor(signal.length / M)

    for (let sym = 0; sym < numSymbols; sym++) {
        for (let k = 0; k < M; k++) {
            const idx = sym * M + k
            if (idx < output.length) {
                // ICI magnitude is proportional to Doppler relative to subcarrier spacing
                const subcarrierSpacing = sampleRate / M
                const normalizedDoppler = Math.abs(channel.maxDoppler) / subcarrierSpacing
                const iciPhaseNoise = (Math.random() - 0.5) * 2 * Math.PI * normalizedDoppler
                output[idx] = output[idx].multiply(Complex.fromPolar(1, iciPhaseNoise))
            }
        }
    }

    return output
}

// ==================== Performance Metrics ====================

/**
 * Calculate Bit Error Rate (BER) between transmitted and received bitstreams
 */
export function calculateBER(txBits: number[], rxBits: number[]): number {
    let errors = 0
    const length = Math.min(txBits.length, rxBits.length)

    for (let i = 0; i < length; i++) {
        if (txBits[i] !== rxBits[i]) errors++
    }

    return length > 0 ? errors / length : 0
}

/**
 * Generate a random stream of binary bits
 */
export function randomBits(length: number): number[] {
    return Array.from({ length }, () => Math.random() > 0.5 ? 1 : 0)
}

/**
 * Run a comparative Monte Carlo simulation for OTFS vs OFDM BER performance
 */
export function simulateOTFSvsOFDM(
    velocityKmh: number,
    snrDb: number,
    params: OTFSParams = DEFAULT_PARAMS
): { otfsBER: number; ofdmBER: number } {
    const numBits = params.M * params.N * Math.log2(params.modOrder)
    const txBits = randomBits(numBits)

    // Configure scenario
    const channel = generateDDChannel(velocityKmh, params.carrierFreq)
    const sampleRate = params.M * params.deltaF

    // Simulation Case 1: OTFS
    const otfsTx = otfsModulate(txBits, params)
    let otfsRx = applyDDChannel(otfsTx, channel, sampleRate)
    otfsRx = addAWGN(otfsRx, snrDb)
    const otfsRxBits = otfsDemodulate(otfsRx, params)
    const otfsBER = calculateBER(txBits, otfsRxBits)

    // Simulation Case 2: OFDM
    const ofdmTx = ofdmModulate(txBits, params.M, params.modOrder)
    let ofdmRx = applyOFDMChannel(ofdmTx, channel, sampleRate, params.M)
    ofdmRx = addAWGN(ofdmRx, snrDb)
    const ofdmRxBits = ofdmDemodulate(ofdmRx, params.M, params.modOrder)
    const ofdmBER = calculateBER(txBits, ofdmRxBits)

    return { otfsBER, ofdmBER }
}

// ==================== Visualization Helpers ====================

/**
 * Export complex constellation symbols for graphical plotting
 */
export function getConstellationAfterChannel(
    velocityKmh: number,
    snrDb: number,
    modType: 'OTFS' | 'OFDM',
    params: OTFSParams = DEFAULT_PARAMS
): Complex[] {
    const numBits = params.M * params.N * Math.log2(params.modOrder)
    const txBits = randomBits(numBits)
    const channel = generateDDChannel(velocityKmh, params.carrierFreq)
    const sampleRate = params.M * params.deltaF

    if (modType === 'OTFS') {
        const txSignal = otfsModulate(txBits, params)
        let rxSignal = applyDDChannel(txSignal, channel, sampleRate)
        rxSignal = addAWGN(rxSignal, snrDb)

        // Project symbols back to Delay-Doppler domain for constellation visualization
        const tfGrid = wignerTransform(rxSignal, params.M, params.N)
        const ddGrid = sfft2d(tfGrid)
        return flattenGrid(ddGrid)
    } else {
        const txSignal = ofdmModulate(txBits, params.M, params.modOrder)
        let rxSignal = applyOFDMChannel(txSignal, channel, sampleRate, params.M)
        rxSignal = addAWGN(rxSignal, snrDb)

        // Project symbols back to Time-Frequency domain (subcarriers)
        const numSymbols = Math.floor(rxSignal.length / params.M)
        const symbols: Complex[] = []
        for (let n = 0; n < numSymbols; n++) {
            const timeSamples = rxSignal.slice(n * params.M, (n + 1) * params.M)
            symbols.push(...fft(timeSamples))
        }
        return symbols
    }
}

/**
 * Map a statistical channel to a 2D matrix for Delay-Doppler heatmap visualization
 */
export function getDDChannelMatrix(
    channel: OTFSChannel,
    M: number,
    N: number,
    params: OTFSParams
): number[][] {
    const matrix: number[][] = Array.from({ length: M }, () => Array(N).fill(0))

    const delayResolution = 1 / (M * params.deltaF)
    const dopplerResolution = params.deltaF / N

    for (const path of channel.paths) {
        const delayBin = Math.round(path.delay / delayResolution) % M
        const dopplerBin = Math.round((path.doppler / dopplerResolution) + N / 2) % N

        if (delayBin >= 0 && delayBin < M && dopplerBin >= 0 && dopplerBin < N) {
            matrix[delayBin][dopplerBin] = path.gain.magnitude()
        }
    }

    return matrix
}

