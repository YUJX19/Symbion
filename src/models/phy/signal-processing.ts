/**
 * Signal Processing Utilities
 * 
 * Complex number operations, FFT/IFFT, and related functions
 * for digital communication simulations.
 */

// ==================== Complex Number Class ====================

export class Complex {
    constructor(public real: number, public imag: number) { }

    static fromPolar(magnitude: number, phase: number): Complex {
        return new Complex(
            magnitude * Math.cos(phase),
            magnitude * Math.sin(phase)
        )
    }

    static exp(phase: number): Complex {
        return Complex.fromPolar(1, phase)
    }

    add(other: Complex): Complex {
        return new Complex(this.real + other.real, this.imag + other.imag)
    }

    subtract(other: Complex): Complex {
        return new Complex(this.real - other.real, this.imag - other.imag)
    }

    multiply(other: Complex): Complex {
        return new Complex(
            this.real * other.real - this.imag * other.imag,
            this.real * other.imag + this.imag * other.real
        )
    }

    divide(other: Complex): Complex {
        const denom = other.real * other.real + other.imag * other.imag
        return new Complex(
            (this.real * other.real + this.imag * other.imag) / denom,
            (this.imag * other.real - this.real * other.imag) / denom
        )
    }

    scale(factor: number): Complex {
        return new Complex(this.real * factor, this.imag * factor)
    }

    conjugate(): Complex {
        return new Complex(this.real, -this.imag)
    }

    magnitude(): number {
        return Math.sqrt(this.real * this.real + this.imag * this.imag)
    }

    phase(): number {
        return Math.atan2(this.imag, this.real)
    }

    clone(): Complex {
        return new Complex(this.real, this.imag)
    }

    toString(): string {
        const sign = this.imag >= 0 ? '+' : '-'
        return `${this.real.toFixed(4)} ${sign} ${Math.abs(this.imag).toFixed(4)}j`
    }
}

// ==================== FFT/IFFT ====================

/**
 * Cooley-Tukey FFT algorithm (radix-2)
 * Input length must be a power of 2
 */
export function fft(input: Complex[]): Complex[] {
    const N = input.length

    // Handle non-power-of-2 by zero-padding
    const nextPow2 = Math.pow(2, Math.ceil(Math.log2(N)))
    const padded = [...input]
    while (padded.length < nextPow2) {
        padded.push(new Complex(0, 0))
    }

    return fftRadix2(padded)
}

/**
 * Inverse FFT
 */
export function ifft(input: Complex[]): Complex[] {
    const N = input.length

    // Conjugate input
    const conjugated = input.map(c => c.conjugate())

    // Forward FFT
    const transformed = fft(conjugated)

    // Conjugate and scale
    return transformed.map(c => c.conjugate().scale(1 / N))
}

/**
 * Radix-2 FFT implementation
 */
function fftRadix2(x: Complex[]): Complex[] {
    const N = x.length

    if (N <= 1) return x

    if (N === 2) {
        return [
            x[0].add(x[1]),
            x[0].subtract(x[1])
        ]
    }

    // Split into even and odd
    const even = x.filter((_, i) => i % 2 === 0)
    const odd = x.filter((_, i) => i % 2 === 1)

    // Recursive FFT
    const evenFFT = fftRadix2(even)
    const oddFFT = fftRadix2(odd)

    // Combine
    const result: Complex[] = new Array(N)
    const halfN = N / 2

    for (let k = 0; k < halfN; k++) {
        const angle = -2 * Math.PI * k / N
        const twiddle = Complex.fromPolar(1, angle)
        const t = oddFFT[k].multiply(twiddle)

        result[k] = evenFFT[k].add(t)
        result[k + halfN] = evenFFT[k].subtract(t)
    }

    return result
}

// ==================== 2D FFT ====================

/**
 * 2D FFT (row-column algorithm)
 */
export function fft2d(input: Complex[][]): Complex[][] {
    const M = input.length
    const N = input[0]?.length || 0

    // FFT along rows
    let result = input.map(row => fft(row))

    // FFT along columns
    result = transpose(result)
    result = result.map(col => fft(col))
    result = transpose(result)

    return result
}

/**
 * 2D Inverse FFT
 */
export function ifft2d(input: Complex[][]): Complex[][] {
    const M = input.length
    const N = input[0]?.length || 0

    // IFFT along rows
    let result = input.map(row => ifft(row))

    // IFFT along columns
    result = transpose(result)
    result = result.map(col => ifft(col))
    result = transpose(result)

    return result
}

/**
 * Transpose 2D array
 */
export function transpose<T>(matrix: T[][]): T[][] {
    if (matrix.length === 0) return []
    const rows = matrix.length
    const cols = matrix[0].length
    const result: T[][] = []

    for (let j = 0; j < cols; j++) {
        result[j] = []
        for (let i = 0; i < rows; i++) {
            result[j][i] = matrix[i][j]
        }
    }

    return result
}

// ==================== Utility Functions ====================

/**
 * Generate time vector
 */
export function linspace(start: number, end: number, n: number): number[] {
    const step = (end - start) / (n - 1)
    return Array.from({ length: n }, (_, i) => start + i * step)
}

/**
 * Calculate power spectral density
 */
export function psd(signal: Complex[], fs: number): { freq: number[]; power: number[] } {
    const N = signal.length
    const spectrum = fft(signal)
    const power = spectrum.map(c => c.magnitude() ** 2 / N)
    const freq = Array.from({ length: N }, (_, i) => (i - N / 2) * fs / N)

    // Shift to center DC
    const halfN = Math.floor(N / 2)
    const shiftedPower = [...power.slice(halfN), ...power.slice(0, halfN)]

    return { freq, power: shiftedPower }
}

/**
 * Normalize signal to unit power
 */
export function normalizePower(signal: Complex[]): Complex[] {
    const power = signal.reduce((sum, s) => sum + s.magnitude() ** 2, 0) / signal.length
    const scale = 1 / Math.sqrt(power)
    return signal.map(s => s.scale(scale))
}

/**
 * Upsample signal by factor
 */
export function upsample(signal: Complex[], factor: number): Complex[] {
    const result: Complex[] = []
    for (const s of signal) {
        result.push(s)
        for (let i = 1; i < factor; i++) {
            result.push(new Complex(0, 0))
        }
    }
    return result
}

/**
 * Downsample signal by factor
 */
export function downsample(signal: Complex[], factor: number): Complex[] {
    return signal.filter((_, i) => i % factor === 0)
}

/**
 * Convolution of two signals
 */
export function convolve(x: Complex[], h: Complex[]): Complex[] {
    const N = x.length + h.length - 1
    const result: Complex[] = Array.from({ length: N }, () => new Complex(0, 0))

    for (let n = 0; n < N; n++) {
        for (let k = 0; k < h.length; k++) {
            if (n - k >= 0 && n - k < x.length) {
                result[n] = result[n].add(x[n - k].multiply(h[k]))
            }
        }
    }

    return result
}

/**
 * Raised Cosine filter
 */
export function raisedCosineFilter(
    numTaps: number,
    rolloff: number,
    symbolPeriod: number,
    sampleRate: number
): Complex[] {
    const T = symbolPeriod
    const beta = rolloff
    const taps: Complex[] = []
    const center = (numTaps - 1) / 2

    for (let i = 0; i < numTaps; i++) {
        const t = (i - center) / sampleRate
        let value: number

        if (t === 0) {
            value = 1
        } else if (Math.abs(t) === T / (2 * beta)) {
            value = (Math.PI / (4 * T)) * Math.sin(Math.PI / (2 * beta))
        } else {
            const sinc = Math.sin(Math.PI * t / T) / (Math.PI * t / T)
            const cos = Math.cos(Math.PI * beta * t / T)
            const denom = 1 - (2 * beta * t / T) ** 2
            value = sinc * cos / denom
        }

        taps.push(new Complex(value, 0))
    }

    return taps
}
