/**
 * @module modulation/analog
 * @description Analog Modulation Algorithms (AM, FM, PM, DSB-SC, SSB)
 * 
 * ## Supported Modulation Types
 * - AM (Amplitude Modulation with carrier)
 * - FM (Frequency Modulation)
 * - PM (Phase Modulation)
 * - DSB-SC (Double Sideband Suppressed Carrier)
 * - SSB (Single Sideband - USB/LSB)
 * 
 * ## References
 * - Haykin, S. (2001). Communication Systems
 */

/**
 * DSB-SC Modulation
 * 
 * s(t) = m(t) * cos(ωc*t)
 * 
 * @param message - Message signal samples
 * @param carrierFreq - Carrier frequency (Hz)
 * @param sampleRate - Sample rate (Hz)
 * @returns Modulated signal
 */
export function dsbScModulate(
    message: number[],
    carrierFreq: number,
    sampleRate: number
): number[] {
    return message.map((m, i) => {
        const t = i / sampleRate;
        return m * Math.cos(2 * Math.PI * carrierFreq * t);
    });
}

/**
 * DSB-SC Coherent Demodulation
 * 
 * y(t) = LPF{ s(t) * cos(ωc*t) }
 * 
 * @param signal - Received signal
 * @param carrierFreq - Carrier frequency (Hz)
 * @param sampleRate - Sample rate (Hz)
 * @param lpfCutoff - Low-pass filter cutoff frequency (Hz)
 * @returns Demodulated message signal
 */
export function dsbScDemodulate(
    signal: number[],
    carrierFreq: number,
    sampleRate: number,
    lpfCutoff: number
): number[] {
    // Mixing
    const mixed = signal.map((s, i) => {
        const t = i / sampleRate;
        return 2 * s * Math.cos(2 * Math.PI * carrierFreq * t);
    });

    // Simple moving average low-pass filter
    const filterLength = Math.floor(sampleRate / lpfCutoff);
    return movingAverage(mixed, filterLength);
}

/**
 * SSB Modulation (Phase Shift Method)
 * 
 * USB: s(t) = m(t)*cos(ωc*t) - m̂(t)*sin(ωc*t)
 * LSB: s(t) = m(t)*cos(ωc*t) + m̂(t)*sin(ωc*t)
 * 
 * @param message - Message signal
 * @param hilbertMessage - Hilbert transform of the message signal
 * @param carrierFreq - Carrier frequency (Hz)
 * @param sampleRate - Sample rate (Hz)
 * @param sideband - 'USB' or 'LSB'
 * @returns SSB modulated signal
 */
export function ssbModulate(
    message: number[],
    hilbertMessage: number[],
    carrierFreq: number,
    sampleRate: number,
    sideband: 'USB' | 'LSB' = 'USB'
): number[] {
    const sign = sideband === 'USB' ? -1 : 1;

    return message.map((m, i) => {
        const t = i / sampleRate;
        const cosine = Math.cos(2 * Math.PI * carrierFreq * t);
        const sine = Math.sin(2 * Math.PI * carrierFreq * t);
        return m * cosine + sign * hilbertMessage[i] * sine;
    });
}

/**
 * Simplified Hilbert Transform Approximation
 * 
 * For single-frequency sine wave cos(ωt), Hilbert transform is sin(ωt)
 * 
 * @param signal - Input signal
 * @returns Hilbert transform approximation
 */
export function hilbertApprox(signal: number[]): number[] {
    // Using finite difference approximation (simplified implementation)
    // In practice, FFT should be used for accurate Hilbert transform
    const result = new Array(signal.length).fill(0);

    // Simple 90-degree phase shift approximation
    const shift = Math.floor(signal.length / 4);
    for (let i = 0; i < signal.length; i++) {
        const shiftedIndex = (i + shift) % signal.length;
        result[i] = signal[shiftedIndex];
    }

    return result;
}

/**
 * AM Modulation (with Carrier)
 * 
 * s(t) = Ac * [1 + μ*m(t)] * cos(ωc*t)
 * 
 * @param message - Normalized message signal (|m(t)| ≤ 1)
 * @param carrierFreq - Carrier frequency (Hz)
 * @param sampleRate - Sample rate (Hz)
 * @param modulationIndex - Modulation index μ (0 < μ ≤ 1)
 * @returns AM modulated signal
 */
export function amModulate(
    message: number[],
    carrierFreq: number,
    sampleRate: number,
    modulationIndex: number = 1
): number[] {
    return message.map((m, i) => {
        const t = i / sampleRate;
        return (1 + modulationIndex * m) * Math.cos(2 * Math.PI * carrierFreq * t);
    });
}

/**
 * AM Envelope Detection
 * 
 * @param signal - AM signal
 * @returns Demodulated message
 */
export function amEnvelopeDetect(signal: number[]): number[] {
    // Absolute value (Rectification)
    const rectified = signal.map(Math.abs);

    // Low-pass filter (Moving Average)
    const filtered = movingAverage(rectified, 10);

    // Remove DC component
    const mean = filtered.reduce((a, b) => a + b, 0) / filtered.length;
    return filtered.map(x => x - mean);
}

/**
 * FM Modulation (Frequency Modulation)
 * 
 * s(t) = Ac * cos(ωc*t + 2π*kf * ∫m(τ)dτ)
 * 
 * @param message - Message signal samples
 * @param carrierFreq - Carrier frequency (Hz)
 * @param sampleRate - Sample rate (Hz)
 * @param frequencyDeviation - Frequency deviation coefficient kf (Hz per unit)
 * @returns FM modulated signal
 */
export function fmModulate(
    message: number[],
    carrierFreq: number,
    sampleRate: number,
    frequencyDeviation: number = 100
): number[] {
    const output: number[] = [];
    let phaseAccumulator = 0;

    for (let i = 0; i < message.length; i++) {
        const t = i / sampleRate;
        // Integrate message signal (Phase accumulation)
        phaseAccumulator += 2 * Math.PI * frequencyDeviation * message[i] / sampleRate;
        // Generate FM signal
        const phase = 2 * Math.PI * carrierFreq * t + phaseAccumulator;
        output.push(Math.cos(phase));
    }

    return output;
}

/**
 * FM Demodulation (Frequency Discriminator)
 * 
 * @param signal - FM signal
 * @param sampleRate - Sample rate
 * @returns Demodulated message signal
 */
export function fmDemodulate(
    signal: number[],
    sampleRate: number
): number[] {
    // Approximate instantaneous frequency using phase difference
    const output: number[] = [0];

    for (let i = 1; i < signal.length; i++) {
        // Simplified frequency discrimination: using differentiation
        // In practice, use Hilbert transform or PLL
        const diff = signal[i] - signal[i - 1];
        output.push(diff * sampleRate);
    }

    return movingAverage(output, 5);
}

/**
 * PM Modulation (Phase Modulation)
 * 
 * s(t) = Ac * cos(ωc*t + kp * m(t))
 * 
 * @param message - Message signal samples
 * @param carrierFreq - Carrier frequency (Hz)
 * @param sampleRate - Sample rate (Hz)
 * @param phaseDeviation - Phase deviation coefficient kp (rad per unit)
 * @returns PM modulated signal
 */
export function pmModulate(
    message: number[],
    carrierFreq: number,
    sampleRate: number,
    phaseDeviation: number = Math.PI / 2
): number[] {
    return message.map((m, i) => {
        const t = i / sampleRate;
        const phase = 2 * Math.PI * carrierFreq * t + phaseDeviation * m;
        return Math.cos(phase);
    });
}

/**
 * PM Demodulation
 * 
 * @param signal - PM signal
 * @param carrierFreq - Carrier frequency
 * @param sampleRate - Sample rate
 * @returns Demodulated message signal
 */
export function pmDemodulate(
    signal: number[],
    carrierFreq: number,
    sampleRate: number
): number[] {
    // Extract phase using coherent demodulation
    const inPhase: number[] = [];
    const quadrature: number[] = [];

    for (let i = 0; i < signal.length; i++) {
        const t = i / sampleRate;
        inPhase.push(signal[i] * Math.cos(2 * Math.PI * carrierFreq * t));
        quadrature.push(signal[i] * Math.sin(2 * Math.PI * carrierFreq * t));
    }

    // Low-pass filtering
    const iFiltered = movingAverage(inPhase, 10);
    const qFiltered = movingAverage(quadrature, 10);

    // Extract phase
    return iFiltered.map((i, idx) => Math.atan2(-qFiltered[idx], i));
}

/**
 * Simple Moving Average Filter
 */
function movingAverage(signal: number[], windowSize: number): number[] {
    const result = new Array(signal.length).fill(0);
    const halfWindow = Math.floor(windowSize / 2);

    for (let i = 0; i < signal.length; i++) {
        let sum = 0;
        let count = 0;

        for (let j = -halfWindow; j <= halfWindow; j++) {
            const idx = i + j;
            if (idx >= 0 && idx < signal.length) {
                sum += signal[idx];
                count++;
            }
        }

        result[i] = sum / count;
    }

    return result;
}

/**
 * Generate Sine Message Signal
 * 
 * @param freq - Message frequency (Hz)
 * @param sampleRate - Sample rate (Hz)
 * @param duration - Duration (s)
 * @param amplitude - Amplitude
 * @returns Message signal samples
 */
export function generateSineMessage(
    freq: number,
    sampleRate: number,
    duration: number,
    amplitude: number = 1
): number[] {
    const numSamples = Math.floor(sampleRate * duration);
    const signal = new Array(numSamples);

    for (let i = 0; i < numSamples; i++) {
        const t = i / sampleRate;
        signal[i] = amplitude * Math.cos(2 * Math.PI * freq * t);
    }

    return signal;
}

/**
 * Calculate Signal Bandwidth
 * 
 * @param type - Modulation type
 * @param messageBandwidth - Message bandwidth (Hz)
 * @returns Modulated signal bandwidth (Hz)
 */
export function modulationBandwidth(
    type: 'DSB-SC' | 'SSB' | 'AM',
    messageBandwidth: number
): number {
    switch (type) {
        case 'DSB-SC':
        case 'AM':
            return 2 * messageBandwidth;
        case 'SSB':
            return messageBandwidth;
    }
}
