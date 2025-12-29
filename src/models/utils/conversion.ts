/**
 * @module utils/conversion
 * @description Unit conversion utility functions
 */

/**
 * Convert linear value to dB (decibels)
 * 
 * @param linear - Linear value (must be > 0)
 * @returns dB value
 * 
 * @example
 * ```typescript
 * linearToDb(10);   // returns 10
 * linearToDb(100);  // returns 20
 * linearToDb(0.5);  // returns -3.01
 * ```
 */
export function linearToDb(linear: number): number {
    if (linear <= 0) {
        throw new Error('Linear value must be positive');
    }
    return 10 * Math.log10(linear);
}

/**
 * Convert dB (decibels) to linear value
 * 
 * @param db - dB value
 * @returns Linear value
 * 
 * @example
 * ```typescript
 * dbToLinear(10);   // returns 10
 * dbToLinear(20);   // returns 100
 * dbToLinear(-3);   // returns ~0.5
 * ```
 */
export function dbToLinear(db: number): number {
    return Math.pow(10, db / 10);
}

/**
 * Convert power (Watts) to dBm
 * 
 * @param watts - Power value in Watts
 * @returns dBm value
 * 
 * @example
 * ```typescript
 * wattsToDbm(1);     // returns 30 dBm
 * wattsToDbm(0.001); // returns 0 dBm
 * ```
 */
export function wattsToDbm(watts: number): number {
    if (watts <= 0) {
        throw new Error('Power value must be positive');
    }
    return 10 * Math.log10(watts * 1000);
}

/**
 * Convert dBm to power (Watts)
 * 
 * @param dbm - dBm value
 * @returns Power value in Watts
 * 
 * @example
 * ```typescript
 * dbmToWatts(30);  // returns 1 W
 * dbmToWatts(0);   // returns 0.001 W (1 mW)
 * ```
 */
export function dbmToWatts(dbm: number): number {
    return Math.pow(10, dbm / 10) / 1000;
}

/**
 * Convert frequency to wavelength
 * 
 * @param frequency - Frequency in Hz
 * @returns Wavelength in meters
 * 
 * @example
 * ```typescript
 * frequencyToWavelength(3e9);  // 3 GHz -> 0.1 m
 * frequencyToWavelength(28e9); // 28 GHz -> ~0.0107 m
 * ```
 */
export function frequencyToWavelength(frequency: number): number {
    const SPEED_OF_LIGHT = 299792458; // m/s
    return SPEED_OF_LIGHT / frequency;
}

/**
 * Convert wavelength to frequency
 * 
 * @param wavelength - Wavelength in meters
 * @returns Frequency in Hz
 */
export function wavelengthToFrequency(wavelength: number): number {
    const SPEED_OF_LIGHT = 299792458; // m/s
    return SPEED_OF_LIGHT / wavelength;
}

