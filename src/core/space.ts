/**
 * @module core/space
 * @description Generic space definitions for input/output data structures
 *
 * Provides unified space types as the contract between protocol layer and task layer.
 * Spaces define the structure of data exchanged in iterative decision systems.
 * Includes schema hash computation for reproducibility verification.
 */

// ==================== Browser-compatible Hash ====================

/**
 * Simple hash function that works in both browser and Node.js
 */
function simpleHash(str: string): string {
    let hash = 5381;
    for (let i = 0; i < str.length; i++) {
        hash = ((hash << 5) + hash) ^ str.charCodeAt(i);
    }
    return (hash >>> 0).toString(16).padStart(8, '0');
}

function browserHash(data: string): string {
    const h1 = simpleHash(data);
    const h2 = simpleHash(data + h1);
    const h3 = simpleHash(h1 + data);
    const h4 = simpleHash(h2 + h3);
    return h1 + h2 + h3 + h4;
}

// ==================== Space Types ====================

/**
 * Discrete space - finite set of possible values [0, n)
 */
export interface DiscreteSpace {
    readonly kind: 'discrete';
    /** Number of discrete values */
    readonly n: number;
    /** Optional labels for each value */
    readonly labels?: readonly string[];
}

/**
 * Box space - n-dimensional continuous space with bounds
 */
export interface BoxSpace {
    readonly kind: 'box';
    /** Shape of the space (e.g., [3] for 3D vector) */
    readonly shape: readonly number[];
    /** Lower bound (scalar or per-dimension) */
    readonly low: number | readonly number[];
    /** Upper bound (scalar or per-dimension) */
    readonly high: number | readonly number[];
    /** Optional dtype hint */
    readonly dtype?: 'float32' | 'float64';
}

/**
 * MultiDiscrete space - multiple discrete spaces combined
 */
export interface MultiDiscreteSpace {
    readonly kind: 'multiDiscrete';
    /** Number of values for each dimension */
    readonly nvec: readonly number[];
    /** Optional labels for each dimension's values */
    readonly labels?: readonly (readonly string[])[];
}

/**
 * Dict space - dictionary of named spaces
 */
export interface DictSpace {
    readonly kind: 'dict';
    /** Named sub-spaces */
    readonly spaces: Readonly<Record<string, Space>>;
}

/**
 * Tuple space - ordered sequence of spaces
 */
export interface TupleSpace {
    readonly kind: 'tuple';
    /** Sub-spaces in order */
    readonly spaces: readonly Space[];
}

/**
 * Union type for all AI environment spaces
 */
export type Space =
    | DiscreteSpace
    | BoxSpace
    | MultiDiscreteSpace
    | DictSpace
    | TupleSpace;

export type DiscreteValue = number;
export type BoxValue = number[];
export type MultiDiscreteValue = number[];

// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type DictValue = Record<string, any>;
// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type TupleValue = any[];

// Union type for all space values - use any for deep recursion
// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type SpaceValue = number | number[] | Record<string, any> | any[];

// ==================== Space Factories ====================

/**
 * Create a discrete space
 */
export function discrete(n: number, labels?: string[]): DiscreteSpace {
    return { kind: 'discrete', n, labels };
}

/**
 * Create a box space
 */
export function box(
    shape: number[],
    low: number | number[] = -Infinity,
    high: number | number[] = Infinity,
    dtype: 'float32' | 'float64' = 'float32'
): BoxSpace {
    return { kind: 'box', shape, low, high, dtype };
}

/**
 * Create a multi-discrete space
 */
export function multiDiscrete(nvec: number[], labels?: string[][]): MultiDiscreteSpace {
    return { kind: 'multiDiscrete', nvec, labels };
}

/**
 * Create a dict space
 */
export function dict(spaces: Record<string, Space>): DictSpace {
    return { kind: 'dict', spaces };
}

/**
 * Create a tuple space
 */
export function tuple(spaces: Space[]): TupleSpace {
    return { kind: 'tuple', spaces };
}

// ==================== Space Operations ====================

/**
 * Sample a random value from a space
 * Uses provided random function for reproducibility
 */
export function sample(space: Space, random: () => number = Math.random): SpaceValue {
    switch (space.kind) {
        case 'discrete':
            return Math.floor(random() * space.n);

        case 'box': {
            const size = space.shape.reduce((a, b) => a * b, 1);
            const result: number[] = [];
            for (let i = 0; i < size; i++) {
                const lo = typeof space.low === 'number' ? space.low : space.low[i] ?? -1;
                const hi = typeof space.high === 'number' ? space.high : space.high[i] ?? 1;
                // Handle infinite bounds
                const actualLo = Number.isFinite(lo) ? lo : -1e6;
                const actualHi = Number.isFinite(hi) ? hi : 1e6;
                result.push(actualLo + random() * (actualHi - actualLo));
            }
            return result;
        }

        case 'multiDiscrete':
            return space.nvec.map(n => Math.floor(random() * n));

        case 'dict': {
            const result: DictValue = {};
            for (const [key, subSpace] of Object.entries(space.spaces)) {
                result[key] = sample(subSpace, random);
            }
            return result;
        }

        case 'tuple':
            return space.spaces.map(s => sample(s, random)) as TupleValue;
    }
}

/**
 * Check if a value is contained within a space
 */
export function contains(space: Space, value: unknown): boolean {
    switch (space.kind) {
        case 'discrete':
            return (
                typeof value === 'number' &&
                Number.isInteger(value) &&
                value >= 0 &&
                value < space.n
            );

        case 'box': {
            if (!Array.isArray(value)) return false;
            const flat = value.flat(Infinity) as number[];
            const expectedSize = space.shape.reduce((a, b) => a * b, 1);
            if (flat.length !== expectedSize) return false;

            const low = typeof space.low === 'number' ? space.low : null;
            const high = typeof space.high === 'number' ? space.high : null;
            const lowArr = Array.isArray(space.low) ? space.low : null;
            const highArr = Array.isArray(space.high) ? space.high : null;

            return flat.every((v, i) => {
                const lo = lowArr ? (lowArr[i] ?? -Infinity) : (low ?? -Infinity);
                const hi = highArr ? (highArr[i] ?? Infinity) : (high ?? Infinity);
                return typeof v === 'number' && v >= lo && v <= hi;
            });
        }

        case 'multiDiscrete': {
            if (!Array.isArray(value)) return false;
            if (value.length !== space.nvec.length) return false;
            return value.every(
                (v, i) =>
                    typeof v === 'number' &&
                    Number.isInteger(v) &&
                    v >= 0 &&
                    v < space.nvec[i]
            );
        }

        case 'dict': {
            if (typeof value !== 'object' || value === null) return false;
            const obj = value as Record<string, unknown>;
            return Object.entries(space.spaces).every(([key, subSpace]) =>
                key in obj ? contains(subSpace, obj[key]) : false
            );
        }

        case 'tuple': {
            if (!Array.isArray(value)) return false;
            if (value.length !== space.spaces.length) return false;
            return space.spaces.every((subSpace, i) => contains(subSpace, value[i]));
        }
    }
}

/**
 * Get the total dimension of a space (for flat vector representation)
 */
export function getDimension(space: Space): number {
    switch (space.kind) {
        case 'discrete':
            return 1;

        case 'box':
            return space.shape.reduce((a, b) => a * b, 1);

        case 'multiDiscrete':
            return space.nvec.length;

        case 'dict':
            return Object.values(space.spaces).reduce((sum, s) => sum + getDimension(s), 0);

        case 'tuple':
            return space.spaces.reduce((sum, s) => sum + getDimension(s), 0);
    }
}

/**
 * Serialize a space to a canonical JSON string (for schema hash)
 */
export function serialize(space: Space): string {
    // Use a deterministic JSON stringification
    return JSON.stringify(space, Object.keys(space).sort());
}

/**
 * Compute schema hash for input and output spaces
 * Used for reproducibility verification
 */
export function computeSchemaHash(inputSpace: Space, outputSpace: Space): string {
    const canonical = JSON.stringify({
        input: serialize(inputSpace),
        output: serialize(outputSpace),
    });
    return browserHash(canonical);
}

/**
 * Flatten a space value to a 1D array
 */
export function flatten(space: Space, value: SpaceValue): number[] {
    switch (space.kind) {
        case 'discrete':
            return [value as number];

        case 'box':
            return (value as number[]).flat(Infinity) as number[];

        case 'multiDiscrete':
            return value as number[];

        case 'dict': {
            const dictSpace = space as DictSpace;
            const dictValue = value as DictValue;
            const result: number[] = [];
            for (const key of Object.keys(dictSpace.spaces).sort()) {
                result.push(...flatten(dictSpace.spaces[key], dictValue[key]));
            }
            return result;
        }

        case 'tuple': {
            const tupleSpace = space as TupleSpace;
            const tupleValue = value as TupleValue;
            const result: number[] = [];
            for (let i = 0; i < tupleSpace.spaces.length; i++) {
                result.push(...flatten(tupleSpace.spaces[i], tupleValue[i]));
            }
            return result;
        }
    }
}

/**
 * Unflatten a 1D array to a space value
 */
export function unflatten(space: Space, flat: number[], offset = 0): { value: SpaceValue; consumed: number } {
    switch (space.kind) {
        case 'discrete':
            return { value: flat[offset], consumed: 1 };

        case 'box': {
            const size = space.shape.reduce((a, b) => a * b, 1);
            return { value: flat.slice(offset, offset + size), consumed: size };
        }

        case 'multiDiscrete': {
            const size = space.nvec.length;
            return { value: flat.slice(offset, offset + size), consumed: size };
        }

        case 'dict': {
            const dictSpace = space as DictSpace;
            const result: DictValue = {};
            let consumed = 0;
            for (const key of Object.keys(dictSpace.spaces).sort()) {
                const { value, consumed: c } = unflatten(dictSpace.spaces[key], flat, offset + consumed);
                result[key] = value;
                consumed += c;
            }
            return { value: result, consumed };
        }

        case 'tuple': {
            const tupleSpace = space as TupleSpace;
            const result: TupleValue = [];
            let consumed = 0;
            for (const subSpace of tupleSpace.spaces) {
                const { value, consumed: c } = unflatten(subSpace, flat, offset + consumed);
                result.push(value);
                consumed += c;
            }
            return { value: result, consumed };
        }
    }
}
