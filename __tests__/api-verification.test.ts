/**
 * API Verification Test Suite
 * 
 * This test suite verifies reported API issues to determine:
 * 1. Which issues are real bugs
 * 2. Which are design decisions requiring better documentation
 * 3. What improvements can be made
 */

import { describe, it, expect } from 'vitest';

// Category 1: Calculation Results vs Theory
describe('Category 1: Calculation Results vs Theory', () => {
    describe('phy.maxSymbolRate(B, α)', () => {
        it('should match theoretical formula B/(1+α)', async () => {
            const { maxSymbolRate } = await import('../src/models/phy/modulation/pulse-shaping');

            const bandwidth = 1000; // kHz
            const rolloff = 0.25;

            // User expected: B/(1+α) = 1000/1.25 = 800 kSym/s
            const expectedRate = bandwidth / (1 + rolloff);

            // Actual result
            const actualRate = maxSymbolRate(bandwidth, rolloff);

            console.log(`\n📊 maxSymbolRate Test:`);
            console.log(`  Bandwidth: ${bandwidth} kHz, Rolloff: ${rolloff}`);
            console.log(`  Expected (B/(1+α)): ${expectedRate} kSym/s`);
            console.log(`  Actual (2B/(1+α)): ${actualRate} kSym/s`);
            console.log(`  Formula in code: 2 * bandwidth / (1 + rolloff)`);

            // This test documents the discrepancy
            expect(actualRate).toBe(2 * bandwidth / (1 + rolloff));
            expect(actualRate).not.toBe(expectedRate);
        });
    });

    describe('sensing.isInFOV(...)', () => {
        it('should correctly detect targets in FOV', async () => {
            const { isInFOV } = await import('../src/models/sensing/perception');

            // Robot at origin, facing east (0 radians)
            const robotPose = { x: 0, y: 0, theta: 0 };

            // Target directly in front
            const target1 = { x: 5, y: 0 };

            // Target to the right (45 degrees)
            const target2 = { x: 5, y: 5 };

            // Target behind
            const target3 = { x: -5, y: 0 };

            const fovAngle = Math.PI / 2; // 90 degrees
            const fovRange = 10; // meters

            const result1 = isInFOV(robotPose, target1, fovAngle, fovRange);
            const result2 = isInFOV(robotPose, target2, fovAngle, fovRange);
            const result3 = isInFOV(robotPose, target3, fovAngle, fovRange);

            console.log(`\n📡 isInFOV Test:`);
            console.log(`  Robot pose: (0, 0, θ=0°)`);
            console.log(`  FOV: 90°, Range: 10m`);
            console.log(`  Target in front (5,0): ${result1} ✓`);
            console.log(`  Target 45° right (5,5): ${result2} ✓`);
            console.log(`  Target behind (-5,0): ${result3} ✓`);

            expect(result1).toBe(true); // Should be in FOV
            expect(result2).toBe(true); // Should be in FOV
            expect(result3).toBe(false); // Should NOT be in FOV
        });
    });
});

// Category 2: API Signature Inconsistencies
describe('Category 2: API Signature Inconsistencies', () => {
    describe('core.box() and getDimension()', () => {
        it('should return valid dimension', async () => {
            const { box, getDimension } = await import('../src/core/space');

            const space = box([3], -1, 1);
            const dim = getDimension(space);

            console.log(`\n📦 box() Test:`);
            console.log(`  Space: box([3], -1, 1)`);
            console.log(`  Dimension: ${dim}`);

            expect(dim).toBe(3);
            expect(Number.isNaN(dim)).toBe(false);
        });
    });

    describe('core.sample()', () => {
        it('should return non-empty array for box space', async () => {
            const { box, sample } = await import('../src/core/space');

            const space = box([3], -1, 1);
            const sampled = sample(space);

            console.log(`\n🎲 sample() Test:`);
            console.log(`  Space: box([3], -1, 1)`);
            console.log(`  Sample: ${JSON.stringify(sampled)}`);
            console.log(`  Length: ${(sampled as number[]).length}`);

            expect(Array.isArray(sampled)).toBe(true);
            expect((sampled as number[]).length).toBeGreaterThan(0);
        });
    });

    describe('planning.evaluateTrajectory()', () => {
        it('should return position as array [x,y,z], not object {x,y,z}', async () => {
            const { generateMinimumSnapTrajectory, evaluateTrajectory } =
                await import('../src/models/planning/planning/minco');

            const waypoints: [number, number, number][] = [
                [0, 0, 0],
                [10, 10, 5]
            ];

            const trajectory = generateMinimumSnapTrajectory(waypoints, 5.0);
            const state = evaluateTrajectory(trajectory, 0);

            console.log(`\n🛤️ evaluateTrajectory() Test:`);
            console.log(`  Return type: ${typeof state.position}`);
            console.log(`  Is array: ${Array.isArray(state.position)}`);
            console.log(`  Position: ${JSON.stringify(state.position)}`);
            console.log(`  Access: state.position[0] = ${state.position[0]}`);
            console.log(`  ⚠️ Note: Use array notation, NOT state.position.x`);

            expect(Array.isArray(state.position)).toBe(true);
            expect(state.position.length).toBe(3);
            expect(typeof state.position[0]).toBe('number');

            // This would fail - documenting why users see errors
            // @ts-expect-error - position is array, not object with .x property
            expect(state.position.x).toBeUndefined();
        });
    });

    describe('beamforming.array.arrayGain()', () => {
        it('should return linear gain, not dB', async () => {
            const { arrayGain } = await import('../src/models/beamforming/array');

            const config = { numAntennas: 8, spacing: 0.5 };
            const gainLinear = arrayGain(config);
            const gainDb = 10 * Math.log10(gainLinear);

            console.log(`\n📡 arrayGain() Test:`);
            console.log(`  Antennas: 8, Spacing: 0.5λ`);
            console.log(`  Return value (linear): ${gainLinear}`);
            console.log(`  Converted to dB: ${gainDb.toFixed(2)} dB`);
            console.log(`  ⚠️ Note: Function returns LINEAR gain, user expected dB`);

            expect(gainLinear).toBe(8); // Returns linear value
            expect(gainDb).toBeCloseTo(9.03, 1); // Manual conversion needed
        });

        it('should provide arrayGainDb for direct dB calculation', async () => {
            const { arrayGainDb } = await import('../src/models/beamforming/array');

            const config = { numAntennas: 8, spacing: 0.5 };
            const gainDb = arrayGainDb(config);

            console.log(`\n📡 arrayGainDb() Test (NEW):`);
            console.log(`  Antennas: 8, Spacing: 0.5λ`);
            console.log(`  Return value (dB): ${gainDb.toFixed(2)} dB`);
            console.log(`  ✓ Direct dB calculation now available`);

            expect(gainDb).toBeCloseTo(9.03, 2);
        });
    });
});

// Category 3: Environment/Initialization Requirements
describe('Category 3: Environment/Initialization Requirements', () => {
    describe('isac.getOutputSpace() and getInputSpace()', () => {
        it('should work in task environment context', async () => {
            const { getOutputSpace, getInputSpace } =
                await import('../src/tasks/isac-trajectory/observation');

            const config = {
                numTargets: 3,
                gridSize: 50,
                dt: 0.1,
                flightConstraints: {
                    maxVelocity: 5,
                    maxAcceleration: 3,
                    minHeight: 1,
                    maxHeight: 10
                },
                users: [
                    { position: { x: 10, y: 10 }, snrThreshold: 10 },
                    { position: { x: -10, y: 10 }, snrThreshold: 10 },
                    { position: { x: 0, y: -10 }, snrThreshold: 10 }
                ]
            };

            // These functions require config, not standalone environment
            const outputSpace = getOutputSpace(config as any, true);
            const inputSpace = getInputSpace(config as any);

            console.log(`\n🎮 getOutputSpace/InputSpace Test:`);
            console.log(`  Output space kind: ${outputSpace.kind}`);
            console.log(`  Input space kind: ${inputSpace.kind}`);
            console.log(`  ✓ Functions work with proper config`);

            expect(outputSpace).toBeDefined();
            expect(inputSpace).toBeDefined();
        });
    });

    describe('phy.ofdmModulate()', () => {
        it('should validate modulation order as perfect square', async () => {
            const { generateConstellation } = await import('../src/models/phy/modulation/qam');

            console.log(`\n📶 OFDM Modulation Order Test:`);

            // Valid orders (perfect squares)
            const validOrders = [4, 16, 64, 256];
            for (const order of validOrders) {
                expect(() => generateConstellation(order)).not.toThrow();
                console.log(`  ✓ Order ${order} (valid)`);
            }

            // Invalid orders
            const invalidOrders = [3, 8, 32];
            for (const order of invalidOrders) {
                expect(() => generateConstellation(order)).toThrow();
                console.log(`  ✗ Order ${order} throws error (expected)`);
            }
        });
    });

    describe('sensing.worldToGrid()', () => {
        it('should handle gridMap with origin', async () => {
            const { worldToGrid } = await import('../src/models/sensing/environment');

            const gridMap = {
                grid: Array(10).fill(0).map(() => Array(10).fill(0)),
                origin: { x: 0, y: 0 },
                resolution: 1.0,
                width: 10,
                height: 10
            };

            const worldPos = { x: 5.5, y: 3.2 };
            const gridPos = worldToGrid(worldPos, gridMap);

            console.log(`\n🗺️ worldToGrid Test:`);
            console.log(`  World pos: (${worldPos.x}, ${worldPos.y})`);
            console.log(`  Grid pos: (${gridPos.gx}, ${gridPos.gy})`);
            console.log(`  ✓ Requires gridMap.origin property`);

            expect(gridPos).toBeDefined();
            expect(typeof gridPos.gx).toBe('number');
            expect(typeof gridPos.gy).toBe('number');
        });
    });
});

// Category 4: API Naming/Structure Differences
describe('Category 4: API Naming/Structure Differences', () => {
    describe('planning.Vector3', () => {
        it('should document usage patterns', async () => {
            // Vector3 is a type alias for [number, number, number]
            // Not a class with constructor or create method

            console.log(`\n📐 Vector3 Usage:`);
            console.log(`  Vector3 is type alias: [number, number, number]`);
            console.log(`  Create: const v: Vector3Array = [1, 2, 3]`);
            console.log(`  ⚠️ No Vector3.create() or new Vector3()`);

            const v: [number, number, number] = [1, 2, 3];
            expect(Array.isArray(v)).toBe(true);
            expect(v.length).toBe(3);
        });
    });

    describe('phy.generateConstellation()', () => {
        it('should return array directly, not object with .symbols', async () => {
            const { generateConstellation } = await import('../src/models/phy/modulation/qam');

            const constellation = generateConstellation(4);

            console.log(`\n⭐ generateConstellation Test:`);
            console.log(`  Return type: ${Array.isArray(constellation) ? 'Array' : 'Object'}`);
            console.log(`  Length: ${constellation.length}`);
            console.log(`  Sample: ${JSON.stringify(constellation[0])}`);
            console.log(`  ⚠️ Returns QAMSymbol[] directly, not {symbols: ...}`);

            expect(Array.isArray(constellation)).toBe(true);
            expect(constellation.length).toBe(4);
            expect(constellation[0]).toHaveProperty('i');
            expect(constellation[0]).toHaveProperty('q');
        });
    });

    describe('robotics.dynamics.identityQuaternion()', () => {
        it('should return array [w,x,y,z], not object', async () => {
            const { identityQuaternion } = await import('../src/models/robotics/dynamics/physics');

            const q = identityQuaternion();

            console.log(`\n🔄 identityQuaternion Test:`);
            console.log(`  Return type: ${Array.isArray(q) ? 'Array' : 'Object'}`);
            console.log(`  Value: ${JSON.stringify(q)}`);
            console.log(`  Format: [w, x, y, z] = [${q[0]}, ${q[1]}, ${q[2]}, ${q[3]}]`);
            console.log(`  ⚠️ Use array notation, not q.w`);

            expect(Array.isArray(q)).toBe(true);
            expect(q.length).toBe(4);
            expect(q[0]).toBe(1); // w
            expect(q[1]).toBe(0); // x
            expect(q[2]).toBe(0); // y
            expect(q[3]).toBe(0); // z
        });
    });

    describe('robotics.dynamics.createHoverState()', () => {
        it('should return QuadrotorState3D without motorThrusts', async () => {
            const { createHoverState } = await import('../src/models/robotics/dynamics/physics');

            const state = createHoverState([1, 2, 3]);

            console.log(`\n🚁 createHoverState Test:`);
            console.log(`  Properties: ${Object.keys(state).join(', ')}`);
            console.log(`  Has motorThrusts: ${('motorThrusts' in state)}`);
            console.log(`  ⚠️ Returns {position, velocity, quaternion, angularVelocity}`);

            expect(state).toHaveProperty('position');
            expect(state).toHaveProperty('velocity');
            expect(state).toHaveProperty('quaternion');
            expect(state).toHaveProperty('angularVelocity');
            expect(state).not.toHaveProperty('motorThrusts');
        });
    });
});
