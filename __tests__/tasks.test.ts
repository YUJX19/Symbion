/**
 * Tasks Module Tests
 * Tests for ISAC Trajectory and U2U-MCS paper tasks
 */

import { describe, it, expect } from 'vitest';
import {
    // ISAC Trajectory exports
    DEFAULT_ISAC_CONFIG,
    createConfig as createIsacConfig,
    getLosProbability,
    LOS_PROBABILITY_PARAMS,
} from '../src/tasks/isac-trajectory/config';
import {
    createScenario as createIsacScenario,
} from '../src/tasks/isac-trajectory/scenario';
import {
    // U2U-MCS exports
    DEFAULT_U2U_MCS_CONFIG,
    getMcsTable,
    getMcsEntry,
    MCS_TABLE_NR1,
} from '../src/tasks/u2u-mcs/config';
import { isClose } from './test-utils';

// ==================== ISAC Trajectory Config Tests ====================

describe('ISAC Trajectory Task', () => {
    describe('Configuration', () => {
        it('should have valid default configuration', () => {
            expect(DEFAULT_ISAC_CONFIG.seed).toBe(42);
            expect(DEFAULT_ISAC_CONFIG.numWaypoints).toBe(20);
            expect(DEFAULT_ISAC_CONFIG.users.length).toBeGreaterThan(0);
            expect(DEFAULT_ISAC_CONFIG.obstacles.length).toBeGreaterThan(0);
        });

        it('should create config with default values', () => {
            const config = createIsacConfig();
            expect(config.seed).toBe(42);
            expect(config.numWaypoints).toBe(20);
        });

        it('should override specific values', () => {
            const config = createIsacConfig({
                seed: 123,
                numWaypoints: 30,
            });
            expect(config.seed).toBe(123);
            expect(config.numWaypoints).toBe(30);
            // Other values should remain default
            expect(config.waypointDurationS).toBe(1.0);
        });

        it('should deeply merge nested config', () => {
            const config = createIsacConfig({
                kinematics: { maxSpeed: 30 },
                comm: { txPowerDbm: 40 },
            });
            // Overridden values
            expect(config.kinematics.maxSpeed).toBe(30);
            expect(config.comm.txPowerDbm).toBe(40);
            // Default values should still exist
            expect(config.kinematics.maxAcceleration).toBe(5);
            expect(config.comm.bandwidthHz).toBe(20e6);
        });

        it('should have valid kinematics constraints', () => {
            const config = createIsacConfig();
            expect(config.kinematics.maxSpeed).toBeGreaterThan(0);
            expect(config.kinematics.maxAcceleration).toBeGreaterThan(0);
            expect(config.kinematics.minAltitude).toBeLessThan(config.kinematics.maxAltitude);
        });

        it('should have valid communication parameters', () => {
            const config = createIsacConfig();
            expect(config.comm.carrierFrequencyHz).toBeGreaterThan(0);
            expect(config.comm.bandwidthHz).toBeGreaterThan(0);
            expect(config.comm.txPowerDbm).toBeGreaterThan(0);
        });

        it('should have URLLC and eMBB users', () => {
            const config = createIsacConfig();
            const urllcUsers = config.users.filter(u => u.isUrllc);
            const embbUsers = config.users.filter(u => !u.isUrllc);
            expect(urllcUsers.length).toBeGreaterThan(0);
            expect(embbUsers.length).toBeGreaterThan(0);
        });
    });

    describe('LoS Probability Model', () => {
        it('should have valid model parameters', () => {
            expect(LOS_PROBABILITY_PARAMS.urban).toBeDefined();
            expect(LOS_PROBABILITY_PARAMS.suburban).toBeDefined();
            expect(LOS_PROBABILITY_PARAMS.rural).toBeDefined();
        });

        it('should return values in [0, 1]', () => {
            for (const model of ['urban', 'suburban', 'rural'] as const) {
                for (const angle of [0, 30, 45, 60, 90]) {
                    const p = getLosProbability(angle, model);
                    expect(p).toBeGreaterThanOrEqual(0);
                    expect(p).toBeLessThanOrEqual(1);
                }
            }
        });

        it('should increase with elevation angle', () => {
            for (const model of ['urban', 'suburban', 'rural'] as const) {
                const p0 = getLosProbability(0, model);
                const p45 = getLosProbability(45, model);
                const p90 = getLosProbability(90, model);
                expect(p45).toBeGreaterThan(p0);
                // At 90 degrees, probability may reach 1, so use >=
                expect(p90).toBeGreaterThanOrEqual(p45);
            }
        });

        it('should be higher in rural than urban for same angle', () => {
            const angle = 30;
            const pUrban = getLosProbability(angle, 'urban');
            const pRural = getLosProbability(angle, 'rural');
            expect(pRural).toBeGreaterThan(pUrban);
        });

        it('should handle edge cases', () => {
            // Negative angles should be clamped
            expect(getLosProbability(-10, 'urban')).toBeGreaterThanOrEqual(0);
            // Angles > 90 should be clamped
            expect(getLosProbability(100, 'urban')).toBeLessThanOrEqual(1);
        });
    });

    describe('Scenario Generation', () => {
        it('should create scenario from config', () => {
            const config = createIsacConfig({ seed: 42 });
            const scenario = createIsacScenario(config);
            expect(scenario).toBeDefined();
        });

        it('should have consistent position getter', () => {
            const config = createIsacConfig({ seed: 42 });
            const scenario = createIsacScenario(config);
            const pos = scenario.getCurrentPosition();
            expect(pos.x).toBeDefined();
            expect(pos.y).toBeDefined();
            expect(pos.z).toBeDefined();
        });

        it('should have step counter', () => {
            const config = createIsacConfig({ seed: 42 });
            const scenario = createIsacScenario(config);
            expect(scenario.getCurrentStep()).toBe(0);
        });

        it('should generate state', () => {
            const config = createIsacConfig({ seed: 42 });
            const scenario = createIsacScenario(config);
            const state = scenario.generateState();
            expect(state).toBeDefined();
            expect(state.step).toBe(0);
            expect(state.uav).toBeDefined();
            expect(state.userChannels).toBeDefined();
        });

        it('should reset correctly', () => {
            const config = createIsacConfig({ seed: 42 });
            const scenario = createIsacScenario(config);
            scenario.generateState();
            scenario.reset();
            expect(scenario.getCurrentStep()).toBe(0);
        });
    });
});

// ==================== U2U-MCS Task Tests ====================

describe('U2U-MCS Task', () => {
    describe('Configuration', () => {
        it('should have valid default configuration', () => {
            expect(DEFAULT_U2U_MCS_CONFIG.seed).toBe(42);
            expect(DEFAULT_U2U_MCS_CONFIG.episodeLength).toBeGreaterThan(0);
        });

        it('should have valid BLER target', () => {
            expect(DEFAULT_U2U_MCS_CONFIG.blerTarget).toBeLessThan(1);
            expect(DEFAULT_U2U_MCS_CONFIG.blerTarget).toBeGreaterThan(0);
        });

        it('should have valid channel parameters', () => {
            expect(DEFAULT_U2U_MCS_CONFIG.bandwidthHz).toBeGreaterThan(0);
            expect(DEFAULT_U2U_MCS_CONFIG.txPowerDbm).toBeGreaterThan(0);
            expect(DEFAULT_U2U_MCS_CONFIG.numRBs).toBeGreaterThan(0);
        });

        it('should have valid mobility configuration', () => {
            const mobility = DEFAULT_U2U_MCS_CONFIG.mobilityConfig;
            expect(mobility.relativeSpeedRange[0]).toBeLessThanOrEqual(mobility.relativeSpeedRange[1]);
            expect(mobility.distanceRange[0]).toBeLessThanOrEqual(mobility.distanceRange[1]);
        });

        it('should have valid reward weights', () => {
            const weights = DEFAULT_U2U_MCS_CONFIG.rewardWeights;
            expect(weights.throughput).toBeGreaterThan(0);
            expect(weights.blerPenalty).toBeGreaterThan(0);
        });
    });

    describe('MCS Table', () => {
        it('should have valid MCS entries', () => {
            expect(MCS_TABLE_NR1.length).toBeGreaterThan(0);
            for (const mcs of MCS_TABLE_NR1) {
                expect(mcs.index).toBeGreaterThanOrEqual(0);
                expect(mcs.modulation).toBeDefined();
                expect(mcs.codeRate).toBeGreaterThan(0);
                expect(mcs.spectralEfficiency).toBeGreaterThan(0);
            }
        });

        it('should get MCS table by ID', () => {
            const table = getMcsTable('nr-table1');
            expect(table.length).toBeGreaterThan(0);
            expect(table[0].index).toBe(0);
        });

        it('should get MCS entry by index', () => {
            const entry = getMcsEntry('nr-table1', 5);
            expect(entry.index).toBe(5);
            expect(entry.modulation).toBe('16QAM');
        });

        it('should clamp out-of-range MCS index', () => {
            const entry = getMcsEntry('nr-table1', 100);
            // Should return last valid entry
            expect(entry.index).toBe(MCS_TABLE_NR1.length - 1);
        });

        it('should clamp negative MCS index', () => {
            const entry = getMcsEntry('nr-table1', -5);
            expect(entry.index).toBe(0);
        });

        it('should have increasing spectral efficiency with index', () => {
            const table = getMcsTable('nr-table1');
            for (let i = 1; i < table.length; i++) {
                expect(table[i].spectralEfficiency).toBeGreaterThanOrEqual(table[i - 1].spectralEfficiency);
            }
        });

        it('should have increasing required SINR with index', () => {
            const table = getMcsTable('nr-table1');
            for (let i = 1; i < table.length; i++) {
                expect(table[i].requiredSinrDb).toBeGreaterThanOrEqual(table[i - 1].requiredSinrDb);
            }
        });
    });
});

// ==================== Cross-Task Tests ====================

describe('Task Module Integration', () => {
    it('should export both tasks from index', async () => {
        const tasks = await import('../src/tasks');
        expect(tasks.isacTrajectory).toBeDefined();
        expect(tasks.u2uMcs).toBeDefined();
    });

    it('should have consistent config patterns', () => {
        // Both should have seed
        expect(typeof DEFAULT_ISAC_CONFIG.seed).toBe('number');
        expect(typeof DEFAULT_U2U_MCS_CONFIG.seed).toBe('number');
    });
});

// ==================== Edge Cases ====================

describe('Task Edge Cases', () => {
    describe('ISAC Config Edge Cases', () => {
        it('should handle empty users array', () => {
            const config = createIsacConfig({ users: [] });
            expect(config.users).toEqual([]);
        });

        it('should handle empty obstacles array', () => {
            const config = createIsacConfig({ obstacles: [] });
            expect(config.obstacles).toEqual([]);
        });

        it('should handle extreme altitude values', () => {
            const config = createIsacConfig({
                kinematics: { minAltitude: 1, maxAltitude: 10000 },
            });
            expect(config.kinematics.minAltitude).toBe(1);
            expect(config.kinematics.maxAltitude).toBe(10000);
        });

        it('should handle extreme frequency values', () => {
            const config = createIsacConfig({
                comm: { carrierFrequencyHz: 100e9 }, // 100 GHz (mmWave)
            });
            expect(config.comm.carrierFrequencyHz).toBe(100e9);
        });
    });

    describe('LoS Probability Edge Cases', () => {
        it('should handle zero elevation angle', () => {
            const p = getLosProbability(0, 'urban');
            expect(Number.isFinite(p)).toBe(true);
        });

        it('should handle 90 degree elevation', () => {
            const p = getLosProbability(90, 'urban');
            expect(Number.isFinite(p)).toBe(true);
            expect(p).toBeGreaterThan(0.9); // High LoS probability at zenith
        });
    });

    describe('MCS Edge Cases', () => {
        it('should handle boundary MCS indices', () => {
            const first = getMcsEntry('nr-table1', 0);
            const last = getMcsEntry('nr-table1', MCS_TABLE_NR1.length - 1);
            expect(first.index).toBe(0);
            expect(last.index).toBe(MCS_TABLE_NR1.length - 1);
        });
    });
});
