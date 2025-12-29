/**
 * @module tasks/u2u-mcs/config
 * @description U2U-MCS Task Configuration
 */

// ==================== MCS Table Definitions ====================

/**
 * MCS Table entry
 */
export interface McsEntry {
    /** MCS index */
    index: number;
    /** Modulation scheme */
    modulation: 'QPSK' | '16QAM' | '64QAM' | '256QAM';
    /** Code rate (x/1024) */
    codeRate: number;
    /** Spectral efficiency (bps/Hz) */
    spectralEfficiency: number;
    /** Minimum required SINR (dB) for target BLER */
    requiredSinrDb: number;
}

/**
 * 
 * SINR thresholds are from uavlink project's link-level simulation (ns-3)
 * for BLER target = 0.1, 
 * 
 * [1] J. Yu and H. Jiang, “ns3-uavlink: AI-Driven Dynamic MCS Scheduling for U2U Sidelink Communication,” IEEE VTC.
 */
export const MCS_TABLE_NR1: McsEntry[] = [
    { index: 0, modulation: 'QPSK', codeRate: 120, spectralEfficiency: 0.2344, requiredSinrDb: -6.02 },
    { index: 1, modulation: 'QPSK', codeRate: 193, spectralEfficiency: 0.3770, requiredSinrDb: -4.14 },
    { index: 2, modulation: 'QPSK', codeRate: 308, spectralEfficiency: 0.6016, requiredSinrDb: -2.05 },
    { index: 3, modulation: 'QPSK', codeRate: 449, spectralEfficiency: 0.8770, requiredSinrDb: -0.03 },
    { index: 4, modulation: 'QPSK', codeRate: 602, spectralEfficiency: 1.1758, requiredSinrDb: 1.99 },
    { index: 5, modulation: '16QAM', codeRate: 378, spectralEfficiency: 1.4766, requiredSinrDb: 7.03 },
    { index: 6, modulation: '16QAM', codeRate: 434, spectralEfficiency: 1.6953, requiredSinrDb: 9.93 },
    { index: 7, modulation: '16QAM', codeRate: 490, spectralEfficiency: 1.9141, requiredSinrDb: 11.01 },
    { index: 8, modulation: '16QAM', codeRate: 553, spectralEfficiency: 2.1602, requiredSinrDb: 11.95 },
    { index: 9, modulation: '16QAM', codeRate: 616, spectralEfficiency: 2.4063, requiredSinrDb: 12.09 },
    { index: 10, modulation: '64QAM', codeRate: 438, spectralEfficiency: 2.5703, requiredSinrDb: 13.10 },
    { index: 11, modulation: '64QAM', codeRate: 466, spectralEfficiency: 2.7305, requiredSinrDb: 15.12 },
    { index: 12, modulation: '64QAM', codeRate: 517, spectralEfficiency: 3.0293, requiredSinrDb: 16.07 },
    { index: 13, modulation: '64QAM', codeRate: 567, spectralEfficiency: 3.3223, requiredSinrDb: 19.03 },
    { index: 14, modulation: '64QAM', codeRate: 616, spectralEfficiency: 3.6094, requiredSinrDb: 19.10 },
    { index: 15, modulation: '64QAM', codeRate: 666, spectralEfficiency: 3.9023, requiredSinrDb: 21.06 },
    { index: 16, modulation: '64QAM', codeRate: 719, spectralEfficiency: 4.2129, requiredSinrDb: 21.13 },
    { index: 17, modulation: '64QAM', codeRate: 772, spectralEfficiency: 4.5234, requiredSinrDb: 23.02 },
    { index: 18, modulation: '64QAM', codeRate: 822, spectralEfficiency: 4.8164, requiredSinrDb: 23.96 },
    { index: 19, modulation: '256QAM', codeRate: 873, spectralEfficiency: 5.1152, requiredSinrDb: 24.09 },
    { index: 20, modulation: '256QAM', codeRate: 910, spectralEfficiency: 5.3320, requiredSinrDb: 28.07 },
    { index: 21, modulation: '256QAM', codeRate: 948, spectralEfficiency: 5.5547, requiredSinrDb: 31.10 },
    { index: 22, modulation: '256QAM', codeRate: 1006, spectralEfficiency: 5.8906, requiredSinrDb: 35.14 },
];

// ==================== Configuration ====================

/**
 * Channel model type for SINR generation
 */
export type ChannelModelType = 'rayleigh' | 'rician' | 'awgn';

/**
 * Mobility configuration
 */
export interface MobilityConfig {
    /** Relative speed range [min, max] in m/s */
    relativeSpeedRange: [number, number];
    /** Distance range [min, max] in meters */
    distanceRange: [number, number];
    /** Whether speed changes during episode */
    dynamicSpeed: boolean;
    /** Speed change rate (m/s per step) */
    speedChangeRate?: number;
}

/**
 * Reward weights configuration
 */
export interface RewardWeights {
    /** Weight for throughput component */
    throughput: number;
    /** Penalty weight for BLER exceeding target */
    blerPenalty: number;
    /** Penalty weight for MCS switching (optional) */
    switchPenalty?: number;
}

/**
 * U2U-MCS Task Configuration
 */
export interface U2UMcsConfig {
    /** Random seed for reproducibility */
    seed: number;
    /** Episode length (number of steps) */
    episodeLength: number;
    /** Step duration in milliseconds */
    stepDurationMs: number;
    /** Target BLER for URLLC (e.g., 1e-5) */
    blerTarget: number;
    /** MCS table to use */
    mcsTableId: 'nr-table1' | 'nr-table2';
    /** Input keys to include */
    inputKeys: string[];
    /** Channel model type */
    channelModel: ChannelModelType;
    /** Rician K-factor (dB) if using rician channel */
    ricianKDb?: number;
    /** Mobility configuration */
    mobilityConfig: MobilityConfig;
    /** Reward weights */
    rewardWeights: RewardWeights;
    /** Bandwidth in Hz */
    bandwidthHz: number;
    /** Transmit power in dBm */
    txPowerDbm: number;
    /** Noise figure in dB */
    noiseFigureDb: number;
    /** Number of resource blocks */
    numRBs: number;
    /** Enable power control (optional action dimension) */
    enablePowerControl?: boolean;
}

/**
 * Default configuration
 */
export const DEFAULT_U2U_MCS_CONFIG: U2UMcsConfig = {
    seed: 42,
    episodeLength: 1000,
    stepDurationMs: 1, // 1ms TTI
    blerTarget: 1e-5,
    mcsTableId: 'nr-table1',
    inputKeys: ['sinrDb', 'distance', 'relativeSpeed', 'recentAckRate', 'recentBler', 'lastMcs'],
    channelModel: 'rician',
    ricianKDb: 6,
    mobilityConfig: {
        relativeSpeedRange: [0, 30], // 0-30 m/s
        distanceRange: [10, 200], // 10-200 meters
        dynamicSpeed: true,
        speedChangeRate: 0.5,
    },
    rewardWeights: {
        throughput: 1.0,
        blerPenalty: 100.0,
        switchPenalty: 0.1,
    },
    bandwidthHz: 10e6, // 10 MHz
    txPowerDbm: 23,
    noiseFigureDb: 9,
    numRBs: 50,
    enablePowerControl: false,
};

/**
 * Get MCS table by ID
 */
export function getMcsTable(tableId: 'nr-table1' | 'nr-table2'): McsEntry[] {
    // For now, we only have table 1
    return MCS_TABLE_NR1;
}

/**
 * Get MCS entry by index
 */
export function getMcsEntry(tableId: 'nr-table1' | 'nr-table2', index: number): McsEntry {
    const table = getMcsTable(tableId);
    const clampedIndex = Math.max(0, Math.min(index, table.length - 1));
    return table[clampedIndex];
}

/**
 * Configuration for the specific scenario in the paper
 * 28 GHz, 100 MHz BW, Urban Micro path loss (approximated), customized mobility
 */
export const PAPER_SCENARIO_CONFIG: U2UMcsConfig = {
    ...DEFAULT_U2U_MCS_CONFIG,
    // Paper: 28 GHz
    // Note: path loss model in scenario.ts is simple log-distance,
    // might need adjustment if strict 3GPP channel model is required,
    // but the paper mentions custom ns-3 module. We stick to config params.
    bandwidthHz: 100e6, // 100 MHz
    txPowerDbm: 23, // 23 dBm
    noiseFigureDb: 7, // 7 dB
    episodeLength: 3000, // 30s at 10ms step? Or 30000 at 1ms? Paper says "30-second simulation".
    // Step duration default is 1ms. So 30000 steps.
    // However, for testing speed, we might want fewer steps or larger step size.
    // Let's stick to 1ms TTI -> 30,000 steps.
    stepDurationMs: 1,
    blerTarget: 0.1, // Paper: "fixed BLER constraint ... BLER <= 0.1"

    // Mobility: UAV A (0,0,2) 140km/h; UAV B (80,4,2) 135km/h (one case)
    mobilityConfig: {
        relativeSpeedRange: [30, 40], // ~120-140 km/h approx relative
        distanceRange: [10, 500],
        dynamicSpeed: true,
        speedChangeRate: 0.1,
    },

    // For LSTM input
    inputKeys: ['sinrDb', 'distance', 'relativeSpeed'],

    // Reward weights (Paper: maximize throughput s.t. BLER <= 0.1)
    rewardWeights: {
        throughput: 1.0,
        blerPenalty: 10.0, // High penalty for violating BLER
        switchPenalty: 0.0,
    }
};

// ==================== Task2 Runtime Configuration ====================

/**
 * Runtime configuration for Task2
 * 
 * Separate from scenario config - these are experiment runner settings.
 */
export interface Task2RunConfig {
    /** Python command (e.g., 'python3', 'python') */
    pythonCommand: string;
    /** Path to agent script (relative to task directory) */
    agentScript: string;
    /** Request timeout in milliseconds */
    agentTimeout: number;
    /** Number of episodes to run */
    totalEpisodes: number;
    /** Random seed for experiment */
    seed: number;
    /** Directory for log output */
    logDir: string;
    /** Enable step-level logging */
    enableStepLogs: boolean;
    /** Enable JSONL file output */
    enableJsonlFile: boolean;
    /** Enable console output */
    enableConsoleOutput: boolean;
}

/**
 * Default runtime configuration
 */
export const DEFAULT_TASK2_RUN_CONFIG: Task2RunConfig = {
    pythonCommand: 'python3',
    agentScript: './agents/u2u_mcs_lstm_agent.py',
    agentTimeout: 5000,
    totalEpisodes: 1,
    seed: 42,
    logDir: './logs',
    enableStepLogs: true,
    enableJsonlFile: true,
    enableConsoleOutput: true
};
