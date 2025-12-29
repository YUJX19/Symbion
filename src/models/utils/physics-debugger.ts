/**
 * @module utils/physics-debugger
 * @description Diagnostic utilities for debugging quadrotor physics simulation.
 * 
 * Helps identify coordinate system issues, thrust direction problems,
 * and unstable flight behaviors.
 */

import type { Vector3Array, TrajectoryState } from '../planning/trajectory/types';
import type { Quaternion, ControlCommand } from '../robotics/control/types';
import type { QuadrotorState3D } from '../robotics/dynamics/types';

/**
 * Single log entry for physics debugging
 */
export interface PhysicsDebugEntry {
    timestamp: number;
    // Trajectory reference
    desiredPos: Vector3Array;
    desiredVel: Vector3Array;
    desiredAccel: Vector3Array;
    // Controller output
    thrust: number;
    quaternion: Quaternion;
    // Physics state
    actualPos: Vector3Array;
    actualVel: Vector3Array;
    // Derived metrics
    posError: Vector3Array;
    velError: Vector3Array;
}

/**
 * Physics debugger for tracking simulation behavior
 */
export class PhysicsDebugger {
    private logs: PhysicsDebugEntry[] = [];
    private maxLogs: number = 500;
    private enabled: boolean = true;
    private alertThreshold: number = 50; // meters
    private lastAlertTime: number = 0;

    /**
     * Enable or disable logging
     */
    setEnabled(enabled: boolean): void {
        this.enabled = enabled;
    }

    /**
     * Reset all logs
     */
    reset(): void {
        this.logs = [];
        this.lastAlertTime = 0;
    }

    /**
     * Log a physics step
     */
    log(
        timestamp: number,
        desiredState: TrajectoryState,
        command: ControlCommand,
        actualState: QuadrotorState3D
    ): void {
        if (!this.enabled) return;

        const posError: Vector3Array = [
            desiredState.position[0] - actualState.position[0],
            desiredState.position[1] - actualState.position[1],
            desiredState.position[2] - actualState.position[2]
        ];

        const velError: Vector3Array = [
            desiredState.velocity[0] - actualState.velocity[0],
            desiredState.velocity[1] - actualState.velocity[1],
            desiredState.velocity[2] - actualState.velocity[2]
        ];

        const entry: PhysicsDebugEntry = {
            timestamp,
            desiredPos: [...desiredState.position] as Vector3Array,
            desiredVel: [...desiredState.velocity] as Vector3Array,
            desiredAccel: [...desiredState.acceleration] as Vector3Array,
            thrust: command.thrust,
            quaternion: [...actualState.quaternion] as Quaternion,  // Use actual state quaternion
            actualPos: [...actualState.position] as Vector3Array,
            actualVel: [...actualState.velocity] as Vector3Array,
            posError,
            velError
        };

        this.logs.push(entry);

        // Trim old logs
        if (this.logs.length > this.maxLogs) {
            this.logs.shift();
        }

        // Check for divergence
        this.checkDivergence(entry, timestamp);
    }

    /**
     * Check for position divergence and alert
     */
    private checkDivergence(entry: PhysicsDebugEntry, timestamp: number): void {
        const posErrorMag = Math.sqrt(
            entry.posError[0] ** 2 +
            entry.posError[1] ** 2 +
            entry.posError[2] ** 2
        );

        // Alert at most once per second
        if (posErrorMag > this.alertThreshold && timestamp - this.lastAlertTime > 1.0) {
            this.lastAlertTime = timestamp;
            console.warn(`[PhysicsDebugger] Position divergence detected!`);
            console.warn(`  Position error: ${posErrorMag.toFixed(1)}m`);
            console.warn(`  Actual pos: [${entry.actualPos.map(v => v.toFixed(1)).join(', ')}]`);
            console.warn(`  Desired pos: [${entry.desiredPos.map(v => v.toFixed(1)).join(', ')}]`);
            console.warn(`  Thrust: ${entry.thrust.toFixed(2)}N`);
            console.warn(`  Quaternion: [${entry.quaternion.map(v => v.toFixed(3)).join(', ')}]`);

            // Analyze the issue
            this.diagnoseProblem(entry);
        }
    }

    /**
     * Attempt to diagnose the physics problem
     */
    private diagnoseProblem(entry: PhysicsDebugEntry): void {
        const zActual = entry.actualPos[2];
        const zDesired = entry.desiredPos[2];
        const zError = entry.posError[2];
        const zVel = entry.actualVel[2];

        // Check if it's an altitude issue
        if (Math.abs(zError) > this.alertThreshold * 0.5) {
            console.warn(`[Diagnosis] Z-axis (altitude) issue detected:`);
            console.warn(`  Z actual: ${zActual.toFixed(1)}m, Z desired: ${zDesired.toFixed(1)}m`);
            console.warn(`  Z velocity: ${zVel.toFixed(2)} m/s`);

            if (zActual > zDesired && zVel > 0) {
                console.warn(`  -> Drone accelerating upward when it should descend`);
                console.warn(`  -> Possible cause: thrust direction or gravity compensation error`);
            } else if (zActual < zDesired && zVel < 0) {
                console.warn(`  -> Drone falling when it should climb`);
                console.warn(`  -> Possible cause: insufficient thrust or inverted Z-axis`);
            }
        }

        // Check thrust sanity
        const expectedHoverThrust = 0.98 * 9.81; // mass * gravity
        if (entry.thrust < expectedHoverThrust * 0.3 || entry.thrust > expectedHoverThrust * 5) {
            console.warn(`[Diagnosis] Abnormal thrust: ${entry.thrust.toFixed(2)}N`);
            console.warn(`  Expected hover thrust: ~${expectedHoverThrust.toFixed(2)}N`);
        }

        // Check quaternion sanity (should be near identity for level flight)
        const [w, x, y, z] = entry.quaternion;
        const tiltAngle = 2 * Math.acos(Math.abs(w)) * (180 / Math.PI);
        if (tiltAngle > 45) {
            console.warn(`[Diagnosis] Excessive tilt angle: ${tiltAngle.toFixed(1)}°`);
        }
    }

    /**
     * Get analysis report
     */
    getReport(): string {
        if (this.logs.length === 0) {
            return "No physics logs available";
        }

        const firstLog = this.logs[0];
        const lastLog = this.logs[this.logs.length - 1];

        const posErrorMags = this.logs.map(log =>
            Math.sqrt(log.posError[0] ** 2 + log.posError[1] ** 2 + log.posError[2] ** 2)
        );
        const maxError = Math.max(...posErrorMags);
        const avgError = posErrorMags.reduce((a, b) => a + b, 0) / posErrorMags.length;

        const thrusts = this.logs.map(l => l.thrust);
        const avgThrust = thrusts.reduce((a, b) => a + b, 0) / thrusts.length;
        const maxThrust = Math.max(...thrusts);

        return `
=== Physics Debug Report ===
Time range: ${firstLog.timestamp.toFixed(2)}s - ${lastLog.timestamp.toFixed(2)}s
Total samples: ${this.logs.length}

Position Error:
  Average: ${avgError.toFixed(2)}m
  Maximum: ${maxError.toFixed(2)}m

Thrust:
  Average: ${avgThrust.toFixed(2)}N
  Maximum: ${maxThrust.toFixed(2)}N
  
Start position: [${firstLog.actualPos.map(v => v.toFixed(1)).join(', ')}]
End position: [${lastLog.actualPos.map(v => v.toFixed(1)).join(', ')}]
Total drift: [${(lastLog.actualPos[0] - firstLog.actualPos[0]).toFixed(1)}, ${(lastLog.actualPos[1] - firstLog.actualPos[1]).toFixed(1)}, ${(lastLog.actualPos[2] - firstLog.actualPos[2]).toFixed(1)}]
`;
    }

    /**
     * Get structured report data for UI display
     */
    getReportData(): {
        duration: number;
        samples: number;
        maxError: number;
        avgError: number;
        avgThrust: number;
        maxThrust: number;
        minThrust: number;
        startPos: Vector3Array;
        endPos: Vector3Array;
        totalDrift: Vector3Array;
    } | null {
        if (this.logs.length === 0) {
            return null;
        }

        const firstLog = this.logs[0];
        const lastLog = this.logs[this.logs.length - 1];

        const posErrorMags = this.logs.map(log =>
            Math.sqrt(log.posError[0] ** 2 + log.posError[1] ** 2 + log.posError[2] ** 2)
        );
        const maxError = Math.max(...posErrorMags);
        const avgError = posErrorMags.reduce((a, b) => a + b, 0) / posErrorMags.length;

        const thrusts = this.logs.map(l => l.thrust);
        const avgThrust = thrusts.reduce((a, b) => a + b, 0) / thrusts.length;
        const maxThrust = Math.max(...thrusts);
        const minThrust = Math.min(...thrusts);

        return {
            duration: lastLog.timestamp - firstLog.timestamp,
            samples: this.logs.length,
            maxError,
            avgError,
            avgThrust,
            maxThrust,
            minThrust,
            startPos: firstLog.actualPos,
            endPos: lastLog.actualPos,
            totalDrift: [
                lastLog.actualPos[0] - firstLog.actualPos[0],
                lastLog.actualPos[1] - firstLog.actualPos[1],
                lastLog.actualPos[2] - firstLog.actualPos[2]
            ] as Vector3Array
        };
    }

    /**
     * Get raw logs for detailed analysis
     */
    getLogs(): PhysicsDebugEntry[] {
        return [...this.logs];
    }
}

/**
 * Utility: Extract body Z-axis from quaternion
 * The body Z-axis indicates thrust direction
 */
export function getBodyZAxis(quaternion: Quaternion): Vector3Array {
    const [w, x, y, z] = quaternion;

    // Body Z in world frame: R * [0, 0, 1]
    return [
        2 * (x * z + w * y),
        2 * (y * z - w * x),
        1 - 2 * (x * x + y * y)
    ];
}

/**
 * Utility: Check if thrust vector is roughly aligned with gravity compensation
 */
export function checkThrustAlignment(
    quaternion: Quaternion,
    thrust: number,
    mass: number = 0.98,
    gravity: number = 9.81
): { aligned: boolean; angle: number; expectedThrust: number } {
    const bodyZ = getBodyZAxis(quaternion);

    // At hover, body Z should point straight up: [0, 0, 1]
    const dotWithUp = bodyZ[2];
    const angle = Math.acos(Math.max(-1, Math.min(1, dotWithUp))) * (180 / Math.PI);
    const expectedThrust = mass * gravity / Math.max(0.1, dotWithUp);

    return {
        aligned: angle < 30 && Math.abs(thrust - expectedThrust) < expectedThrust * 0.5,
        angle,
        expectedThrust
    };
}

/**
 * Simple hover test: verify physics can maintain position
 */
export function runHoverTest(
    controller: { update: (s: QuadrotorState3D, d: TrajectoryState, y: number, yr: number, dt: number) => ControlCommand },
    physicsUpdate: (s: QuadrotorState3D, c: ControlCommand, p: unknown, dt: number) => QuadrotorState3D,
    physicsParams: unknown,
    hoverPosition: Vector3Array = [0, 0, 15],
    duration: number = 5.0,
    dt: number = 0.01
): { success: boolean; maxDrift: number; finalPosition: Vector3Array } {
    // Initial state at hover position
    let state: QuadrotorState3D = {
        position: [...hoverPosition] as Vector3Array,
        velocity: [0, 0, 0] as Vector3Array,
        quaternion: [1, 0, 0, 0] as Quaternion,
        angularVelocity: [0, 0, 0] as Vector3Array
    };

    // Desired state: same position, zero velocity/accel
    const desired: TrajectoryState = {
        position: [...hoverPosition] as Vector3Array,
        velocity: [0, 0, 0] as Vector3Array,
        acceleration: [0, 0, 0] as Vector3Array,
        jerk: [0, 0, 0] as Vector3Array
    };

    let maxDrift = 0;
    const steps = Math.ceil(duration / dt);

    for (let i = 0; i < steps; i++) {
        const cmd = controller.update(state, desired, 0, 0, dt);
        state = physicsUpdate(state, cmd, physicsParams, dt);

        const drift = Math.sqrt(
            (state.position[0] - hoverPosition[0]) ** 2 +
            (state.position[1] - hoverPosition[1]) ** 2 +
            (state.position[2] - hoverPosition[2]) ** 2
        );
        maxDrift = Math.max(maxDrift, drift);

        // Early exit if diverging badly
        if (drift > 100) {
            console.error(`[HoverTest] FAILED: Drift exceeded 100m at step ${i}`);
            return { success: false, maxDrift, finalPosition: state.position };
        }
    }

    const success = maxDrift < 1.0; // Should stay within 1m for good hover
    console.log(`[HoverTest] ${success ? 'PASSED' : 'FAILED'}: maxDrift=${maxDrift.toFixed(3)}m`);

    return { success, maxDrift, finalPosition: state.position };
}
