/**
 * Control Module Tests
 * Tests for CascadeController, FlatnessMap, and QuaternionUtils
 */

import { describe, it, expect, beforeEach } from 'vitest';
import {
    CascadeController,
    DEFAULT_CONTROLLER_PARAMS,
    AGGRESSIVE_TRACKING_PARAMS,
    DEFAULT_QUADROTOR_PARAMS,
    simpleHoverController,
    FlatnessMap,
    QuaternionUtils,
} from '../src/models/robotics/control';
import type { QuadrotorState3D } from '../src/models/robotics/dynamics/types';
import type { TrajectoryState, Vector3Array } from '../src/models/planning/trajectory/types';
import type { Quaternion } from '../src/models/robotics/control/types';
import { isClose, arraysClose } from './test-utils';

// ==================== Helper Functions ====================

function createHoverState(position: Vector3Array): QuadrotorState3D {
    return {
        position,
        velocity: [0, 0, 0],
        quaternion: [1, 0, 0, 0], // Identity quaternion
        angularVelocity: [0, 0, 0],
    };
}

function createTrajectoryState(position: Vector3Array): TrajectoryState {
    return {
        position,
        velocity: [0, 0, 0],
        acceleration: [0, 0, 0],
        jerk: [0, 0, 0],
    };
}

function quaternionNorm(q: Quaternion): number {
    return Math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2);
}

// ==================== CascadeController Tests ====================

describe('CascadeController', () => {
    let controller: CascadeController;

    beforeEach(() => {
        controller = new CascadeController();
    });

    describe('constructor', () => {
        it('should create controller with default parameters', () => {
            const params = controller.getParams();
            expect(params.positionPID).toBeDefined();
            expect(params.velocityPID).toBeDefined();
            expect(params.attitudePID).toBeDefined();
            expect(params.angularRatePID).toBeDefined();
        });

        it('should create controller with custom parameters', () => {
            const customController = new CascadeController(AGGRESSIVE_TRACKING_PARAMS);
            const params = customController.getParams();
            expect(params.positionPID.Kp[0]).toBe(AGGRESSIVE_TRACKING_PARAMS.positionPID.Kp[0]);
        });

        it('should create controller with custom quad parameters', () => {
            const customQuadParams = { ...DEFAULT_QUADROTOR_PARAMS, mass: 2.0 };
            const customController = new CascadeController(DEFAULT_CONTROLLER_PARAMS, customQuadParams);
            expect(customController).toBeDefined();
        });
    });

    describe('reset', () => {
        it('should reset controller state without error', () => {
            // Perform some updates first to accumulate state
            const state = createHoverState([0, 0, 0]);
            const desired = createTrajectoryState([0, 0, 5]);
            controller.update(state, desired, 0, 0, 0.01);
            controller.update(state, desired, 0, 0, 0.01);

            // Reset should not throw
            expect(() => controller.reset()).not.toThrow();
        });
    });

    describe('update', () => {
        it('should return valid control command', () => {
            const state = createHoverState([0, 0, 0]);
            const desired = createTrajectoryState([0, 0, 5]);

            const command = controller.update(state, desired, 0, 0, 0.01);

            expect(command.thrust).toBeGreaterThan(0);
            expect(command.torque).toHaveLength(3);
            expect(Number.isFinite(command.thrust)).toBe(true);
            expect(command.torque.every(t => Number.isFinite(t))).toBe(true);
        });

        it('should produce positive thrust for upward target', () => {
            const state = createHoverState([0, 0, 0]);
            const desired = createTrajectoryState([0, 0, 10]);

            const command = controller.update(state, desired, 0, 0, 0.01);

            expect(command.thrust).toBeGreaterThan(DEFAULT_QUADROTOR_PARAMS.thrustMin);
        });

        it('should clamp thrust to physical limits', () => {
            const state = createHoverState([0, 0, 0]);
            const desired = createTrajectoryState([0, 0, 100]); // Very high target

            const command = controller.update(state, desired, 0, 0, 0.01);

            expect(command.thrust).toBeLessThanOrEqual(DEFAULT_QUADROTOR_PARAMS.thrustMax);
            expect(command.thrust).toBeGreaterThanOrEqual(DEFAULT_QUADROTOR_PARAMS.thrustMin);
        });

        it('should clamp torque to physical limits', () => {
            const state = createHoverState([0, 0, 0]);
            state.quaternion = [0.7, 0.7, 0, 0]; // Large attitude error
            const desired = createTrajectoryState([10, 10, 10]);

            const command = controller.update(state, desired, 0, 0, 0.01);

            for (let i = 0; i < 3; i++) {
                expect(Math.abs(command.torque[i])).toBeLessThanOrEqual(
                    DEFAULT_QUADROTOR_PARAMS.torqueMax[i]
                );
            }
        });

        it('should incorporate feedforward from desired state', () => {
            const state = createHoverState([0, 0, 5]);
            const desired: TrajectoryState = {
                position: [0, 0, 5],
                velocity: [1, 0, 0], // Moving in X direction
                acceleration: [0, 0, 0],
                jerk: [0, 0, 0],
            };

            const command = controller.update(state, desired, 0, 0, 0.01);

            // With feedforward velocity, attitude should tilt for tracking
            expect(command.thrust).toBeGreaterThan(0);
        });
    });

    describe('setParams / getParams', () => {
        it('should update controller parameters', () => {
            const newKp: Vector3Array = [5.0, 5.0, 10.0];
            controller.setParams({
                positionPID: { ...DEFAULT_CONTROLLER_PARAMS.positionPID, Kp: newKp },
            });

            const params = controller.getParams();
            expect(arraysClose(params.positionPID.Kp, newKp)).toBe(true);
        });

        it('should return a copy of params (not reference)', () => {
            const params1 = controller.getParams();
            const params2 = controller.getParams();
            expect(params1).not.toBe(params2);
        });
    });

    describe('setQuadParams', () => {
        it('should update quadrotor parameters', () => {
            controller.setQuadParams({ mass: 2.0 });
            // Internal state updated, test by checking control output changes
            const state = createHoverState([0, 0, 5]);
            const desired = createTrajectoryState([0, 0, 5]);
            const command = controller.update(state, desired, 0, 0, 0.01);
            expect(command.thrust).toBeGreaterThan(0);
        });
    });

    describe('setIntegralLimit / getIntegralLimit', () => {
        it('should set and get integral limit', () => {
            controller.setIntegralLimit(10.0);
            expect(controller.getIntegralLimit()).toBe(10.0);
        });

        it('should default to 5.0', () => {
            expect(controller.getIntegralLimit()).toBe(5.0);
        });
    });
});

// ==================== simpleHoverController Tests ====================

describe('simpleHoverController', () => {
    it('should produce upward thrust for below-target position', () => {
        const state = createHoverState([0, 0, 3]);
        const target: Vector3Array = [0, 0, 5];

        const command = simpleHoverController(state, target);

        expect(command.thrust).toBeGreaterThan(0);
    });

    it('should produce approximately hover thrust when at target', () => {
        const state = createHoverState([0, 0, 5]);
        const target: Vector3Array = [0, 0, 5];

        const command = simpleHoverController(state, target);

        // At target with zero velocity, thrust should be close to m*g
        const expectedHoverThrust = DEFAULT_QUADROTOR_PARAMS.mass * DEFAULT_QUADROTOR_PARAMS.gravity;
        expect(isClose(command.thrust, expectedHoverThrust, 0.5)).toBe(true);
    });

    it('should damp angular velocity', () => {
        const state: QuadrotorState3D = {
            position: [0, 0, 5],
            velocity: [0, 0, 0],
            quaternion: [1, 0, 0, 0],
            angularVelocity: [1, 0, 0], // Spinning
        };
        const target: Vector3Array = [0, 0, 5];

        const command = simpleHoverController(state, target);

        // Torque should oppose angular velocity
        expect(command.torque[0]).toBeLessThan(0);
    });

    it('should clamp thrust to physical limits', () => {
        const state = createHoverState([0, 0, 0]);
        const target: Vector3Array = [0, 0, 100]; // Very high

        const command = simpleHoverController(state, target);

        expect(command.thrust).toBeLessThanOrEqual(DEFAULT_QUADROTOR_PARAMS.thrustMax);
        expect(command.thrust).toBeGreaterThanOrEqual(DEFAULT_QUADROTOR_PARAMS.thrustMin);
    });

    it('should accept custom gains', () => {
        const state = createHoverState([0, 0, 3]);
        const target: Vector3Array = [0, 0, 5];

        const command1 = simpleHoverController(state, target, DEFAULT_QUADROTOR_PARAMS, 2.0, 1.0);
        const command2 = simpleHoverController(state, target, DEFAULT_QUADROTOR_PARAMS, 5.0, 2.0);

        // Higher gains should produce different (typically higher) thrust
        expect(command1.thrust).not.toBe(command2.thrust);
    });
});

// ==================== Default Parameters Tests ====================

describe('Default Parameters', () => {
    describe('DEFAULT_CONTROLLER_PARAMS', () => {
        it('should have all required PID gains', () => {
            expect(DEFAULT_CONTROLLER_PARAMS.positionPID.Kp).toHaveLength(3);
            expect(DEFAULT_CONTROLLER_PARAMS.positionPID.Ki).toHaveLength(3);
            expect(DEFAULT_CONTROLLER_PARAMS.positionPID.Kd).toHaveLength(3);
            expect(DEFAULT_CONTROLLER_PARAMS.velocityPID.Kp).toHaveLength(3);
            expect(DEFAULT_CONTROLLER_PARAMS.attitudePID.Kp).toHaveLength(3);
            expect(DEFAULT_CONTROLLER_PARAMS.angularRatePID.Kp).toHaveLength(3);
        });

        it('should have positive gains', () => {
            for (const kp of DEFAULT_CONTROLLER_PARAMS.positionPID.Kp) {
                expect(kp).toBeGreaterThan(0);
            }
        });
    });

    describe('AGGRESSIVE_TRACKING_PARAMS', () => {
        it('should have reduced integral gains compared to default', () => {
            // Aggressive params have lower Ki to prevent saturation
            expect(AGGRESSIVE_TRACKING_PARAMS.velocityPID.Ki[0]).toBeLessThan(
                DEFAULT_CONTROLLER_PARAMS.velocityPID.Ki[0]
            );
        });
    });

    describe('DEFAULT_QUADROTOR_PARAMS', () => {
        it('should have positive physical parameters', () => {
            expect(DEFAULT_QUADROTOR_PARAMS.mass).toBeGreaterThan(0);
            expect(DEFAULT_QUADROTOR_PARAMS.gravity).toBeGreaterThan(0);
            expect(DEFAULT_QUADROTOR_PARAMS.thrustMax).toBeGreaterThan(DEFAULT_QUADROTOR_PARAMS.thrustMin);
        });

        it('should have valid inertia', () => {
            expect(DEFAULT_QUADROTOR_PARAMS.inertia).toHaveLength(3);
            for (const I of DEFAULT_QUADROTOR_PARAMS.inertia) {
                expect(I).toBeGreaterThan(0);
            }
        });
    });
});

// ==================== FlatnessMap Tests ====================

describe('FlatnessMap', () => {
    let flatnessMap: FlatnessMap;

    beforeEach(() => {
        flatnessMap = new FlatnessMap();
    });

    describe('forward', () => {
        it('should produce valid control output for hover state', () => {
            const state: TrajectoryState = {
                position: [0, 0, 5],
                velocity: [0, 0, 0],
                acceleration: [0, 0, 0],
                jerk: [0, 0, 0],
            };

            const output = flatnessMap.forward(state, 0, 0);

            expect(output.thrust).toBeGreaterThan(0);
            expect(output.quaternion).toHaveLength(4);
            expect(output.bodyRate).toHaveLength(3);
        });

        it('should return unit quaternion', () => {
            const state: TrajectoryState = {
                position: [0, 0, 5],
                velocity: [1, 0, 0],
                acceleration: [0, 0, 0],
                jerk: [0, 0, 0],
            };

            const output = flatnessMap.forward(state, 0, 0);
            const norm = quaternionNorm(output.quaternion);

            expect(isClose(norm, 1, 1e-6)).toBe(true);
        });

        it('should produce higher thrust for upward acceleration', () => {
            const hoverState: TrajectoryState = {
                position: [0, 0, 5],
                velocity: [0, 0, 0],
                acceleration: [0, 0, 0],
                jerk: [0, 0, 0],
            };

            const accelState: TrajectoryState = {
                position: [0, 0, 5],
                velocity: [0, 0, 0],
                acceleration: [0, 0, 5], // Upward acceleration
                jerk: [0, 0, 0],
            };

            const hoverOutput = flatnessMap.forward(hoverState, 0, 0);
            const accelOutput = flatnessMap.forward(accelState, 0, 0);

            expect(accelOutput.thrust).toBeGreaterThan(hoverOutput.thrust);
        });

        it('should tilt for horizontal acceleration', () => {
            const state: TrajectoryState = {
                position: [0, 0, 5],
                velocity: [0, 0, 0],
                acceleration: [2, 0, 0], // Accelerating in X
                jerk: [0, 0, 0],
            };

            const output = flatnessMap.forward(state, 0, 0);

            // Quaternion should not be identity (tilted)
            expect(Math.abs(output.quaternion[1]) + Math.abs(output.quaternion[2])).toBeGreaterThan(0.01);
        });

        it('should incorporate yaw angle', () => {
            const state: TrajectoryState = {
                position: [0, 0, 5],
                velocity: [0, 0, 0],
                acceleration: [0, 0, 0],
                jerk: [0, 0, 0],
            };

            const output0 = flatnessMap.forward(state, 0, 0);
            const output90 = flatnessMap.forward(state, Math.PI / 2, 0);

            // Different yaw should produce different quaternion
            expect(output0.quaternion[3]).not.toBe(output90.quaternion[3]);
        });
    });

    describe('backward', () => {
        it('should return gradient vectors', () => {
            const state: TrajectoryState = {
                position: [0, 0, 5],
                velocity: [1, 0, 0],
                acceleration: [0, 0, 0],
                jerk: [0, 0, 0],
            };

            const grads = flatnessMap.backward(state, 0, 0, 1.0, [0, 0, 0, 0], [0, 0, 0]);

            expect(grads.velGrad).toHaveLength(3);
            expect(grads.accGrad).toHaveLength(3);
            expect(grads.jerkGrad).toHaveLength(3);
        });
    });

    describe('checkThrustConstraint', () => {
        it('should return true for valid thrust', () => {
            expect(flatnessMap.checkThrustConstraint(10, 2, 30)).toBe(true);
        });

        it('should return false for thrust below minimum', () => {
            expect(flatnessMap.checkThrustConstraint(1, 2, 30)).toBe(false);
        });

        it('should return false for thrust above maximum', () => {
            expect(flatnessMap.checkThrustConstraint(35, 2, 30)).toBe(false);
        });
    });

    describe('checkTiltConstraint', () => {
        it('should return true for identity quaternion (no tilt)', () => {
            const q: Quaternion = [1, 0, 0, 0];
            expect(flatnessMap.checkTiltConstraint(q, Math.PI / 4)).toBe(true);
        });

        it('should return false for large tilt', () => {
            // 90 degree tilt around X axis
            const q: Quaternion = [Math.cos(Math.PI / 4), Math.sin(Math.PI / 4), 0, 0];
            expect(flatnessMap.checkTiltConstraint(q, Math.PI / 6)).toBe(false);
        });
    });

    describe('checkBodyRateConstraint', () => {
        it('should return true for zero rate', () => {
            expect(flatnessMap.checkBodyRateConstraint([0, 0, 0], 10)).toBe(true);
        });

        it('should return true for rate within limit', () => {
            expect(flatnessMap.checkBodyRateConstraint([1, 1, 1], 10)).toBe(true);
        });

        it('should return false for rate exceeding limit', () => {
            expect(flatnessMap.checkBodyRateConstraint([5, 5, 5], 5)).toBe(false);
        });
    });

    describe('reset', () => {
        it('should reset physics parameters', () => {
            const params = {
                mass: 2.0,
                gravity: 10.0,
                horizDragCoeff: 0.1,
                vertDragCoeff: 0.1,
                parasDragCoeff: 0.05,
                speedEps: 1e-4,
            };

            flatnessMap.reset(params);

            // Verify by checking thrust calculation changes
            const state: TrajectoryState = {
                position: [0, 0, 5],
                velocity: [0, 0, 0],
                acceleration: [0, 0, 0],
                jerk: [0, 0, 0],
            };

            const output = flatnessMap.forward(state, 0, 0);
            // Thrust should be mass * gravity for hover
            expect(isClose(output.thrust, 2.0 * 10.0, 0.1)).toBe(true);
        });
    });
});

// ==================== QuaternionUtils Tests ====================

describe('QuaternionUtils', () => {
    describe('multiply', () => {
        it('should return identity when multiplying by identity', () => {
            const q: Quaternion = [0.7, 0.3, 0.4, 0.5];
            const qNorm = QuaternionUtils.normalize(q);
            const identity: Quaternion = [1, 0, 0, 0];

            const result = QuaternionUtils.multiply(qNorm, identity);

            expect(isClose(result[0], qNorm[0], 1e-10)).toBe(true);
            expect(isClose(result[1], qNorm[1], 1e-10)).toBe(true);
            expect(isClose(result[2], qNorm[2], 1e-10)).toBe(true);
            expect(isClose(result[3], qNorm[3], 1e-10)).toBe(true);
        });

        it('should preserve unit norm', () => {
            const q1: Quaternion = QuaternionUtils.normalize([0.7, 0.3, 0.4, 0.5]);
            const q2: Quaternion = QuaternionUtils.normalize([0.5, 0.5, 0.5, 0.5]);

            const result = QuaternionUtils.multiply(q1, q2);
            const norm = quaternionNorm(result);

            expect(isClose(norm, 1, 1e-10)).toBe(true);
        });

        it('should be non-commutative', () => {
            const q1: Quaternion = QuaternionUtils.normalize([1, 1, 0, 0]);
            const q2: Quaternion = QuaternionUtils.normalize([1, 0, 1, 0]);

            const r1 = QuaternionUtils.multiply(q1, q2);
            const r2 = QuaternionUtils.multiply(q2, q1);

            // Quaternion multiplication is generally non-commutative
            // Compare the z component which differs for these rotations
            expect(r1[3]).not.toBeCloseTo(r2[3], 5);
        });
    });

    describe('toEuler', () => {
        it('should return zero angles for identity quaternion', () => {
            const euler = QuaternionUtils.toEuler([1, 0, 0, 0]);

            expect(isClose(euler.roll, 0, 1e-10)).toBe(true);
            expect(isClose(euler.pitch, 0, 1e-10)).toBe(true);
            expect(isClose(euler.yaw, 0, 1e-10)).toBe(true);
        });

        it('should recover yaw rotation correctly', () => {
            // 90 degree yaw rotation
            const q = QuaternionUtils.fromEuler(0, 0, Math.PI / 2);
            const euler = QuaternionUtils.toEuler(q);

            expect(isClose(euler.yaw, Math.PI / 2, 0.01)).toBe(true);
            expect(isClose(euler.roll, 0, 0.01)).toBe(true);
            expect(isClose(euler.pitch, 0, 0.01)).toBe(true);
        });

        it('should recover roll rotation correctly', () => {
            // 45 degree roll
            const q = QuaternionUtils.fromEuler(Math.PI / 4, 0, 0);
            const euler = QuaternionUtils.toEuler(q);

            expect(isClose(euler.roll, Math.PI / 4, 0.01)).toBe(true);
        });
    });

    describe('fromEuler', () => {
        it('should return identity for zero angles', () => {
            const q = QuaternionUtils.fromEuler(0, 0, 0);

            expect(isClose(q[0], 1, 1e-10)).toBe(true);
            expect(isClose(q[1], 0, 1e-10)).toBe(true);
            expect(isClose(q[2], 0, 1e-10)).toBe(true);
            expect(isClose(q[3], 0, 1e-10)).toBe(true);
        });

        it('should produce unit quaternion', () => {
            const q = QuaternionUtils.fromEuler(0.5, 0.3, 0.7);
            const norm = quaternionNorm(q);

            expect(isClose(norm, 1, 1e-10)).toBe(true);
        });

        it('should be inverse of toEuler', () => {
            const roll = 0.3, pitch = 0.2, yaw = 0.5;
            const q = QuaternionUtils.fromEuler(roll, pitch, yaw);
            const euler = QuaternionUtils.toEuler(q);

            expect(isClose(euler.roll, roll, 0.01)).toBe(true);
            expect(isClose(euler.pitch, pitch, 0.01)).toBe(true);
            expect(isClose(euler.yaw, yaw, 0.01)).toBe(true);
        });
    });

    describe('normalize', () => {
        it('should return unit quaternion', () => {
            const q: Quaternion = [1, 2, 3, 4];
            const normalized = QuaternionUtils.normalize(q);
            const norm = quaternionNorm(normalized);

            expect(isClose(norm, 1, 1e-10)).toBe(true);
        });

        it('should preserve direction', () => {
            const q: Quaternion = [2, 0, 0, 0];
            const normalized = QuaternionUtils.normalize(q);

            expect(normalized[0]).toBe(1);
            expect(normalized[1]).toBe(0);
            expect(normalized[2]).toBe(0);
            expect(normalized[3]).toBe(0);
        });

        it('should handle already-normalized quaternion', () => {
            const q: Quaternion = [1, 0, 0, 0];
            const normalized = QuaternionUtils.normalize(q);

            expect(normalized[0]).toBe(1);
            expect(normalized[1]).toBe(0);
        });
    });
});
