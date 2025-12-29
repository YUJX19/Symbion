/**
 * Dynamics Module Tests
 * Tests for kinematics (differential drive, omni, Ackermann, quadrotor)
 * and physics (quadrotor rigid body dynamics, quaternion operations)
 */

import { describe, it, expect } from 'vitest';
import {
    // Kinematics
    differentialDriveUpdate,
    omniUpdate,
    ackermannUpdate,
    quadrotorUpdate,
    applyKinematicsNoise,
} from '../src/models/robotics/dynamics/kinematics';
import {
    // Physics
    quadrotorPhysicsUpdate,
    rotateVectorByQuaternion,
    quaternionIntegration,
    quaternionMultiply,
    normalizeQuaternion,
    quaternionConjugate,
    quaternionToRotationMatrix,
    cross,
    dot,
    addVectors,
    subtractVectors,
    scaleVector,
    magnitude,
    identityQuaternion,
    createHoverState,
    computeHoverThrust,
} from '../src/models/robotics/dynamics/physics';
import type { QuadrotorState3D, QuadrotorParams, Vector3Array, Quaternion } from '../src/models/robotics/dynamics/types';
import { isClose, arraysClose, matricesClose } from './test-utils';

// ==================== Differential Drive Kinematics ====================

describe('Differential Drive Kinematics', () => {
    describe('differentialDriveUpdate', () => {
        it('should move straight when both wheels have equal velocity', () => {
            const pose = { x: 0, y: 0, theta: 0 };
            const newPose = differentialDriveUpdate(pose, 1, 1, 0.5, 1);

            expect(newPose.x).toBeGreaterThan(0);
            expect(isClose(newPose.y, 0, 1e-10)).toBe(true);
            expect(isClose(newPose.theta, 0, 1e-10)).toBe(true);
        });

        it('should turn when wheels have different velocities', () => {
            const pose = { x: 0, y: 0, theta: 0 };
            // Right faster than left → turns left (positive theta)
            const newPose = differentialDriveUpdate(pose, 0.5, 1, 0.5, 1);

            expect(newPose.theta).toBeGreaterThan(0);
        });

        it('should rotate in place when wheels have opposite velocities', () => {
            const pose = { x: 0, y: 0, theta: 0 };
            // Left forward, right backward → pure rotation
            const newPose = differentialDriveUpdate(pose, 0.5, -0.5, 0.5, 1);

            // Position should not change significantly
            expect(Math.abs(newPose.x)).toBeLessThan(0.1);
            expect(Math.abs(newPose.y)).toBeLessThan(0.1);
            // But heading should change
            expect(newPose.theta).not.toBeCloseTo(0, 5);
        });

        it('should respect wheelbase parameter', () => {
            const pose = { x: 0, y: 0, theta: 0 };
            // Same wheel velocities, different wheelbase
            const narrow = differentialDriveUpdate(pose, 0.5, 1, 0.3, 1);
            const wide = differentialDriveUpdate(pose, 0.5, 1, 0.6, 1);

            // Narrower wheelbase should cause tighter turn (more rotation)
            expect(Math.abs(narrow.theta)).toBeGreaterThan(Math.abs(wide.theta));
        });

        it('should handle dt correctly', () => {
            const pose = { x: 0, y: 0, theta: 0 };
            const pose1 = differentialDriveUpdate(pose, 1, 1, 0.5, 0.5);
            const pose2 = differentialDriveUpdate(pose, 1, 1, 0.5, 1);

            // Double dt should give approximately double displacement
            expect(isClose(pose2.x, 2 * pose1.x, 0.01)).toBe(true);
        });
    });
});

// ==================== Omnidirectional Kinematics ====================

describe('Omnidirectional Kinematics', () => {
    describe('omniUpdate', () => {
        it('should move in heading direction with vx only', () => {
            const pose = { x: 0, y: 0, theta: 0 };
            const newPose = omniUpdate(pose, 1, 0, 0, 1);

            expect(isClose(newPose.x, 1, 1e-10)).toBe(true);
            expect(isClose(newPose.y, 0, 1e-10)).toBe(true);
        });

        it('should move perpendicular with vy only', () => {
            const pose = { x: 0, y: 0, theta: 0 };
            const newPose = omniUpdate(pose, 0, 1, 0, 1);

            expect(isClose(newPose.x, 0, 1e-10)).toBe(true);
            expect(isClose(newPose.y, 1, 1e-10)).toBe(true);
        });

        it('should rotate in place with omega only', () => {
            const pose = { x: 5, y: 5, theta: 0 };
            const newPose = omniUpdate(pose, 0, 0, Math.PI, 0.5);

            expect(isClose(newPose.x, 5, 1e-10)).toBe(true);
            expect(isClose(newPose.y, 5, 1e-10)).toBe(true);
            expect(isClose(newPose.theta, Math.PI / 2, 1e-10)).toBe(true);
        });

        it('should transform velocities to world frame correctly', () => {
            const pose = { x: 0, y: 0, theta: Math.PI / 2 }; // Facing +Y
            const newPose = omniUpdate(pose, 1, 0, 0, 1);

            // vx in body should become vy in world
            expect(isClose(newPose.x, 0, 1e-10)).toBe(true);
            expect(isClose(newPose.y, 1, 1e-10)).toBe(true);
        });
    });
});

// ==================== Ackermann Steering Kinematics ====================

describe('Ackermann Steering Kinematics', () => {
    describe('ackermannUpdate', () => {
        it('should move straight with zero steering', () => {
            const pose = { x: 0, y: 0, theta: 0 };
            const newPose = ackermannUpdate(pose, 1, 0, 0.5, 1);

            expect(isClose(newPose.x, 1, 1e-10)).toBe(true);
            expect(isClose(newPose.y, 0, 1e-10)).toBe(true);
            expect(isClose(newPose.theta, 0, 1e-10)).toBe(true);
        });

        it('should turn with positive steering angle', () => {
            const pose = { x: 0, y: 0, theta: 0 };
            const newPose = ackermannUpdate(pose, 1, 0.3, 0.5, 1);

            expect(newPose.theta).toBeGreaterThan(0);
            expect(newPose.y).toBeGreaterThan(0);
        });

        it('should limit steering angle', () => {
            const pose = { x: 0, y: 0, theta: 0 };
            // Very large steering angle
            const newPose1 = ackermannUpdate(pose, 1, 2, 0.5, 1);
            const newPose2 = ackermannUpdate(pose, 1, Math.PI / 4, 0.5, 1);

            // Should be clamped to max of π/4
            expect(isClose(newPose1.theta, newPose2.theta, 0.01)).toBe(true);
        });

        it('should handle reverse motion', () => {
            const pose = { x: 5, y: 5, theta: 0 };
            const newPose = ackermannUpdate(pose, -1, 0, 0.5, 1);

            expect(newPose.x).toBeLessThan(5);
        });
    });
});

// ==================== Quadrotor 2D Kinematics ====================

describe('Quadrotor 2D Kinematics', () => {
    describe('quadrotorUpdate', () => {
        it('should move in heading direction with vx only', () => {
            const pose = { x: 0, y: 0, theta: 0 };
            const newPose = quadrotorUpdate(pose, 1, 0, 0, 1);

            expect(isClose(newPose.x, 1, 1e-10)).toBe(true);
            expect(isClose(newPose.y, 0, 1e-10)).toBe(true);
        });

        it('should strafe with vy only', () => {
            const pose = { x: 0, y: 0, theta: 0 };
            const newPose = quadrotorUpdate(pose, 0, 1, 0, 1);

            expect(isClose(newPose.x, 0, 1e-10)).toBe(true);
            expect(isClose(newPose.y, 1, 1e-10)).toBe(true);
        });

        it('should rotate with omega', () => {
            const pose = { x: 0, y: 0, theta: 0 };
            const newPose = quadrotorUpdate(pose, 0, 0, 1, 1);

            expect(isClose(newPose.theta, 1, 1e-10)).toBe(true);
        });

        it('should correctly transform body velocities', () => {
            const pose = { x: 0, y: 0, theta: Math.PI / 4 };
            const newPose = quadrotorUpdate(pose, 1, 0, 0, 1);

            // At 45 degrees, vx should split into x and y
            const expected = Math.cos(Math.PI / 4);
            expect(isClose(newPose.x, expected, 1e-10)).toBe(true);
            expect(isClose(newPose.y, expected, 1e-10)).toBe(true);
        });
    });
});

// ==================== Kinematics Noise Model ====================

describe('Kinematics Noise Model', () => {
    describe('applyKinematicsNoise', () => {
        it('should add noise to pose', () => {
            const prev = { x: 0, y: 0, theta: 0 };
            const next = { x: 1, y: 0, theta: 0 };
            const alpha: [number, number, number, number] = [0.1, 0.1, 0.1, 0.1];

            // Run multiple times to check it adds randomness
            const results = [];
            for (let i = 0; i < 100; i++) {
                const noisy = applyKinematicsNoise(prev, next, alpha);
                results.push(noisy.x);
            }

            // Should have different values
            const unique = new Set(results.map(x => x.toFixed(6)));
            expect(unique.size).toBeGreaterThan(50);
        });

        it('should return similar pose with zero noise coefficients', () => {
            const prev = { x: 0, y: 0, theta: 0 };
            const next = { x: 1, y: 0.5, theta: 0.1 };
            const alpha: [number, number, number, number] = [0, 0, 0, 0];

            const noisy = applyKinematicsNoise(prev, next, alpha);

            // Very small noise coefficients still allow some numerical error
            expect(Math.abs(noisy.x - next.x)).toBeLessThan(0.01);
            expect(Math.abs(noisy.y - next.y)).toBeLessThan(0.01);
        });
    });
});

// ==================== Quaternion Operations ====================

describe('Quaternion Operations', () => {
    describe('identityQuaternion', () => {
        it('should return [1, 0, 0, 0]', () => {
            expect(identityQuaternion()).toEqual([1, 0, 0, 0]);
        });
    });

    describe('normalizeQuaternion', () => {
        it('should normalize to unit length', () => {
            const q: Quaternion = [2, 0, 0, 0];
            const normalized = normalizeQuaternion(q);
            const len = magnitude([normalized[0], normalized[1], normalized[2]]);
            const totalLen = Math.sqrt(
                normalized[0] ** 2 + normalized[1] ** 2 +
                normalized[2] ** 2 + normalized[3] ** 2
            );
            expect(isClose(totalLen, 1, 1e-10)).toBe(true);
        });

        it('should handle degenerate (zero) quaternion', () => {
            const q: Quaternion = [0, 0, 0, 0];
            const normalized = normalizeQuaternion(q);
            expect(normalized).toEqual([1, 0, 0, 0]);
        });

        it('should preserve identity quaternion', () => {
            const q = identityQuaternion();
            expect(normalizeQuaternion(q)).toEqual([1, 0, 0, 0]);
        });
    });

    describe('quaternionConjugate', () => {
        it('should negate imaginary parts', () => {
            const q: Quaternion = [0.5, 0.5, 0.5, 0.5];
            const conj = quaternionConjugate(q);
            expect(conj).toEqual([0.5, -0.5, -0.5, -0.5]);
        });
    });

    describe('quaternionMultiply', () => {
        it('should satisfy q * q_conj = identity for unit quaternion', () => {
            const q: Quaternion = normalizeQuaternion([1, 1, 1, 1]);
            const conj = quaternionConjugate(q);
            const result = quaternionMultiply(q, conj);

            expect(isClose(result[0], 1, 1e-10)).toBe(true);
            expect(isClose(result[1], 0, 1e-10)).toBe(true);
            expect(isClose(result[2], 0, 1e-10)).toBe(true);
            expect(isClose(result[3], 0, 1e-10)).toBe(true);
        });

        it('should satisfy identity property: q * I = q', () => {
            const q: Quaternion = normalizeQuaternion([1, 2, 3, 4]);
            const I = identityQuaternion();
            const result = quaternionMultiply(q, I);

            expect(arraysClose(result, q, 1e-10)).toBe(true);
        });

        it('should be associative: (q1 * q2) * q3 = q1 * (q2 * q3)', () => {
            const q1: Quaternion = normalizeQuaternion([1, 0, 1, 0]);
            const q2: Quaternion = normalizeQuaternion([0, 1, 0, 1]);
            const q3: Quaternion = normalizeQuaternion([1, 1, 0, 0]);

            const left = quaternionMultiply(quaternionMultiply(q1, q2), q3);
            const right = quaternionMultiply(q1, quaternionMultiply(q2, q3));

            expect(arraysClose(left, right, 1e-10)).toBe(true);
        });
    });

    describe('rotateVectorByQuaternion', () => {
        it('should not change vector with identity quaternion', () => {
            const v: Vector3Array = [1, 2, 3];
            const q = identityQuaternion();
            const rotated = rotateVectorByQuaternion(v, q);

            expect(arraysClose(rotated, v, 1e-10)).toBe(true);
        });

        it('should rotate 90° around Z axis correctly', () => {
            const v: Vector3Array = [1, 0, 0];
            // 90° around Z: q = [cos(45°), 0, 0, sin(45°)]
            const angle = Math.PI / 2;
            const q: Quaternion = [
                Math.cos(angle / 2),
                0,
                0,
                Math.sin(angle / 2)
            ];
            const rotated = rotateVectorByQuaternion(v, q);

            // [1,0,0] rotated 90° around Z → [0,1,0]
            expect(isClose(rotated[0], 0, 1e-10)).toBe(true);
            expect(isClose(rotated[1], 1, 1e-10)).toBe(true);
            expect(isClose(rotated[2], 0, 1e-10)).toBe(true);
        });

        it('should preserve vector magnitude', () => {
            const v: Vector3Array = [3, 4, 5];
            const q: Quaternion = normalizeQuaternion([1, 1, 1, 1]);
            const rotated = rotateVectorByQuaternion(v, q);

            const originalMag = magnitude(v);
            const rotatedMag = magnitude(rotated);
            expect(isClose(rotatedMag, originalMag, 1e-10)).toBe(true);
        });
    });

    describe('quaternionIntegration', () => {
        it('should not change quaternion with zero angular velocity', () => {
            const q = identityQuaternion();
            const omega: Vector3Array = [0, 0, 0];
            const newQ = quaternionIntegration(q, omega, 1);

            expect(arraysClose(newQ, q, 1e-10)).toBe(true);
        });

        it('should rotate around Z axis with omega_z', () => {
            const q = identityQuaternion();
            const omega: Vector3Array = [0, 0, Math.PI]; // π rad/s around Z
            const newQ = quaternionIntegration(q, omega, 0.5); // 0.5s → 90°

            // After 90° rotation around Z
            const v: Vector3Array = [1, 0, 0];
            const rotated = rotateVectorByQuaternion(v, newQ);

            expect(isClose(rotated[0], 0, 1e-5)).toBe(true);
            expect(isClose(rotated[1], 1, 1e-5)).toBe(true);
        });

        it('should maintain unit quaternion', () => {
            const q: Quaternion = normalizeQuaternion([1, 0.5, 0.3, 0.1]);
            const omega: Vector3Array = [1, 2, 3];
            const newQ = quaternionIntegration(q, omega, 0.01);

            const norm = Math.sqrt(
                newQ[0] ** 2 + newQ[1] ** 2 + newQ[2] ** 2 + newQ[3] ** 2
            );
            expect(isClose(norm, 1, 1e-10)).toBe(true);
        });
    });

    describe('quaternionToRotationMatrix', () => {
        it('should give identity matrix for identity quaternion', () => {
            const q = identityQuaternion();
            const R = quaternionToRotationMatrix(q);

            expect(matricesClose(R, [
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]
            ], 1e-10)).toBe(true);
        });

        it('should be orthogonal (R * R^T = I)', () => {
            const q: Quaternion = normalizeQuaternion([1, 1, 0, 0]);
            const R = quaternionToRotationMatrix(q);

            // Compute R * R^T
            const RRT = [
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0]
            ];
            for (let i = 0; i < 3; i++) {
                for (let j = 0; j < 3; j++) {
                    for (let k = 0; k < 3; k++) {
                        RRT[i][j] += R[i][k] * R[j][k];
                    }
                }
            }

            expect(matricesClose(RRT, [
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1]
            ], 1e-10)).toBe(true);
        });
    });
});

// ==================== Vector Operations ====================

describe('Vector Operations', () => {
    describe('cross', () => {
        it('should compute i × j = k', () => {
            const result = cross([1, 0, 0], [0, 1, 0]);
            expect(result).toEqual([0, 0, 1]);
        });

        it('should be anti-commutative', () => {
            const a: Vector3Array = [1, 2, 3];
            const b: Vector3Array = [4, 5, 6];
            const ab = cross(a, b);
            const ba = cross(b, a);

            expect(arraysClose(ab, [-ba[0], -ba[1], -ba[2]], 1e-10)).toBe(true);
        });

        it('should return zero for parallel vectors', () => {
            const result = cross([2, 4, 6], [1, 2, 3]);
            expect(arraysClose(result, [0, 0, 0], 1e-10)).toBe(true);
        });
    });

    describe('dot', () => {
        it('should compute dot product correctly', () => {
            expect(dot([1, 2, 3], [4, 5, 6])).toBe(32);
        });

        it('should return 0 for orthogonal vectors', () => {
            expect(dot([1, 0, 0], [0, 1, 0])).toBe(0);
        });
    });

    describe('addVectors', () => {
        it('should add element-wise', () => {
            expect(addVectors([1, 2, 3], [4, 5, 6])).toEqual([5, 7, 9]);
        });
    });

    describe('subtractVectors', () => {
        it('should subtract element-wise', () => {
            expect(subtractVectors([4, 5, 6], [1, 2, 3])).toEqual([3, 3, 3]);
        });
    });

    describe('scaleVector', () => {
        it('should scale by scalar', () => {
            expect(scaleVector([1, 2, 3], 2)).toEqual([2, 4, 6]);
        });

        it('should handle negative scalar', () => {
            expect(scaleVector([1, 2, 3], -1)).toEqual([-1, -2, -3]);
        });
    });

    describe('magnitude', () => {
        it('should compute Euclidean norm', () => {
            expect(magnitude([3, 4, 0])).toBe(5);
        });

        it('should handle 3D vector', () => {
            expect(isClose(magnitude([1, 1, 1]), Math.sqrt(3), 1e-10)).toBe(true);
        });
    });
});

// ==================== Quadrotor Physics ====================

describe('Quadrotor Physics', () => {
    const testParams: QuadrotorParams = {
        mass: 1.0,
        inertia: [0.01, 0.01, 0.02],
        dragCoeff: [0.1, 0.1, 0.1],
        gravity: 9.81,
        thrustMin: 0,
        thrustMax: 30,
        torqueMax: [1, 1, 1]
    };

    describe('createHoverState', () => {
        it('should create state at given position', () => {
            const state = createHoverState([1, 2, 3]);
            expect(state.position).toEqual([1, 2, 3]);
            expect(state.velocity).toEqual([0, 0, 0]);
            expect(state.angularVelocity).toEqual([0, 0, 0]);
        });
    });

    describe('computeHoverThrust', () => {
        it('should compute thrust = mg', () => {
            expect(computeHoverThrust(1.5, 9.81)).toBe(1.5 * 9.81);
        });
    });

    describe('quadrotorPhysicsUpdate', () => {
        it('should maintain hover with correct thrust', () => {
            const state = createHoverState([0, 0, 10]);
            const thrust = computeHoverThrust(testParams.mass, testParams.gravity);
            const control = { thrust, torque: [0, 0, 0] as Vector3Array };

            const newState = quadrotorPhysicsUpdate(state, control, testParams, 0.01);

            // Should stay roughly at same position
            expect(isClose(newState.position[2], 10, 0.01)).toBe(true);
            expect(Math.abs(newState.velocity[2])).toBeLessThan(0.1);
        });

        it('should fall with zero thrust', () => {
            const state = createHoverState([0, 0, 10]);
            const control = { thrust: 0, torque: [0, 0, 0] as Vector3Array };

            const newState = quadrotorPhysicsUpdate(state, control, testParams, 0.1);

            // Should have negative Z velocity (falling)
            expect(newState.velocity[2]).toBeLessThan(0);
            expect(newState.position[2]).toBeLessThan(10);
        });

        it('should accelerate upward with excess thrust', () => {
            const state = createHoverState([0, 0, 10]);
            const hoverThrust = computeHoverThrust(testParams.mass, testParams.gravity);
            const control = { thrust: hoverThrust * 1.5, torque: [0, 0, 0] as Vector3Array };

            const newState = quadrotorPhysicsUpdate(state, control, testParams, 0.1);

            expect(newState.velocity[2]).toBeGreaterThan(0);
            expect(newState.position[2]).toBeGreaterThan(10);
        });

        it('should rotate with torque', () => {
            const state = createHoverState([0, 0, 10]);
            const thrust = computeHoverThrust(testParams.mass, testParams.gravity);
            const control = { thrust, torque: [0, 0, 0.1] as Vector3Array }; // Yaw torque

            const newState = quadrotorPhysicsUpdate(state, control, testParams, 0.1);

            // Should have angular velocity around Z
            expect(newState.angularVelocity[2]).toBeGreaterThan(0);
        });

        it('should apply drag to reduce velocity', () => {
            const state: QuadrotorState3D = {
                position: [0, 0, 10],
                velocity: [10, 0, 0], // Moving fast in X
                quaternion: identityQuaternion(),
                angularVelocity: [0, 0, 0]
            };
            const thrust = computeHoverThrust(testParams.mass, testParams.gravity);
            const control = { thrust, torque: [0, 0, 0] as Vector3Array };

            const newState = quadrotorPhysicsUpdate(state, control, testParams, 0.1);

            // Drag should slow it down
            expect(Math.abs(newState.velocity[0])).toBeLessThan(10);
        });
    });
});
