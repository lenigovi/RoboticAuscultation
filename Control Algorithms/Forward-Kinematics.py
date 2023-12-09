#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import numpy as np
from scipy.spatial.transform import Rotation

class Robot:
    def __init__(self, joint_angles):
        self.joint_angles = joint_angles

    def ForwardKinematics(self):
        link_lengths = [1, 1, 1]
        joint_axes = np.array([[0, 0, 1], [0, 1, 0], [0, 1, 0]])

        T = np.eye(4)

        for i in range(len(self.joint_angles)):
            theta = self.joint_angles[i]
            axis = joint_axes[i]

            exp_screw = self.create_screw_matrix(axis, theta)

            T = np.dot(T, exp_screw)

        return T

    def create_screw_matrix(self, axis, angle):
        omega = np.array(axis)
        omega_hat = np.array([[0, -omega[2], omega[1]],
                             [omega[2], 0, -omega[0]],
                             [-omega[1], omega[0], 0]])

        rotation_matrix = Rotation.from_rotvec(angle * omega).as_matrix()

        upper_left = rotation_matrix
        upper_right = np.dot((np.eye(3) - rotation_matrix), omega_hat) + np.outer(omega, omega) * angle
        lower_left = np.zeros((3, 3))
        lower_right = rotation_matrix

        screw_matrix = np.block([[upper_left, upper_right],
                                [lower_left, lower_right]])

        return np.linalg.expm(screw_matrix)

    def jacobian(self):
        epsilon = 1e-6
        J = np.zeros((6, len(self.joint_angles)))

        for i in range(len(self.joint_angles)):
            joint_angles_pos = np.array(self.joint_angles)
            joint_angles_pos[i] += epsilon
            pos_pos = Robot(joint_angles_pos).forward_kinematics()[:3, 3]
            pos_current = self.forward_kinematics()[:3, 3]
            J[:3, i] = (pos_pos - pos_current) / epsilon

            joint_angles_pos = np.array(self.joint_angles)
            joint_angles_pos[i] += epsilon
            orientation_pos = Rotation.from_matrix(Robot(joint_angles_pos).forward_kinematics()[:3, :3]).as_rotvec()
            orientation_current = Rotation.from_matrix(self.forward_kinematics()[:3, :3]).as_rotvec()
            J[3:, i] = (orientation_pos - orientation_current) / epsilon

        return J

def main():
    joint_angles = [0.1, 0.2, 0.3]
    robot = Robot(joint_angles)

    transformation_matrix = robot.forward_kinematics()

    position = transformation_matrix[:3, 3]
    orientation = Rotation.from_matrix(transformation_matrix[:3, :3]).as_rotvec()

    print("Forward Kinematics Results:")
    print("Position:", position)
    print("Orientation:", orientation)

    jacobian_matrix = robot.jacobian()
    print("\nJacobian Matrix:")
    print(jacobian_matrix)

if __name__ == "__main__":
    main()

