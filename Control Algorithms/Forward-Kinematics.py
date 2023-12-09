#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import numpy as np
from scipy.spatial.transform import Rotation

class ForwardKinematics:
    def __init__(self):
        self.PEE = np.array([0, 0, 1.953])
        tool_orientation = Rotation.from_axis_angle([0, 1, 0], 0.0).as_quat()
        self.M = DualQuaternion.from_quat_pos(tool_orientation, self.PEE)

        self.screws_0 = [
            DualQuaternion.screw_axis([0, 0, 1], [0, 0, 0]),
            DualQuaternion.screw_axis([0, 1, 0], [0, 0, 0]),
            DualQuaternion.screw_axis([0, 0, 1], [0, 0, self.d1]),
            DualQuaternion.screw_axis([0, 1, 0], [0, 0, self.d1 + self.d3]),
            DualQuaternion.screw_axis([0, 0, 1], [0, 0, self.d1 + self.d3 + self.d5]),
            DualQuaternion.screw_axis([0, 1, 0], [0, 0, self.d1 + self.d3 + self.d5 + self.d7]),
            DualQuaternion.screw_axis([0, 0, 1], [0, 0, self.d1 + self.d3 + self.d5 + self.d7])
        ]

    def forward_kinematics(self, theta):
        x = DualQuaternion(1, 0, 0, 0, 0, 0, 0, 0)
        for i in range(len(theta)):
            x = x * DualQuaternion.exp(0.5 * theta[i] * self.screws_0[i])

        return x * self.M

    def jacobian(self, theta):
        x = DualQuaternion(1, 0, 0, 0, 0, 0, 0, 0)
        J = np.zeros((8, len(theta)))

        for i in range(len(theta)):
            s_0 = self.screws_0[i]
            s_i = x * s_0 * x.inverse()
            x = x * DualQuaternion.exp(0.5 * theta[i] * s_0)
            J[:, i] = s_i.to_vector().flatten()

        return J

    def jacobian_body(self, theta):
        x = self.forward_kinematics(theta)
        J = self.jacobian(theta)
        Jb = x.inverse().as_matrix_left() @ x.as_matrix_right() @ J

        return np.vstack([Jb[1:4], Jb[5:8]])

    def jacobian6(self, theta):
        x = DualQuaternion(1, 0, 0, 0, 0, 0, 0, 0)
        J = np.zeros((6, len(theta)))

        for i in range(len(theta)):
            s_0 = self.screws_0[i]
            s_i = x * s_0 * x.inverse()
            x = x * DualQuaternion.exp(0.5 * theta[i] * s_0)
            J[:, i] = s_i.to_6vector().flatten()

        return J

    def jacobian_dot(self, theta, theta_dot):
        x = DualQuaternion(1, 0, 0, 0, 0, 0, 0, 0)
        x_dot = DualQuaternion(0, 0, 0, 0, 0, 0, 0, 0)
        J_dot = np.zeros((8, len(theta)))

        for i in range(len(theta)):
            s_0 = self.screws_0[i]
            s_i_dot = x_dot * s_0 * x.inverse() + x * s_0 * x_dot.inverse()
            x_temp = DualQuaternion.exp(0.5 * theta[i] * s_0)
            x_dot_temp = DualQuaternion.exp(0.5 * theta[i] * s_0) * (0.5 * theta_dot[i] * s_0)
            x_dot = x_dot * x_temp + x * x_dot_temp
            x = x * x_temp
            J_dot[:, i] = s_i_dot.to_vector().flatten()

        return J_dot

    def jacobian_dot6(self, theta, theta_dot):
        x = DualQuaternion(1, 0, 0, 0, 0, 0, 0, 0)
        x_dot = DualQuaternion(0, 0, 0, 0, 0, 0, 0, 0)
        J_dot = np.zeros((6, len(theta)))

        for i in range(len(theta)):
            s_0 = self.screws_0[i]
            s_i_dot = x_dot * s_0 * x.inverse() + x * s_0 * x_dot.inverse()
            x_temp = DualQuaternion.exp(0.5 * theta[i] * s_0)
            x_dot_temp = DualQuaternion.exp(0.5 * theta[i] * s_0) * (0.5 * theta_dot[i] * s_0)
            x_dot = x_dot * x_temp + x * x_dot_temp
            x = x * x_temp
            J_dot[:, i] = s_i_dot.to_6vector().flatten()

        return J_dot

    def hessian(self, theta):
        h = 0.000001
        j = self.jacobian_body(theta)
        n = j.shape[1]
        H = np.zeros((j.shape[0], j.shape[1], n))

        for i in range(n):
            theta_temp = theta.copy()
            theta_temp[i] += h
            j_temp = self.jacobian_body(theta_temp)
            H[:, :, i] = (j_temp - j) / h

        return H

