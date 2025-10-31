import numpy as np

from mujoco_openarm.lib.pin_module import PinSolver


class JntImpedance:
    def __init__(
            self,
            urdf_path: str,
    ):
        self.kd_solver = PinSolver(urdf_path)

        # hyperparameters of impedance controller
        self.B = 0.6 * np.ones(14)
        self.k = 1.0 * np.ones(14)
        self.scale = 0.9

    def compute_jnt_torque(self, q_des, v_des, q_cur, v_cur):
        """ robot的关节空间控制的计算公式
            Compute desired torque with robot dynamics modeling:
            > M(q)qdd + C(q, qd)qd + G(q) + tau_F(qd) = tau_ctrl + tau_env

        :param q_des: desired joint position
        :param v_des: desired joint velocity
        :param q_cur: current joint position
        :param v_cur: current joint velocity
        :return: desired joint torque
        """
        M = self.kd_solver.get_inertia_mat(q_cur)
        C = self.kd_solver.get_coriolis_mat(q_cur, v_cur)
        g = self.kd_solver.get_gravity_mat(q_cur)
        coriolis_gravity = C[-1] + g

        acc_desire = self.k * (q_des - q_cur) + self.B * (v_des - v_cur)
        tau_offset = np.array([
            0.00, -0.05, 0.00, 0.00, 0.00, 
            0.00, 0.06, 0.00, 0.00, 0.00,
            0.00, 0.00, 0.00, 0.00
        ]) 
        alpha = np.array([
            1.00, 0.98, 1.00, 1.00, 1.00,
            1.10, 1.10, 1.00, 1.00, 1.00,
            1.00, 1.00, 1.00, 1.00
        ])  # 每个关节的重力系数（示例）
        tau = np.dot(M, acc_desire) + alpha*coriolis_gravity
        return tau