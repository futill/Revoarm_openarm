import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from openarm_msgs.msg import ArmJoints


import mujoco
from mujoco import viewer
import numpy as np
from mujoco_openarm.lib.jnt_imp_controller import JntImpedance


class RobotMujocoNode(Node):
    ACTUATORS = [
        'left_joint1_ctrl', 'left_joint2_ctrl', 'left_joint3_ctrl', 'left_joint4_ctrl',
        'left_joint5_ctrl', 'left_joint6_ctrl', 'left_joint7_ctrl',
        'right_joint1_ctrl', 'right_joint2_ctrl', 'right_joint3_ctrl', 'right_joint4_ctrl',
        'right_joint5_ctrl', 'right_joint6_ctrl', 'right_joint7_ctrl',
    ]

    def __init__(self):
        super().__init__('robot_mujoco_node')
        self.get_logger().info("启动 MuJoCo 仿真节点...")

        # MuJoCo 初始化
        self.mj_model = mujoco.MjModel.from_xml_path(
            '~/openarm/openarm_mujoco/v1/openarm_bimanual.xml')
        self.mj_data = mujoco.MjData(self.mj_model)
        self.viewer = viewer.launch_passive(self.mj_model, self.mj_data)

        self.controller = JntImpedance(
            urdf_path='~/openarm/openarm_mujoco/v1/openarm_bimanual.urdf')

        self.joint_name_to_id = {mujoco.mj_id2name(self.mj_model, mujoco.mjtObj.mjOBJ_JOINT, i): i
                                 for i in range(self.mj_model.njnt)}

        # ROS2 订阅 /joint_states
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # ROS2 发布扭矩和位置
        self.left_arm_effort_topic_pub = self.create_publisher(ArmJoints, '/left_arm_effort/motor_commands', 10)
        self.right_arm_effort_topic_pub = self.create_publisher(ArmJoints, '/right_arm_effort/motor_commands', 10)
        self.left_arm_position_topic_pub = self.create_publisher(ArmJoints, '/left_arm_postion/motor_commands', 10)
        self.right_arm_position_topic_pub = self.create_publisher(ArmJoints, '/right_arm_postion/motor_commands', 10)


        self.timer_period = 0.01
        self.timer = self.create_timer(self.timer_period, self.update_simulation)
        self.latest_joint_state = None

        # effort限幅
        self.MAX_TORQUE = 5.0
        self.MIN_TORQUE = -5.0

    def joint_state_callback(self, msg: JointState):
        if not msg.name:
            return
        for name,  pos , vel in zip(msg.name, msg.position , msg.velocity):
            if name == 'openarm_left_finger_joint1' or name == 'openarm_right_finger_joint1':
                continue
            elif name in self.joint_name_to_id:
                idx = self.joint_name_to_id[name]
                if name == 'left_joint7_ctrl':
                    self.mj_data.qpos[idx] = -pos
                    self.mj_data.qvel[idx] =  -vel
                else:
                    self.mj_data.qpos[idx] =  pos
                    self.mj_data.qvel[idx] =  vel
            else:
                self.get_logger().warn_once(f"未找到匹配的关节: {name}")
        self.latest_joint_state = msg

    def update_simulation(self):
        if self.latest_joint_state is not None:
            # 计算阻抗扭矩
            torque = self.controller.compute_jnt_torque(
                q_des=np.array(self.mj_data.qpos),
                v_des=np.zeros_like(self.mj_data.qvel),
                q_cur=self.mj_data.qpos,
                v_cur=self.mj_data.qvel,
            )
            torque = np.clip(torque, self.MIN_TORQUE, self.MAX_TORQUE)

            # 应用到 MuJoCo actuators
            for j, actuator in enumerate(self.ACTUATORS):
                self.mj_data.actuator(j).ctrl = torque[j]

            # -----------------------------
            # 发布到滤波话题
            # -----------------------------
            left_effort_data = torque[:7].tolist()
            left_effort_data[6] = -left_effort_data[6]  # 第7关节取反

            right_effort_data = torque[7:14].tolist()
            right_effort_data[6] = -right_effort_data[6]  # 第7关节取反

            left_pos_data = self.mj_data.qpos[:7].tolist()
            left_pos_data[6] = -left_pos_data[6]

            right_pos_data = self.mj_data.qpos[7:14].tolist()
            right_pos_data[6] = -right_pos_data[6]

            # 左臂消息
            left_msg = ArmJoints()
            left_msg.positions = left_pos_data
            left_msg.velocities = self.mj_data.qvel[:7].tolist()
            left_msg.efforts = left_effort_data
            left_msg.arm_name = "left_arm"

            self.left_arm_effort_topic_pub.publish(left_msg)
            self.left_arm_position_topic_pub.publish(left_msg)

            # 右臂消息
            right_msg = ArmJoints()
            right_msg.positions = right_pos_data
            right_msg.velocities = self.mj_data.qvel[7:14].tolist()
            right_msg.efforts = right_effort_data
            right_msg.arm_name = "right_arm"

            self.right_arm_effort_topic_pub.publish(right_msg)
            self.right_arm_position_topic_pub.publish(right_msg)

        mujoco.mj_step(self.mj_model, self.mj_data)
        if self.viewer.is_running():
            self.viewer.sync()


def main(args=None):
    rclpy.init(args=args)
    node = RobotMujocoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
