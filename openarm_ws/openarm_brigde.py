import os
import rclpy
from rclpy.node import Node 
from openarm_msgs.msg import ArmJoints
from std_msgs.msg import Float64MultiArray

class ExponentialMovingAverageFilter:
    def __init__(self, alpha=0.3, num_channels=7):
        """
        初始化滤波器
        
        Args:
            alpha: 平滑系数 (0-1), 越小越平滑但响应越慢
                   建议: 0.1-0.3 (平滑) 或 0.5-0.7 (快速响应)
            num_channels: 通道数量（关节数量）
        """
        self.alpha = alpha
        self.num_channels = num_channels
        self.filtered_values = [0.0] * num_channels
        self.initialized = False
    
    def filter(self, new_values):
        """
        应用滤波器
        
        Args:
            new_values: 新的输入值列表
        
        Returns:
            filtered_values: 滤波后的值列表
        """
        if not self.initialized:
            self.filtered_values = [float(v) for v in new_values[:self.num_channels]]
            self.initialized = True
            return [float(v) for v in self.filtered_values]

        for i in range(min(len(new_values), self.num_channels)):
            self.filtered_values[i] = (
                self.alpha * float(new_values[i]) + 
                (1 - self.alpha) * self.filtered_values[i]
            )
        return [float(v) for v in self.filtered_values]
    
    def reset(self):
        """重置滤波器状态"""
        self.filtered_values = [0.0] * self.num_channels
        self.initialized = False


class HandControlBridge(Node):
    """桥接节点：订阅 Vision Pro 命令并转发到灵巧手硬件"""
    
    def __init__(self):
        super().__init__('hand_control_bridge')
        
        # 滤波器配置
        # alpha 参数: 0.1-0.3 (平滑但响应慢) 或 0.5-0.7 (快速响应)
        self.position_alpha = 0.4  # 位置滤波系数 - 相对平滑
        self.efforts_alpha = 0.4
        
        # 为左手创建滤波器（每个参数类型一个）
        self.left_position_filter = ExponentialMovingAverageFilter(alpha=self.position_alpha)
        self.left_efforts_filter = ExponentialMovingAverageFilter(alpha=self.efforts_alpha)
        
        # 为右手创建滤波器（每个参数类型一个）
        self.right_position_filter = ExponentialMovingAverageFilter(alpha=self.position_alpha)
        self.right_efforts_filter = ExponentialMovingAverageFilter(alpha=self.efforts_alpha)
        
        # 创建订阅者 - 订阅 Vision Pro 发布的话题
        self.left_arm_effort_sub = self.create_subscription(
            ArmJoints,
            '/left_arm_effort/motor_commands',
            self.left_arm_effort_callback,
            10
        )

        self.left_arm_pos_sub = self.create_subscription(
            ArmJoints,
            '/left_arm_postion/motor_commands',
            self.left_arm_postion_callback,
            10
        )

        self.right_arm_effort_sub = self.create_subscription(
            ArmJoints,
            '/right_arm_effort/motor_commands',
            self.right_arm_effort_callback,
            10
        )

        self.right_arm_pos_sub = self.create_subscription(
            ArmJoints,
            '/right_arm_postion/motor_commands',
            self.right_arm_postion_callback,
            10
        )

        # 创建发布者 - 发布到实际的硬件控制话题
        self.left_effort_pub = self.create_publisher(Float64MultiArray, '/left_effort_controller/commands', 10)
        self.right_effort_pub = self.create_publisher(Float64MultiArray, '/right_effort_controller/commands', 10)
        self.left_pos_pub    = self.create_publisher(Float64MultiArray, '/left_forward_position_controller/commands', 10)
        self.right_pos_pub   = self.create_publisher(Float64MultiArray, '/right_forward_position_controller/commands', 10)
    
        # 统计信息
        self.left_count = 0
        self.right_count = 0
        
        # 创建定时器，定期打印统计信息
        self.stats_timer = self.create_timer(2.0, self.print_statistics)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('openarm Control Bridge Node Started (with EMA Filtering)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Left arm eff:  Subscribe /left_arm_effort/motor_commands  → Publish /left_effort_controller/commands')
        self.get_logger().info(f'rigth arm eff:  Subscribe /right_arm_effort/motor_commands  → Publish /right_effort_controller/commands')
        self.get_logger().info(f'Left arm pos: Subscribe /left_arm_postion/motor_commands → Publish /left_forward_position_controller/commands')
        self.get_logger().info(f'Right arm pos: Subscribe /right_arm_postion/motor_commands → Publish /right_forward_position_controller/commands')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Filter Config: Position α={self.position_alpha}, torpue α={self.efforts_alpha}')
        self.get_logger().info('  (α closer to 0 = smoother, closer to 1 = faster response)')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waiting for commands from Vision Pro...')

    def left_arm_postion_callback(self, msg):
        """左臂位置命令回调函数（带滤波）"""
        # 准备输入数据（确保长度为6）
        positions = list(msg.positions[:7]) if len(msg.positions) >= 7 else list(msg.positions) + [0] * (7 - len(msg.positions))
        # 应用滤波器
        filtered_positions = self.left_position_filter.filter(positions)
        # 创建并填充转发消息
        left_pos_msg  = Float64MultiArray()
        left_pos_msg.data = filtered_positions
        # 发布到硬件控制话题
        self.left_pos_pub.publish(left_pos_msg)

        # 更新计数
        self.left_count += 1
        
        # 打印调试信息（可选，频率较高时可注释掉）
        # if self.left_count % 10 == 0:  # 每10条消息打印一次
        #     self.get_logger().info(
        #         f'[LEFT_POS]  #{self.left_count:04d}Pos={left_pos_msg.data }'
        #     )

    def right_arm_postion_callback(self, msg):
        """右臂位置命令回调函数（带滤波）"""
        # 准备输入数据（确保长度为6）
        positions = list(msg.positions[:7]) if len(msg.positions) >= 7 else list(msg.positions) + [0] * (7 - len(msg.positions))
        # 应用滤波器
        filtered_positions = self.right_position_filter.filter(positions)
        # 创建并填充转发消息
        right_pos_msg = Float64MultiArray()
        right_pos_msg.data = filtered_positions
        # 发布到硬件控制话题
        self.right_pos_pub.publish(right_pos_msg)

        # 更新计数
        self.right_count += 1
        
        # 打印调试信息（可选，频率较高时可注释掉）
        # if self.right_count % 10 == 0:  # 每10条消息打印一次
        #     self.get_logger().info(
        #         f'[RIGHT_POS]  #{self.left_count:04d} Pos={right_pos_msg.data}'
        #     )


    def left_arm_effort_callback(self, msg):
        """左臂力矩命令回调函数（带滤波）"""
        # 准备输入数据（确保长度为6）
        efforts = list(msg.efforts[:7]) if len(msg.efforts) >= 7 else list(msg.efforts) + [0] * (7 - len(msg.efforts))
        # 应用滤波器
        filtered_efforts = self.left_efforts_filter.filter(efforts)
        # 创建并填充转发消息
        left_tor_msg  = Float64MultiArray()
        left_tor_msg.data = filtered_efforts
        # 发布到硬件控制话题
        self.left_effort_pub.publish(left_tor_msg)

        # 更新计数
        self.left_count += 1
        
        # 打印调试信息（可选，频率较高时可注释掉）
        # if self.left_count % 10 == 0:  # 每10条消息打印一次
        #     self.get_logger().info(
        #         f'[LEFT_EFF]  #{self.left_count:04d} TOR={left_tor_msg.data}'
        #     )
            
    def right_arm_effort_callback(self, msg):
        """右臂力矩命令回调函数（带滤波）"""
        # 准备输入数据（确保长度为6）
        efforts = list(msg.efforts[:7]) if len(msg.efforts) >= 7 else list(msg.efforts) + [0] * (7 - len(msg.efforts))
        # 应用滤波器
        filtered_efforts = self.right_efforts_filter.filter(efforts)
        # 创建并填充转发消息
        right_tor_msg = Float64MultiArray()
        right_tor_msg.data = filtered_efforts
        # 发布到硬件控制话题
        self.right_effort_pub.publish(right_tor_msg)

        # 更新计数
        self.right_count += 1
        
        # 打印调试信息（可选，频率较高时可注释掉）
        # if self.right_count % 10 == 0:  # 每10条消息打印一次
        #     self.get_logger().info(
        #         f'[RIGHT_EFF]  #{self.left_count:04d} Pos={right_tor_msg.data}'
        #     )

    def print_statistics(self):
        """定期打印统计信息"""
        self.get_logger().info(
            f'Stats: Left={self.left_count} msgs, Right={self.right_count} msgs'
        )

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    bridge_node = HandControlBridge()
    
    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        print("\n\nShutting down Hand Control Bridge...")
    finally:
        bridge_node.get_logger().info('Hand Control Bridge shutting down')
        bridge_node.get_logger().info(f'Final stats: Left={bridge_node.left_count} msgs, Right={bridge_node.right_count} msgs')
        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
