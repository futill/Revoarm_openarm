#!/usr/bin/env python3
"""
Hand Control Bridge Node
订阅 Vision Pro 发布的手指控制命令，并转发到实际的灵巧手硬件控制话题

Usage:
    1. 先启动灵巧手控制器:
       ./stark_serial_manager.sh launch
    
    2. 在另一个终端运行此脚本:
       python3 hand_control_bridge.py
    
    3. 运行 Vision Pro 控制脚本
"""

import os
import rclpy
from rclpy.node import Node
from ros2_stark_msgs.msg import SetMotorMulti

# 设置 ROS_DOMAIN_ID (与 stark_serial_manager.sh 保持一致)
os.environ['ROS_DOMAIN_ID'] = '5'


class ExponentialMovingAverageFilter:
    """指数移动平均滤波器 - 用于平滑离散控制数据"""
    
    def __init__(self, alpha=0.3, num_channels=6):
        """
        初始化滤波器
        
        Args:
            alpha: 平滑系数 (0-1), 越小越平滑但响应越慢
                   建议: 0.1-0.3 (平滑) 或 0.5-0.7 (快速响应)
            num_channels: 通道数量（灵巧手自由度数量）
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
            # 第一次调用时，直接使用输入值初始化
            self.filtered_values = [float(v) for v in new_values[:self.num_channels]]
            self.initialized = True
            return [int(v) for v in self.filtered_values]
        
        # 应用指数移动平均: filtered = alpha * new + (1 - alpha) * filtered
        for i in range(min(len(new_values), self.num_channels)):
            self.filtered_values[i] = (
                self.alpha * float(new_values[i]) + 
                (1 - self.alpha) * self.filtered_values[i]
            )
        
        return [int(v) for v in self.filtered_values]
    
    def reset(self):
        """重置滤波器状态"""
        self.filtered_values = [0.0] * self.num_channels
        self.initialized = False


class HandControlBridge(Node):
    """桥接节点：订阅 Vision Pro 命令并转发到灵巧手硬件"""
    
    def __init__(self):
        super().__init__('hand_control_bridge')
        
        # 左手配置
        self.left_slave_id = 126  # 左手 ID (0x7e)
        self.right_slave_id = 127  # 右手 ID (0x7f)
        
        # 滤波器配置
        # alpha 参数: 0.1-0.3 (平滑但响应慢) 或 0.5-0.7 (快速响应)
        self.position_alpha = 0.4  # 位置滤波系数 - 相对平滑
        self.speed_alpha = 0.4      # 速度滤波系数 - 稍快响应
        self.current_alpha = 0.3    # 电流滤波系数
        
        # 为左手创建滤波器（每个参数类型一个）
        self.left_position_filter = ExponentialMovingAverageFilter(alpha=self.position_alpha)
        self.left_speed_filter = ExponentialMovingAverageFilter(alpha=self.speed_alpha)
        self.left_current_filter = ExponentialMovingAverageFilter(alpha=self.current_alpha)
        self.left_pwm_filter = ExponentialMovingAverageFilter(alpha=self.speed_alpha)
        self.left_duration_filter = ExponentialMovingAverageFilter(alpha=self.speed_alpha)
        
        # 为右手创建滤波器（每个参数类型一个）
        self.right_position_filter = ExponentialMovingAverageFilter(alpha=self.position_alpha)
        self.right_speed_filter = ExponentialMovingAverageFilter(alpha=self.speed_alpha)
        self.right_current_filter = ExponentialMovingAverageFilter(alpha=self.current_alpha)
        self.right_pwm_filter = ExponentialMovingAverageFilter(alpha=self.speed_alpha)
        self.right_duration_filter = ExponentialMovingAverageFilter(alpha=self.speed_alpha)
        
        # 创建订阅者 - 订阅 Vision Pro 发布的话题
        self.left_hand_sub = self.create_subscription(
            SetMotorMulti,
            '/left_hand/motor_commands',
            self.left_hand_callback,
            10
        )
        
        self.right_hand_sub = self.create_subscription(
            SetMotorMulti,
            '/right_hand/motor_commands',
            self.right_hand_callback,
            10
        )
        
        # 创建发布者 - 发布到实际的硬件控制话题
        self.left_hand_pub = self.create_publisher(
            SetMotorMulti,
            f'/set_motor_multi_{self.left_slave_id}',
            10
        )
        
        self.right_hand_pub = self.create_publisher(
            SetMotorMulti,
            f'/set_motor_multi_{self.right_slave_id}',
            10
        )
        
        # 统计信息
        self.left_count = 0
        self.right_count = 0
        
        # 创建定时器，定期打印统计信息
        self.stats_timer = self.create_timer(2.0, self.print_statistics)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Hand Control Bridge Node Started (with EMA Filtering)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'ROS_DOMAIN_ID: {os.environ.get("ROS_DOMAIN_ID", "not set")}')
        self.get_logger().info(f'Left hand:  Subscribe /left_hand/motor_commands  → Publish /set_motor_multi_{self.left_slave_id}')
        self.get_logger().info(f'Right hand: Subscribe /right_hand/motor_commands → Publish /set_motor_multi_{self.right_slave_id}')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Filter Config: Position α={self.position_alpha}, Speed α={self.speed_alpha}, Current α={self.current_alpha}')
        self.get_logger().info('  (α closer to 0 = smoother, closer to 1 = faster response)')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Waiting for commands from Vision Pro...')
    
    def left_hand_callback(self, msg):
        """左手命令回调函数（带滤波）"""
        # 准备输入数据（确保长度为6）
        positions = list(msg.positions[:6]) if len(msg.positions) >= 6 else list(msg.positions) + [0] * (6 - len(msg.positions))
        speeds = list(msg.speeds[:6]) if len(msg.speeds) >= 6 else list(msg.speeds) + [0] * (6 - len(msg.speeds))
        currents = list(msg.currents[:6]) if len(msg.currents) >= 6 else list(msg.currents) + [0] * (6 - len(msg.currents))
        pwms = list(msg.pwms[:6]) if len(msg.pwms) >= 6 else list(msg.pwms) + [0] * (6 - len(msg.pwms))
        durations = list(msg.durations[:6]) if len(msg.durations) >= 6 else list(msg.durations) + [0] * (6 - len(msg.durations))
        
        # 应用滤波器
        filtered_positions = self.left_position_filter.filter(positions)
        filtered_speeds = self.left_speed_filter.filter(speeds)
        filtered_currents = self.left_current_filter.filter(currents)
        filtered_pwms = self.left_pwm_filter.filter(pwms)
        filtered_durations = self.left_duration_filter.filter(durations)
        
        # 创建并填充转发消息
        forward_msg = SetMotorMulti()
        forward_msg.slave_id = self.left_slave_id
        forward_msg.mode = int(msg.mode)
        forward_msg.positions = filtered_positions
        forward_msg.speeds = filtered_speeds
        forward_msg.currents = filtered_currents
        forward_msg.pwms = filtered_pwms
        forward_msg.durations = filtered_durations
        
        # 发布到硬件控制话题
        self.left_hand_pub.publish(forward_msg)
        
        # 更新计数
        self.left_count += 1
        
        # 打印调试信息（可选，频率较高时可注释掉）
        if self.left_count % 10 == 0:  # 每10条消息打印一次
            self.get_logger().info(
                f'[LEFT]  #{self.left_count:04d} Mode={forward_msg.mode} SlaveID={forward_msg.slave_id} Pos={forward_msg.positions}'
            )
    
    def right_hand_callback(self, msg):
        """右手命令回调函数（带滤波）"""
        # 准备输入数据（确保长度为6）
        positions = list(msg.positions[:6]) if len(msg.positions) >= 6 else list(msg.positions) + [0] * (6 - len(msg.positions))
        speeds = list(msg.speeds[:6]) if len(msg.speeds) >= 6 else list(msg.speeds) + [0] * (6 - len(msg.speeds))
        currents = list(msg.currents[:6]) if len(msg.currents) >= 6 else list(msg.currents) + [0] * (6 - len(msg.currents))
        pwms = list(msg.pwms[:6]) if len(msg.pwms) >= 6 else list(msg.pwms) + [0] * (6 - len(msg.pwms))
        durations = list(msg.durations[:6]) if len(msg.durations) >= 6 else list(msg.durations) + [0] * (6 - len(msg.durations))
        
        # 应用滤波器
        filtered_positions = self.right_position_filter.filter(positions)
        filtered_speeds = self.right_speed_filter.filter(speeds)
        filtered_currents = self.right_current_filter.filter(currents)
        filtered_pwms = self.right_pwm_filter.filter(pwms)
        filtered_durations = self.right_duration_filter.filter(durations)
        
        # 创建并填充转发消息
        forward_msg = SetMotorMulti()
        forward_msg.slave_id = self.right_slave_id
        forward_msg.mode = int(msg.mode)
        forward_msg.positions = filtered_positions
        forward_msg.speeds = filtered_speeds
        forward_msg.currents = filtered_currents
        forward_msg.pwms = filtered_pwms
        forward_msg.durations = filtered_durations
        
        # 发布到硬件控制话题
        self.right_hand_pub.publish(forward_msg)
        
        # 更新计数
        self.right_count += 1
        
        # 打印调试信息（可选，频率较高时可注释掉）
        if self.right_count % 10 == 0:  # 每10条消息打印一次
            self.get_logger().info(
                f'[RIGHT] #{self.right_count:04d} Mode={forward_msg.mode} SlaveID={forward_msg.slave_id} Pos={forward_msg.positions}'
            )
    
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

