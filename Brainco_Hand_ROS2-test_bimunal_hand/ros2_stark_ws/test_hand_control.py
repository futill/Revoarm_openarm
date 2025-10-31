#!/usr/bin/env python3
"""
测试脚本：模拟 Vision Pro 发布手指控制命令
用于测试桥接节点和硬件控制是否正常工作

Usage:
    python3 test_hand_control.py
"""

import os
import time
import rclpy
from rclpy.node import Node
from ros2_stark_msgs.msg import SetMotorMulti

# 设置 ROS_DOMAIN_ID
os.environ['ROS_DOMAIN_ID'] = '5'


class TestHandControlPublisher(Node):
    """测试发布者节点"""
    
    def __init__(self):
        super().__init__('test_hand_control_publisher')
        
        # 创建发布者（与 Vision Pro 脚本相同的话题）
        self.left_hand_pub = self.create_publisher(
            SetMotorMulti, 
            '/left_hand/motor_commands', 
            10
        )
        self.right_hand_pub = self.create_publisher(
            SetMotorMulti, 
            '/right_hand/motor_commands', 
            10
        )
        
        self.get_logger().info('Test Hand Control Publisher initialized')
        self.get_logger().info(f'ROS_DOMAIN_ID: {os.environ.get("ROS_DOMAIN_ID", "not set")}')
    
    def publish_commands(self, left_positions, right_positions):
        """发布手指控制命令"""
        # 创建左手消息
        left_msg = SetMotorMulti()
        left_msg.mode = 1  # Position mode
        left_msg.positions = [int(pos) for pos in left_positions]
        left_msg.speeds = [0] * 6
        left_msg.currents = [0] * 6
        left_msg.pwms = [0] * 6
        left_msg.durations = [0] * 6
        
        # 创建右手消息
        right_msg = SetMotorMulti()
        right_msg.mode = 1  # Position mode
        right_msg.positions = [int(pos) for pos in right_positions]
        right_msg.speeds = [0] * 6
        right_msg.currents = [0] * 6
        right_msg.pwms = [0] * 6
        right_msg.durations = [0] * 6
        
        # 发布消息
        self.left_hand_pub.publish(left_msg)
        self.right_hand_pub.publish(right_msg)
        
        self.get_logger().info(f'Published - Left: {left_positions}, Right: {right_positions}')


def run_test_sequence(node):
    """运行测试序列"""
    print("\n" + "="*60)
    print("Hand Control Test Sequence")
    print("="*60)
    print("This script will test the following:")
    print("1. Open hands")
    print("2. Close hands (fist)")
    print("3. Individual finger test")
    print("4. Smooth motion test")
    print("="*60 + "\n")
    
    # 等待2秒让系统准备好
    print("Waiting 2 seconds for system to be ready...")
    time.sleep(1)
    
    # 测试1: 张开手
    print("\n[Test 1] Opening both hands...")
    open_positions = [400, 400, 0, 0, 0, 0]
    node.publish_commands(open_positions, open_positions)
    time.sleep(1)
    
    # 测试2: 握拳
    print("\n[Test 2] Closing both hands (fist)...")
    close_positions = [300, 300, 1000, 1000, 1000, 1000]
    node.publish_commands(close_positions, close_positions)
    time.sleep(1)
    
    # 测试3: 张开手
    print("\n[Test 3] Opening both hands again...")
    node.publish_commands(open_positions, open_positions)
    time.sleep(1)
    
    # 测试4: 单个手指测试 - 食指
    print("\n[Test 4] Testing index finger (motor 2)...")
    test_positions = [400, 400, 1000, 0, 0, 0]
    node.publish_commands(test_positions, test_positions)
    time.sleep(1.5)
    node.publish_commands(open_positions, open_positions)
    time.sleep(1.5)
    
    # 测试5: 单个手指测试 - 中指
    print("\n[Test 5] Testing middle finger (motor 3)...")
    test_positions = [400, 400, 0, 1000, 0, 0]
    node.publish_commands(test_positions, test_positions)
    time.sleep(1.5)
    node.publish_commands(open_positions, open_positions)
    time.sleep(1.5)
    
    # 测试6: 单个手指测试 - 无名指
    print("\n[Test 6] Testing ring finger (motor 4)...")
    test_positions = [400, 400, 0, 0, 1000, 0]
    node.publish_commands(test_positions, test_positions)
    time.sleep(1.5)
    node.publish_commands(open_positions, open_positions)
    time.sleep(1.5)
    
    # 测试7: 单个手指测试 - 小指
    print("\n[Test 7] Testing pinky finger (motor 5)...")
    test_positions = [400, 400, 0, 0, 0, 1000]
    node.publish_commands(test_positions, test_positions)
    time.sleep(1.5)
    node.publish_commands(open_positions, open_positions)
    time.sleep(1.5)
    
    # 测试8: 拇指测试
    print("\n[Test 8] Testing thumb (motor 0 and 1)...")
    test_positions = [300, 300, 0, 0, 0, 0]
    node.publish_commands(test_positions, test_positions)
    # time.sleep(1.5)
    node.publish_commands(open_positions, open_positions)
    # time.sleep(1.5)
    
    # 测试9: 平滑运动测试
    print("\n[Test 9] Smooth motion test (gradual close)...")
    steps = 10
    for i in range(steps + 1):
        progress = i / steps
        # 从张开 [400,400,0,0,0,0] 到握拳 [300,300,1000,1000,1000,1000]
        positions = [
            int(400 - 100 * progress),  # thumb adduct: 400 -> 300
            int(400 - 100 * progress),  # thumb flex: 400 -> 300
            int(0 + 1000 * progress),   # index: 0 -> 1000
            int(0 + 1000 * progress),   # middle: 0 -> 1000
            int(0 + 1000 * progress),   # ring: 0 -> 1000
            int(0 + 1000 * progress)    # pinky: 0 -> 1000
        ]
        node.publish_commands(positions, positions)
        #time.sleep(0.2)
    
    time.sleep(1)
    
    # 测试10: 平滑运动测试（反向）
    print("\n[Test 10] Smooth motion test (gradual open)...")
    for i in range(steps + 1):
        progress = i / steps
        # 从握拳 [300,300,1000,1000,1000,1000] 到张开 [400,400,0,0,0,0]
        positions = [
            int(300 + 100 * progress),  # thumb adduct: 300 -> 400
            int(300 + 100 * progress),  # thumb flex: 300 -> 400
            int(1000 - 1000 * progress), # index: 1000 -> 0
            int(1000 - 1000 * progress), # middle: 1000 -> 0
            int(1000 - 1000 * progress), # ring: 1000 -> 0
            int(1000 - 1000 * progress)  # pinky: 1000 -> 0
        ]
        node.publish_commands(positions, positions)
        #time.sleep(0.2)
    
    print("\n" + "="*60)
    print("Test sequence completed!")
    print("="*60 + "\n")


def main(args=None):
    """主函数"""
    # 初始化 ROS2
    rclpy.init(args=args)
    
    # 创建测试发布者节点
    test_node = TestHandControlPublisher()
    
    try:
        # 运行测试序列
        run_test_sequence(test_node)
        
        # 测试完成后，保持节点运行以便手动测试
        print("\nTest sequence finished. You can now:")
        print("1. Run your Vision Pro script")
        print("2. Or press Ctrl+C to exit")
        print("\nNode will keep running to maintain publishers...")
        
        rclpy.spin(test_node)
        
    except KeyboardInterrupt:
        print("\n\nShutting down test node...")
    finally:
        # 清理
        test_node.destroy_node()
        rclpy.shutdown()
        print("Test node shutdown complete")


if __name__ == "__main__":
    main()

