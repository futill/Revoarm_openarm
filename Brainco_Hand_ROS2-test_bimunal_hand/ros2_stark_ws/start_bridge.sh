#!/bin/bash

# 启动手势控制桥接节点
# 此脚本需要在 stark_serial_manager.sh launch 之后运行

# 不在当前目录时，切换到脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 设置环境变量
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu

# 清理 conda 相关变量
for var in $(env | grep -i conda | cut -d= -f1 2>/dev/null); do
    unset $var 2>/dev/null
done

# 设置 ROS 2 环境
source /opt/ros/humble/setup.bash

# 设置工作空间
source install/setup.bash

# 设置 ROS_DOMAIN_ID (与 stark_serial_manager.sh 保持一致)
export ROS_DOMAIN_ID=5

echo "=========================================="
echo "Starting Hand Control Bridge Node"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "=========================================="
echo ""
echo "This bridge will:"
echo "  1. Subscribe to /left_hand/motor_commands"
echo "  2. Subscribe to /right_hand/motor_commands"
echo "  3. Forward commands to /set_motor_multi_126 (left)"
echo "  4. Forward commands to /set_motor_multi_127 (right)"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="
echo ""

# 运行桥接节点
python3 hand_control_bridge.py

