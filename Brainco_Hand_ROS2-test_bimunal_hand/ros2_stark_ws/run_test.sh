#!/bin/bash

# 测试脚本启动器
# 自动设置 ROS2 环境并运行测试脚本

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

# 设置 ROS_DOMAIN_ID
export ROS_DOMAIN_ID=5

echo "=========================================="
echo "Running Hand Control Test Script"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "=========================================="
echo ""

# 运行测试脚本
python3 test_hand_control.py

