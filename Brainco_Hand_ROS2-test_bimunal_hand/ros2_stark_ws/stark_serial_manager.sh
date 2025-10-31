#!/bin/bash

# 不在当前目录时，切换到脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" # || exit 1

if [ "$1" == "launch" ]; then
    echo "=== Launch Stark Node ==="
fi    

# 设置环境变量
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu

# 清理 conda 相关变量
for var in $(env | grep -i conda | cut -d= -f1 2>/dev/null); do
    unset $var 2>/dev/null
done

# 设置 ROS 2 环境
source /opt/ros/humble/setup.bash

# 设置 ROS_DOMAIN_ID 环境变量（如果已设置）
if [ -n "$ROS_DOMAIN_ID" ]; then
    export ROS_DOMAIN_ID
    echo "Using ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
fi

if [ "$3" == "clean" ]; then
    rm -rf build install log
fi    

# 是否编译, 根据传入的参数
if [ "$2" == "build" ]; then
    echo "Building workspace..."
    colcon build --packages-select ros2_stark_msgs # 编译自定义消息包
    colcon build --packages-select ros2_stark_controller # 编译控制器节点 RS-485或CAN/CAN FD 版本
    if [ $? -ne 0 ]; then
        echo "Build failed. Exiting."
        exit 1
    fi
fi

# 设置工作空间
source install/setup.bash

# 运行
if [ "$1" == "launch" ]; then
    # 使用传入的ROS_DOMAIN_ID，如果没有设置则使用默认值5
    DOMAIN_ID=${ROS_DOMAIN_ID:-5}

    echo "Launching left hand node (stark_node_left)..."
    # 启动左手节点，并将其放入后台
    ROS_DOMAIN_ID=$DOMAIN_ID ros2 run ros2_stark_controller stark_node --ros-args \
        --params-file $SCRIPT_DIR/src/ros2_stark_controller/config/params_revo2_left.yaml \
        --remap __node:=stark_node_left &

    # 等待一小段时间，确保第一个节点有足够的时间初始化
    sleep 2

    echo "Launching right hand node (stark_node_right)..."
    # 启动右手节点，并将其放入后台
    ROS_DOMAIN_ID=$DOMAIN_ID ros2 run ros2_stark_controller stark_node --ros-args \
        --params-file $SCRIPT_DIR/src/ros2_stark_controller/config/params_revo2_right.yaml \
        --remap __node:=stark_node_right &

    echo "Both nodes are running. Press Ctrl+C to stop."

    # 等待所有后台任务完成 (这样脚本就不会立即退出)
    wait

elif [ "$1" == "monitor" ]; then
    # 测试监控关节信息
    ros2 topic echo /motor_status

elif [ "$1" == "monitor_touch" ]; then

    # 测试监控触觉信息
    ros2 topic echo /touch_status 

else
    echo "Available topics:"
    ros2 topic list
    echo ""
    echo "Available services:"
    ros2 service list
    echo ""
    echo "=========================================================="
    echo "Testing Stark controller..."
    echo "To test a specific hand, pass 'test' and the slave_id."
    echo "Example usage:"
    echo "  ./stark_serial_manager.sh test 126  # Test left hand (ID 0x7e)"
    echo "  ./stark_serial_manager.sh test 127  # Test right hand (ID 0x7f)"
    echo "=========================================================="

    if [ "$1" == "test" ] && [ -n "$2" ]; then
        echo "Running client for slave_id: $2"
        ros2 run ros2_stark_controller stark_node_client.py $2
    fi
fi