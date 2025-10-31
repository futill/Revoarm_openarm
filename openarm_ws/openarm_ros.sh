source /opt/ros/humble/setup.bash
source install/setup.bash
if [ "$EUID" -ne 0 ]; then
    echo "Root privileges are required to configure CAN interfaces."
    echo "Re-running with sudo..."
    exec sudo bash "$0" "$@"
fi

# 配置 CAN 接口
ip link set can0 down
ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
ip link set can0 up

ip link set can1 down
ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
ip link set can1 up

