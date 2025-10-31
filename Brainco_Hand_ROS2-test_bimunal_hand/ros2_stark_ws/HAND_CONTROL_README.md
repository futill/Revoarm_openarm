# Vision Pro 手势控制灵巧手使用说明

## 系统架构

```
Vision Pro 脚本
    ↓ 发布
/left_hand/motor_commands
/right_hand/motor_commands
    ↓ 订阅
Hand Control Bridge (桥接节点)
    ↓ 转发
/set_motor_multi_126 (左手)
/set_motor_multi_127 (右手)
    ↓ 订阅
Stark Node (硬件控制节点)
    ↓ 控制
灵巧手硬件 (左手: 126, 右手: 127)
```

## 使用步骤

### 1. 启动灵巧手硬件控制节点

在第一个终端中运行：

```bash
cd /home/scy/stark-serialport-example/ros2_stark_ws
./stark_serial_manager.sh launch
```

这将启动：
- `stark_node_left`: 控制左手 (slave_id=126, /dev/ttyUSB0)
- `stark_node_right`: 控制右手 (slave_id=127, /dev/ttyUSB1)

### 2. 启动桥接节点

在第二个终端中运行：

```bash
cd /home/scy/stark-serialport-example/ros2_stark_ws
./start_bridge.sh
```

或者直接运行 Python 脚本：

```bash
cd /home/scy/stark-serialport-example/ros2_stark_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5
python3 hand_control_bridge.py
```

### 3. 运行 Vision Pro 控制脚本

在第三个终端中，先设置环境再运行你的 Vision Pro 控制脚本：

```bash
cd /home/scy/stark-serialport-example/ros2_stark_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5
python3 your_visionpro_script.py
```

## 测试方法

### 测试桥接功能

**方法1: 使用测试脚本（推荐）**

```bash
cd /home/scy/stark-serialport-example/ros2_stark_ws
./run_test.sh
```

这将自动运行一系列测试：张开、握拳、单个手指测试、平滑运动等。

**方法2: 手动发布消息**

在启动桥接节点后，可以手动发布消息测试：

```bash
# 测试左手
ros2 topic pub --once /left_hand/motor_commands ros2_stark_msgs/msg/SetMotorMulti '{
  mode: 1, 
  positions: [300, 300, 1000, 1000, 1000, 1000]
}'

# 测试右手
ros2 topic pub --once /right_hand/motor_commands ros2_stark_msgs/msg/SetMotorMulti '{
  mode: 1, 
  positions: [300, 300, 1000, 1000, 1000, 1000]
}'
```

### 查看话题列表

```bash
ros2 topic list
```

应该能看到以下话题：
- `/left_hand/motor_commands`
- `/right_hand/motor_commands`
- `/set_motor_multi_126`
- `/set_motor_multi_127`

### 监控话题数据

```bash
# 监控 Vision Pro 发布的命令
ros2 topic echo /left_hand/motor_commands
ros2 topic echo /right_hand/motor_commands

# 监控发送到硬件的命令
ros2 topic echo /set_motor_multi_126
ros2 topic echo /set_motor_multi_127
```

## 配置说明

### ROS_DOMAIN_ID

所有终端都需要使用相同的 `ROS_DOMAIN_ID=5`。如果需要修改，请同时修改：
- `stark_serial_manager.sh` (第50行)
- `start_bridge.sh` (第24行)
- `hand_control_bridge.py` (第20行)
- Vision Pro 控制脚本 (第10行)

### Slave ID 映射

- 左手: slave_id = 126 (0x7e)
- 右手: slave_id = 127 (0x7f)

如需修改，请编辑 `hand_control_bridge.py` 中的配置：
```python
self.left_slave_id = 126
self.right_slave_id = 127
```

### 串口配置

- 左手: `/dev/ttyUSB0` (在 `params_revo2_left.yaml` 中配置)
- 右手: `/dev/ttyUSB1` (在 `params_revo2_right.yaml` 中配置)

### 手指位置参数范围

根据你的 Vision Pro 脚本注释：
- idx 0: thumb adduct range [200, 430] → 映射到 [0, 1000]
- idx 1: thumb flex range [390, 1000] → 映射到 [0, 1000]
- idx 2: index flex range [550, 900] → 映射到 [0, 1000]
- idx 3: middle flex range [550, 900] → 映射到 [0, 1000]
- idx 4: ring flex range [550, 900] → 映射到 [0, 1000]
- idx 5: pinky flex range [550, 900] → 映射到 [0, 1000]

## 故障排查

### 1. 桥接节点收不到 Vision Pro 数据

检查：
- ROS_DOMAIN_ID 是否一致
- Vision Pro 脚本是否正常运行
- 使用 `ros2 topic list` 检查话题是否存在
- 使用 `ros2 topic echo /left_hand/motor_commands` 检查数据

### 2. 灵巧手不动

检查：
- `stark_serial_manager.sh launch` 是否正常运行
- 串口设备是否正确 (`/dev/ttyUSB0`, `/dev/ttyUSB1`)
- 使用 `ros2 topic echo /set_motor_multi_126` 检查命令是否发送

### 3. 串口权限问题

```bash
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

或者将用户添加到 dialout 组：
```bash
sudo usermod -aG dialout $USER
# 需要重新登录
```

## 文件说明

- `hand_control_bridge.py`: 桥接节点主程序
- `start_bridge.sh`: 桥接节点启动脚本
- `test_hand_control.py`: 测试脚本主程序
- `run_test.sh`: 测试脚本启动器（自动设置环境）
- `stark_serial_manager.sh`: 硬件控制节点启动脚本
- `params_revo2_left.yaml`: 左手配置文件
- `params_revo2_right.yaml`: 右手配置文件

## 性能优化

如果觉得打印信息太多，可以修改 `hand_control_bridge.py`：
- 注释掉第 71-74 行和第 97-100 行的调试输出
- 或者调整打印频率（例如改为每 50 条消息打印一次）

## 联系信息

如有问题，请检查日志输出或联系技术支持。

