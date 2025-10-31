# 🔧 Vision Pro 脚本修复方案

## 问题原因

错误 `[ERROR] Invalid mode: 0` 的原因是：你的 Vision Pro 脚本中创建消息时**没有设置 `slave_id` 字段**。

根据 `SetMotorMulti.msg` 定义，消息包含以下字段：
```
uint8 slave_id  # 必需字段！
uint8 mode
uint16[6] positions
int16[6] speeds
int16[6] currents
int16[6] pwms
uint16[6] durations
```

## ✅ 修复方法

在你的 `publish_hand_commands` 方法中添加 `slave_id` 字段：

### 修改前（当前代码）：
```python
def publish_hand_commands(self, left_positions, right_positions):
    """Publish hand control commands"""
    # Create left hand message
    left_msg = SetMotorMulti()
    left_msg.mode = 1  # Position mode
    left_msg.positions = [int(pos) for pos in left_positions]
    left_msg.speeds = [0] * 6
    left_msg.currents = [0] * 6
    left_msg.pwms = [0] * 6
    left_msg.durations = [0] * 6
    
    # Create right hand message
    right_msg = SetMotorMulti()
    right_msg.mode = 1  # Position mode
    right_msg.positions = [int(pos) for pos in right_positions]
    right_msg.speeds = [0] * 6
    right_msg.currents = [0] * 6
    right_msg.pwms = [0] * 6
    right_msg.durations = [0] * 6
    
    # Publish messages
    self.left_hand_pub.publish(left_msg)
    self.right_hand_pub.publish(right_msg)
```

### 修改后（添加 slave_id）：
```python
def publish_hand_commands(self, left_positions, right_positions):
    """Publish hand control commands"""
    # Create left hand message
    left_msg = SetMotorMulti()
    left_msg.slave_id = 0  # 桥接节点会将其改为 126
    left_msg.mode = 1  # Position mode
    left_msg.positions = [int(pos) for pos in left_positions]
    left_msg.speeds = [0] * 6
    left_msg.currents = [0] * 6
    left_msg.pwms = [0] * 6
    left_msg.durations = [0] * 6
    
    # Create right hand message
    right_msg = SetMotorMulti()
    right_msg.slave_id = 0  # 桥接节点会将其改为 127
    right_msg.mode = 1  # Position mode
    right_msg.positions = [int(pos) for pos in right_positions]
    right_msg.speeds = [0] * 6
    right_msg.currents = [0] * 6
    right_msg.pwms = [0] * 6
    right_msg.durations = [0] * 6
    
    # Publish messages
    self.left_hand_pub.publish(left_msg)
    self.right_hand_pub.publish(right_msg)
```

**说明**：
- 设置 `slave_id = 0` 作为占位符
- 桥接节点会将左手的 slave_id 改为 126，右手改为 127
- 这样消息就完整了，不会报 `Invalid mode: 0` 错误

## 🎯 完整的修复后的方法

```python
def publish_hand_commands(self, left_positions, right_positions):
    """Publish hand control commands"""
    # Create left hand message
    left_msg = SetMotorMulti()
    left_msg.slave_id = 0       # ✅ 添加这一行！
    left_msg.mode = 1
    left_msg.positions = [int(pos) for pos in left_positions]
    left_msg.speeds = [0] * 6
    left_msg.currents = [0] * 6
    left_msg.pwms = [0] * 6
    left_msg.durations = [0] * 6
    
    # Create right hand message
    right_msg = SetMotorMulti()
    right_msg.slave_id = 0      # ✅ 添加这一行！
    right_msg.mode = 1
    right_msg.positions = [int(pos) for pos in right_positions]
    right_msg.speeds = [0] * 6
    right_msg.currents = [0] * 6
    right_msg.pwms = [0] * 6
    right_msg.durations = [0] * 6
    
    # Publish messages
    self.left_hand_pub.publish(left_msg)
    self.right_hand_pub.publish(right_msg)
```

## 🧪 测试

修改后，重新运行：

1. 确保 `stark_serial_manager.sh launch` 在运行
2. 确保 `start_bridge.sh` 在运行
3. 运行修改后的 Vision Pro 脚本

现在应该不会再有 `Invalid mode: 0` 错误了！

## 💡 为什么会这样？

ROS2 消息的数值类型字段（如 `uint8 slave_id`）如果不初始化，默认值是 0。

当你没有设置 `slave_id` 时：
- `slave_id = 0`（默认值）
- 硬件控制节点检查 `slave_id == 0` 时认为是无效设备
- 导致后续的 `mode` 检查也失败，报错 `Invalid mode: 0`

所以**必须显式设置 `slave_id`**，即使是占位符值。

## ✨ 完成！

只需添加两行代码：
```python
left_msg.slave_id = 0
right_msg.slave_id = 0
```

就可以解决问题了！

