# ⚠️ 重要修正：包名错误

## 问题说明

你的 Vision Pro 脚本中使用了**错误的包名**，导致无法订阅话题。

### 错误现象
```bash
ros2 topic echo /left_hand/motor_commands 
The message type 'ros2_stark_interfaces/msg/SetMotorMulti' is invalid
```

## 🔧 解决方法

### 你的 Vision Pro 脚本中需要修改的地方

**❌ 错误的导入（当前）：**
```python
from ros2_stark_interfaces.msg import SetMotorMulti
```

**✅ 正确的导入（需要改成）：**
```python
from ros2_stark_msgs.msg import SetMotorMulti
```

---

## 📝 完整修正步骤

在你的 Vision Pro 控制脚本中：

### 第 1 处修改：导入语句

找到这一行：
```python
from ros2_stark_interfaces.msg import SetMotorMulti
```

改为：
```python
from ros2_stark_msgs.msg import SetMotorMulti
```

### 验证包名

在终端中运行以下命令确认正确的包名：

```bash
# 查看已安装的包
ros2 pkg list | grep stark

# 输出应该包含：
# ros2_stark_controller
# ros2_stark_msgs          <--- 这是正确的包名！
```

```bash
# 查看包中的消息类型
ros2 interface show ros2_stark_msgs/msg/SetMotorMulti

# 应该显示消息定义
```

---

## 🧪 测试修复

修改后，在终端中测试：

```bash
# 1. 设置环境
cd /home/scy/stark-serialport-example/ros2_stark_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5

# 2. 测试订阅（应该能正常工作）
ros2 topic echo /left_hand/motor_commands

# 3. 在另一个终端发布测试数据
ros2 topic pub --once /left_hand/motor_commands \
  ros2_stark_msgs/msg/SetMotorMulti \
  '{mode: 1, positions: [300, 300, 1000, 1000, 1000, 1000]}'
```

如果修复成功，`ros2 topic echo` 应该能显示接收到的数据。

---

## ✅ 正确的 Vision Pro 脚本模板

```python
#!/usr/bin/env python3
import numpy as np
import yaml
import os
from pathlib import Path
from dex_retargeting.retargeting_config import RetargetingConfig
from avp_stream import VisionProStreamer
from scipy.spatial.transform import Rotation as R

# Set ROS_DOMAIN_ID
os.environ['ROS_DOMAIN_ID'] = '5'

import rclpy
from rclpy.node import Node
from ros2_stark_msgs.msg import SetMotorMulti  # ✅ 正确的包名

# ... 其余代码保持不变 ...
```

---

## 📋 包名对照表

| 错误包名 (你的脚本) | 正确包名 (系统实际) |
|-------------------|-------------------|
| ros2_stark_interfaces | ros2_stark_msgs |

---

## 🔍 为什么会这样？

在 ROS2 工作空间中，包名是在 `package.xml` 中定义的。你的系统中：

- **实际包名**: `ros2_stark_msgs`（在 `src/ros2_stark_msgs/package.xml` 中定义）
- **你使用的包名**: `ros2_stark_interfaces`（不存在）

这就是为什么能看到话题（因为话题名称是正确的），但无法订阅（因为消息类型引用了不存在的包）。

---

## ⚡ 快速修复命令

如果你想快速验证，可以用 Python 直接测试：

```bash
cd /home/scy/stark-serialport-example/ros2_stark_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5

python3 -c "from ros2_stark_msgs.msg import SetMotorMulti; print('✅ Import successful!')"
```

应该输出：`✅ Import successful!`

如果尝试错误的包名：
```bash
python3 -c "from ros2_stark_interfaces.msg import SetMotorMulti; print('Import successful!')"
```

会报错：`ModuleNotFoundError: No module named 'ros2_stark_interfaces'`

---

## 📁 参考文件

我创建的所有脚本都使用了**正确的包名** `ros2_stark_msgs`：

- ✅ `hand_control_bridge.py` - 使用 `ros2_stark_msgs`
- ✅ `test_hand_control.py` - 使用 `ros2_stark_msgs`
- ✅ `stark_node_client.py` - 使用 `ros2_stark_msgs`

你可以参考这些文件中的导入语句。

---

## 🎯 总结

**只需要修改一行代码：**

```python
# 把这行：
from ros2_stark_interfaces.msg import SetMotorMulti

# 改成这行：
from ros2_stark_msgs.msg import SetMotorMulti
```

修改后重新运行你的 Vision Pro 脚本即可！

