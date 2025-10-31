# ROS_DOMAIN_ID 与 sudo 环境变量问题记录

## 问题描述

在使用 `stark_serial_manager.sh` 脚本启动 ROS2 节点时，发现以下问题：

1. **环境变量传递失败**：通过 `export ROS_DOMAIN_ID=5` 设置的环境变量无法通过 `sudo` 正确传递给子进程
2. **节点运行在错误域**：节点实际运行在默认域（域0），而不是期望的域5
3. **话题不可见**：在域5中无法看到节点发布的话题和服务

## 问题原因

`sudo` 命令会重置大部分环境变量，只保留少数安全相关的变量。这导致通过 `export` 设置的环境变量无法传递给 `sudo` 启动的子进程。

## 解决方法

### 推荐方法：使用 sudo 前缀设置环境变量

```bash
# 启动节点在域5
sudo ROS_DOMAIN_ID=5 ./stark_serial_manager.sh launch

# 启动节点在域3
sudo ROS_DOMAIN_ID=3 ./stark_serial_manager.sh launch

# 启动节点在域10
sudo ROS_DOMAIN_ID=10 ./stark_serial_manager.sh launch
```

**验证方法**：
```bash
# 在对应的域中查看话题
export ROS_DOMAIN_ID=5
ros2 topic list
ros2 service list
```

## 如何查找ROS当前的域ID

### 方法1：检查当前环境变量

```bash
# 查看当前设置的域ID
echo $ROS_DOMAIN_ID

# 如果没有输出，说明使用默认域0
```

### 方法2：检查运行中进程的域ID

```bash
# 1. 找到stark_node进程ID
ps aux | grep stark_node | grep -v grep

# 2. 查看进程的环境变量（替换[PID]为实际进程ID）
sudo cat /proc/[PID]/environ | tr '\0' '\n' | grep ROS_DOMAIN_ID
```

### 方法3：通过话题可见性验证

```bash
# 在域5中查看话题
export ROS_DOMAIN_ID=5
ros2 topic list

# 在域3中查看话题
export ROS_DOMAIN_ID=3
ros2 topic list

# 如果节点在域5运行，在域3中就看不到话题
```

### 方法4：检查节点信息

```bash
# 查看节点列表（需要先设置正确的域ID）
export ROS_DOMAIN_ID=5
ros2 node list

# 查看服务列表
ros2 service list
```

## 实际使用示例

### 启动节点并验证

```bash
# 终端1：启动节点在域5
sudo ROS_DOMAIN_ID=5 ./stark_serial_manager.sh launch

# 终端2：在域5中查看话题
export ROS_DOMAIN_ID=5
ros2 topic list
# 应该能看到：/motor_status, /touch_status, /set_motor_multi_127, /set_motor_single_127

# 终端3：在域3中查看话题
export ROS_DOMAIN_ID=3
ros2 topic list
# 应该只能看到：/parameter_events, /rosout
```

### 测试不同域

```bash
# 启动节点在域3
sudo ROS_DOMAIN_ID=3 ./stark_serial_manager.sh launch

# 在域3中查看话题
export ROS_DOMAIN_ID=3
ros2 topic list
# 现在应该能看到stark相关的话题
```

## 常见问题

### Q1: 为什么 export 设置的环境变量不起作用？

A: `sudo` 会重置环境变量，只保留安全相关的变量。需要使用 `sudo VAR=value` 前缀语法。

### Q2: 如何确认节点运行在正确的域？

A: 检查进程的环境变量：
```bash
ps aux | grep stark_node | grep -v grep
sudo cat /proc/[PID]/environ | tr '\0' '\n' | grep ROS_DOMAIN_ID
```

### Q3: 节点启动成功但看不到话题？

A: 确保查看话题时使用相同的域：
```bash
export ROS_DOMAIN_ID=5
ros2 topic list
```

## 总结

- **问题**：`sudo` 会重置环境变量，导致 `export ROS_DOMAIN_ID=5` 不起作用
- **解决**：使用 `sudo ROS_DOMAIN_ID=5` 前缀语法
- **验证**：通过检查进程环境变量和话题可见性来确认域ID
