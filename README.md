# OpenArm 双臂机器人系统 README

> **OpenArm** 是一款开源、高性能、双臂人形机械臂，专为 **物理 AI 研究、遥操作、模仿学习、接触密集任务** 设计。  
> 本项目基于 **ROS 2 Humble**，集成 **MoveIt 2**、**CAN FD 通信**、**力位混控** 与 **MuJoCo 重力补偿**，支持 **真实硬件** 与 **仿真** 无缝切换。

---

## 项目结构
~/openarm/  
├── Brainco_Hand_ROS2-test_bimunal_hand/        # revo2灵巧手  
├── openarm_mujoco/        #openarm仿真  
├── openarm_can/        # CAN 总线底层驱动（C++）  
└── openarm_ws/         # ROS 2 工作空间（MoveIt、控制器、桥接脚本  
---

## 系统要求

| 项目       | 要求 |
|------------|------|
| 操作系统   | Ubuntu 22.04 LTS |
| ROS 2      | Humble Hawksbill |
| 硬件接口   | 2× USB-CAN FD 适配器（`can0` 左臂，`can1` 右臂） |
| 内核模块   | `can`, `can_raw`, `can_dev` |

---


# install
sudo apt install ros-humble-moveit*  

sudo apt install ros-humble-ros2-controllers ros-humble-effort-controllers  

cd ~/openarm/openarm_can  

mkdir build  

cmake ..  

make -j8  

sudo install make  


cd ~/openarm/openarm_ws  
colcon build  
source install/setup.bash  

---

# test
sudo ip link set can0 down  
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on  
sudo ip link set can0 up  

sudo ip link set can1 down  
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on  
sudo ip link set can1 up  

cd openarm/openarm_can/build/  
./openarm_can0_demo  
./openarm_can1_demo  

---

# moveit2
ros2 launch openarm_bimanual_moveit_config demo.launch.py hardware_type:=real   left_can_interface:=can0 right_can_interface:=can1  

---

# joint信息订阅
对/joint_states进行订阅可得到关节的pos，vel,effort信息  
- openarm_right_joint1
- openarm_right_joint3
- openarm_left_finger_joint1
- openarm_right_joint4
- openarm_left_joint6
- openarm_left_joint5
- openarm_left_joint2
- openarm_left_joint7
- openarm_left_joint4
- openarm_right_joint5
- openarm_left_joint1
- openarm_right_finger_joint1
- openarm_right_joint2
- openarm_left_joint3
- openarm_right_joint7
- openarm_right_joint6
进行订阅要重新进行信息对齐
---

# 启动流程
# openarm_effort_postion API(开两个终端)
cd ~/openarm/openarm_ws  
./openarm_ros.sh  
ros2 launch openarm_bringup openarm.bimanual.launch.py   
python3 openarm_brigde.py  
# effort_topic 
/right_arm_effort/motor_commands  
/left_arm_effort/motor_commands  
# postion_topic
/right_arm_postion/motor_commands  
/left_arm_postion/motor_commands  
# 进行力位混控只需要同时对effort和postion话题进行订阅并发送data
# 同理只对position进行订阅发送data，进行位置控制

**话题消息类型**
# ArmJoints.msg
# 7关节机械臂的关节角度、速度、力矩等信息
float64[7] positions     # 7个关节的角度  
float64[7] velocities    # 7个关节的速度  
float64[7] efforts       # 7个关节的力矩  
string arm_name          # 机械臂名称  
**示例**
left_msg = ArmJoints()  
left_msg.positions = left_pos_data  
left_msg.efforts = left_effort_data  
left_msg.arm_name = "left_arm"  

self.left_arm_effort_topic_pub.publish(left_msg)  
self.left_arm_position_topic_pub.publish(left_msg)  

---

# 关于重力补偿
**前提条件**
-pip install pin
-pip install mujoco
-到mujoco仓库下载源码编译

**启动**
ros2 run mujoco_openarm gravity_compensation

---

# Revoarm_openarm
