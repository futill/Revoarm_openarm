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



# moveit2
ros2 launch openarm_bimanual_moveit_config demo.launch.py hardware_type:=real left_can_interface:=can0 right_can_interface:=can1

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

# 关于重力补偿
ros2 run mujoco_openarm gravity_compensation


# Revoarm_openarm
