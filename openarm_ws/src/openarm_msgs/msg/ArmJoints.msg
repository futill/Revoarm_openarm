# ArmJoints.msg
# 7关节机械臂的关节角度、速度、力矩等信息

float64[7] positions     # 7个关节的角度
float64[7] velocities    # 7个关节的速度
float64[7] efforts       # 7个关节的力矩
string arm_name          # 机械臂名称