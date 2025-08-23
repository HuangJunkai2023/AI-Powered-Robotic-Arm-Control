
### Using MoveIt / 使用 MoveIt

You can also run the MoveIt example and use the `MotionPlanning` plugin in RViZ to start planning:
您也可以运行 MoveIt 示例，并在 RViZ 中使用 `MotionPlanning` 插件开始规划：

```bash
# 设置库路径并启动真实机器人的 MoveIt（需要真实机器人IP）
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/aubo_ros2_ws/install/aubo_ros2_driver/lib/aubo_ros2_driver/aubocontroller
ros2 launch aubo_ros2_moveit_config aubo_moveit.launch.py robot_ip:=[robot ip]
# 启动机器人电源
ros2 topic pub --once /aubo_robot/robot_control std_msgs/msg/String 'data: powerOn'
```

Test sim / 仿真测试:
```bash
# 启动 MoveIt 仿真环境（无需真实机器人）
ros2 launch  aubo_ros2_moveit_config aubo_moveit_sim.launch.py
```

## Auto Control System / 自动控制系统

This workspace includes an automated control system (`aubo_auto_control` package) that can automatically control the robot arm without manual RViz interaction.
本工作空间包含一个自动控制系统（`aubo_auto_control` 包），可以在无需手动 RViz 交互的情况下自动控制机械臂。

### Launch Files / 启动文件

#### Simulation Mode / 仿真模式

##### 1. Full Auto Control (with pose publisher) / 完整自动控制（包含位姿发布器）
Starts complete automated system including pose publisher that cycles through predefined poses:
启动完整的自动化系统，包括循环发布预定义位姿的位姿发布器：
```bash
ros2 launch aubo_auto_control full_auto_control.launch.py
```

##### 2. Auto Control Only (without pose publisher) / 仅自动控制（不包含位姿发布器）
Starts MoveIt system and auto controller, but requires manual pose publishing:
启动 MoveIt 系统和自动控制器，但需要手动发布位姿：
```bash
ros2 launch aubo_auto_control auto_control.launch.py
```

#### Real Robot Mode / 真实机器人模式

##### Auto Control Only for Real Robot (without pose publisher) / 真实机器人仅自动控制（不包含位姿发布器）
Connects to real robot with auto controller only (safer - requires manual pose verification):
连接到真实机器人，仅启用自动控制器（更安全 - 需要手动验证位姿）：
```bash
ros2 launch aubo_auto_control auto_control_real.launch.py robot_ip:=192.168.1.2
```

**Safety Note / 安全提示:** For real robots, only the controller-only version is provided to prevent automatic execution of potentially unsafe predefined poses. Always verify poses manually before sending to real hardware.
对于真实机器人，仅提供控制器版本以防止自动执行可能不安全的预定义位姿。在发送到真实硬件之前，请务必手动验证位姿。

**Note / 注意:** Replace `192.168.1.2` with your actual robot IP address.
将 `192.168.1.2` 替换为您的实际机器人 IP 地址。

### Message Formats / 消息格式

#### Target Pose Message / 目标位姿消息
The auto controller subscribes to `/target_pose` topic with the following message format:
自动控制器订阅 `/target_pose` 话题，消息格式如下：

**Topic / 话题:** `/target_pose`  
**Message Type / 消息类型:** `geometry_msgs/msg/PoseStamped`

**Message Structure / 消息结构:**
```yaml
header:                        # 消息头信息
  stamp:                       # 时间戳
    sec: 0                     # 秒数
    nanosec: 0                 # 纳秒数
  frame_id: 'base_link'        # 坐标系名称（通常使用机器人基座坐标系）
pose:                          # 位姿信息
  position:                    # 位置坐标（单位：米）
    x: 0.3                     # X轴位置（前后方向，正值向前）
    y: 0.2                     # Y轴位置（左右方向，正值向左）
    z: 0.4                     # Z轴位置（上下方向，正值向上）
  orientation:                 # 方向信息（四元数表示）
    x: 0.0                     # 四元数X分量
    y: 0.0                     # 四元数Y分量
    z: 0.0                     # 四元数Z分量
    w: 1.0                     # 四元数W分量（单位四元数，表示无旋转）
```

**Example Usage / 使用示例:**
```bash
# Publish a single target pose / 发布单个目标位姿
# 这个命令会向机械臂发送一个目标位置，机械臂将规划路径并移动到指定位置
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: 'base_link'        # 使用机器人基座坐标系
pose:
  position:
    x: 0.3                     # 向前移动30厘米
    y: 0.2                     # 向左移动20厘米  
    z: 0.4                     # 向上移动40厘米
  orientation:
    x: 0.0                     # 保持末端执行器水平
    y: 0.0                     # 无俯仰角度
    z: 0.0                     # 无偏航角度
    w: 1.0"                    # 单位四元数，表示无旋转
```

**坐标系说明 / Coordinate System:**
- **base_link**: 机器人基座坐标系，通常以机器人底座为原点
- **X轴**: 正方向为机器人前方
- **Y轴**: 正方向为机器人左侧
- **Z轴**: 正方向为机器人上方

**四元数说明 / Quaternion Explanation:**
- 四元数 (x=0, y=0, z=0, w=1) 表示无旋转状态
- 使用其他四元数值可以控制末端执行器的方向
- 建议使用在线四元数计算器来获取所需的旋转角度

**安全提示 / Safety Notes:**
- 确保目标位置在机械臂工作空间内
- 避免设置可能导致碰撞的位置
- 对于真实机器人，请先在仿真环境中测试位姿

