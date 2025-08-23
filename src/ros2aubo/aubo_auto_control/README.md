# Aubo机械臂自动控制包

这个包提供了自动控制Aubo机械臂的功能，可以替代手动在rviz中拖动轨迹球和点击plan&execute的操作。

## 功能

该包包含两个主要节点：

1. **pose_publisher**: 发布预定义的目标位姿
2. **auto_move_controller**: 订阅目标位姿并自动控制机械臂执行

## 使用方法

### 方法1：完全自动运行（同时启动控制器和位姿发布器）

1. 首先启动aubo_moveit_sim：
```bash
ros2 launch aubo_ros2_moveit_config aubo_moveit_sim.launch.py
```

2. 在新终端中启动完全自动控制：
```bash
source install/setup.bash
ros2 launch aubo_auto_control full_auto_control.launch.py
```

这将同时启动位姿发布器（每5秒发布一个新的目标位姿）和自动控制器。

### 方法2：只启动控制器

1. 首先启动aubo_moveit_sim：
```bash
ros2 launch aubo_ros2_moveit_config aubo_moveit_sim.launch.py
```

2. 启动只有控制器的版本：
```bash
source install/setup.bash
ros2 launch aubo_auto_control auto_control.launch.py
```

3. 手动发布目标位姿：
```bash
ros2 topic pub /target_pose geometry_msgs/msg/PoseStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'base_link'
pose:
  position:
    x: 0.3
    y: 0.2
    z: 0.4
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

## 预定义的目标位姿

pose_publisher节点包含4个预定义的目标位姿：

1. **位姿1**: [0.3, 0.2, 0.4] - 正向姿态
2. **位姿2**: [0.4, -0.1, 0.5] - 绕Y轴旋转90度
3. **位姿3**: [0.2, 0.3, 0.6] - 绕Z轴旋转90度  
4. **回家位姿**: [0.35, 0.0, 0.45] - 初始位置

这些位姿会循环发布。

## 自定义目标位姿

您可以修改`src/pose_publisher.cpp`文件中的`init_target_poses()`函数来自定义目标位姿：

```cpp
geometry_msgs::msg::PoseStamped custom_pose;
custom_pose.header.frame_id = "base_link";
custom_pose.pose.position.x = 0.5;  // 您的X坐标
custom_pose.pose.position.y = 0.1;  // 您的Y坐标
custom_pose.pose.position.z = 0.3;  // 您的Z坐标
custom_pose.pose.orientation.x = 0.0;
custom_pose.pose.orientation.y = 0.0;
custom_pose.pose.orientation.z = 0.0;
custom_pose.pose.orientation.w = 1.0;
target_poses_.push_back(custom_pose);
```

## 节点说明

### pose_publisher
- **功能**: 发布预定义的目标位姿
- **话题**: `/target_pose` (geometry_msgs/PoseStamped)
- **频率**: 每5秒发布一次

### auto_move_controller  
- **功能**: 订阅目标位姿并控制机械臂执行
- **订阅话题**: `/target_pose` (geometry_msgs/PoseStamped)
- **Action客户端**: `/aubo_i5_controller/follow_joint_trajectory`
- **规划组**: `manipulator_i5`

## 安全注意事项

- 确保目标位姿在机械臂的工作空间内
- 运行前请检查周围环境，确保机械臂运动路径安全
- 程序会自动检查是否有运动正在执行，避免重复命令
- 可以随时按Ctrl+C停止程序

## 故障排除

1. **如果机械臂不动**：
   - 检查aubo_moveit_sim是否正常运行
   - 确认action服务器`/aubo_i5_controller/follow_joint_trajectory`是否可用
   - 检查目标位姿是否在工作空间内

2. **如果规划失败**：
   - 尝试修改目标位姿
   - 检查是否有碰撞
   - 可以在rviz中手动测试相同的目标位姿

3. **如果编译失败**：
   - 确保所有依赖包都已安装
   - 运行`rosdep install --from-paths src --ignore-src -r -y`安装依赖
