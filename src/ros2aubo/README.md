
### Using MoveIt

You can also run the MoveIt example and use the `MotionPlanning` plugin in RViZ to start planning:

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/aubo_ros2_ws/install/aubo_ros2_driver/lib/aubo_ros2_driver/aubocontroller
ros2 launch aubo_ros2_moveit_config aubo_moveit.launch.py robot_ip:=[robot ip]
ros2 topic pub --once /aubo_robot/robot_control std_msgs/msg/String 'data: powerOn'
```

Test sim:
```bash
ros2 launch  aubo_ros2_moveit_config aubo_moveit_sim.launch.py
```

## Auto Control System

This workspace includes an automated control system (`aubo_auto_control` package) that can automatically control the robot arm without manual RViz interaction.

### Launch Files

#### Simulation Mode

##### 1. Full Auto Control (with pose publisher)
Starts complete automated system including pose publisher that cycles through predefined poses:
```bash
ros2 launch aubo_auto_control full_auto_control.launch.py
```

##### 2. Auto Control Only (without pose publisher)
Starts MoveIt system and auto controller, but requires manual pose publishing:
```bash
ros2 launch aubo_auto_control auto_control.launch.py
```

#### Real Robot Mode

##### Auto Control Only for Real Robot (without pose publisher)
Connects to real robot with auto controller only (safer - requires manual pose verification):
```bash
ros2 launch aubo_auto_control auto_control_real.launch.py robot_ip:=192.168.1.2
```

**Safety Note:** For real robots, only the controller-only version is provided to prevent automatic execution of potentially unsafe predefined poses. Always verify poses manually before sending to real hardware.

**Note:** Replace `192.168.1.2` with your actual robot IP address.

### Message Formats

#### Target Pose Message
The auto controller subscribes to `/target_pose` topic with the following message format:

**Topic:** `/target_pose`  
**Message Type:** `geometry_msgs/msg/PoseStamped`

**Message Structure:**
```yaml
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'base_link'
pose:
  position:
    x: 0.3      # X position in meters
    y: 0.2      # Y position in meters  
    z: 0.4      # Z position in meters
  orientation:
    x: 0.0      # Quaternion x
    y: 0.0      # Quaternion y
    z: 0.0      # Quaternion z
    w: 1.0      # Quaternion w
```

**Example Usage:**
```bash
# Publish a single target pose
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped "
header:
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

