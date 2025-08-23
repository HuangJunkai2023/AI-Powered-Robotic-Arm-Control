from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明启动参数
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.2",
            description="IP address of the robot server (remote).",
        )
    )
    
    # 获取启动参数
    robot_ip = LaunchConfiguration("robot_ip")
    
    # 获取 aubo_ros2_moveit_config 包的路径
    moveit_config_pkg = get_package_share_directory('aubo_ros2_moveit_config')
    moveit_launch_file = os.path.join(moveit_config_pkg, 'launch', 'aubo_moveit.launch.py')
    
    return LaunchDescription(declared_arguments + [
        # 包含真实机器人的 MoveIt 启动文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch_file),
            launch_arguments={'robot_ip': robot_ip}.items()
        ),
        
        # 自动移动控制器节点（真实机器人模式，不使用仿真时间）
        Node(
            package='aubo_auto_control',
            executable='auto_move_controller',
            name='auto_move_controller',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        )
    ])
