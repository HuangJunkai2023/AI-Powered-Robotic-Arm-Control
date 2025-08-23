from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取 aubo_ros2_moveit_config 包的路径
    moveit_config_pkg = get_package_share_directory('aubo_ros2_moveit_config')
    moveit_launch_file = os.path.join(moveit_config_pkg, 'launch', 'aubo_moveit_sim.launch.py')
    
    return LaunchDescription([
        # 包含 MoveIt 启动文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch_file),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),
        
        # 自动移动控制器节点（不包括位姿发布器）
        Node(
            package='aubo_auto_control',
            executable='auto_move_controller',
            name='auto_move_controller',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        )
    ])
