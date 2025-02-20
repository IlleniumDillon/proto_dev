import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)

def generate_launch_description():
    rviz2 = ExecuteProcess(
        # cmd=['rviz2', '-d', os.path.join(rviz_config_share, 'rviz', 'host.rviz')],
        cmd=['rviz2'],
        output='screen'
    )
    testHav = Node(
        package='hybrid_astar_voronoi',
        executable='hybrid_astar_voronoi'
    )
    
    launchtestHav = TimerAction(
        period=10.0,
        actions=[testHav]
    )
    
    return LaunchDescription([
        rviz2,
        launchtestHav
    ])
    