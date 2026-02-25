import launch
import launch.actions
import launch.events
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable
import launch_ros.actions
import os
 
def generate_launch_description():
    return LaunchDescription([

	# the ones for the offboard_rtps node
	
	launch_ros.actions.Node(package='offboard_rtps', executable='offboard_rtps_main', output='screen',
	
      	    arguments=["x500_1", "8.4121", "0.250", "0.250"],
	),
	launch_ros.actions.Node(package='offboard_rtps', executable='offboard_rtps_main', output='screen',

      	    arguments=["x500_2", "8.4121", "0.250", "0.250"],
	),
	launch_ros.actions.Node(package='offboard_rtps', executable='offboard_rtps_main', output='screen',

      	    arguments=["x500_3", "8.4121", "0.250", "0.250"],
	),
	
	
	#kill all below after fail 
	# the ones for the mission_handler node




	launch_ros.actions.Node(package='mission_handler', executable='transitions_handler_main', output='screen',

      	    arguments=["x500_1"],
	),
	

	launch_ros.actions.Node(package='mission_handler', executable='transitions_handler_main', output='screen',

      	    arguments=["x500_2"],
	),
	
	
	launch_ros.actions.Node(package='mission_handler', executable='transitions_handler_main', output='screen',

      	    arguments=["x500_3"],
	),
	
	
	
	launch_ros.actions.Node(package='mission_handler', executable='states_handler_main', output='screen',

	    arguments=["x500_1"],
	),
	
	launch_ros.actions.Node(package='mission_handler', executable='states_handler_main', output='screen',

	    arguments=["x500_2"],
	),
	
	launch_ros.actions.Node(package='mission_handler', executable='states_handler_main', output='screen',

	    arguments=["x500_3"],
	),
	
	
	launch_ros.actions.Node(package='mission_handler', executable='failures_handler_main', output='screen',

	    arguments=["x500_1"],
	),
	
	launch_ros.actions.Node(package='mission_handler', executable='failures_handler_main', output='screen',

	    arguments=["x500_2"],
	),
	
	launch_ros.actions.Node(package='mission_handler', executable='failures_handler_main', output='screen',

	    arguments=["x500_3"],
	),

	launch_ros.actions.Node(
    package='mission_handler',
    executable='failure_monitor_node',
    name='failure_monitor',
    output='screen',
),

	# MPC/PID switcher nodes
	launch_ros.actions.Node(
    package='mission_handler',
    executable='mpc_pid_switcher_x500_1_node',
    name='mpc_pid_switcher_x500_1',
    output='screen',
),

	launch_ros.actions.Node(
    package='mission_handler',
    executable='mpc_pid_switcher_x500_2_node',
    name='mpc_pid_switcher_x500_2',
    output='screen',
),

	launch_ros.actions.Node(
    package='mission_handler',
    executable='mpc_pid_switcher_x500_3_node',
    name='mpc_pid_switcher_x500_3',
    output='screen',
),

	# Input setpoint booster nodes (separate for each UAV)
	launch_ros.actions.Node(
    package='mission_handler',
    executable='input_setpoint_booster_uav1_node',
    name='input_setpoint_booster_uav1',
    output='screen',
),

	launch_ros.actions.Node(
    package='mission_handler',
    executable='input_setpoint_booster_uav2_node',
    name='input_setpoint_booster_uav2',
    output='screen',
),

	launch_ros.actions.Node(
    package='mission_handler',
    executable='input_setpoint_booster_uav3_node',
    name='input_setpoint_booster_uav3',
    output='screen',
),

    ])



	# DeclareLaunchArgument(
		# 'world_name',
		# default_value = argument_path,
		# description = 'Argument for Gazebo launch file'),

	# IncludeLaunchDescription( 
	# PythonLaunchDescriptionSource(['/opt/ros/dashing/share/gazebo_ros/launch/empty_world.launch.py']),
	# launch_arguments={'world_name': argument_path}.items()
       #  ),
