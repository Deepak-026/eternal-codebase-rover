import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Find the path to the 'rsp.launch.py' file in the OTHER package
    #    (Change 'robot_description' if your package name is different)
    rsp_pkg_path = get_package_share_directory('warehouse_robot_bringup')
    rsp_launch_path = os.path.join(rsp_pkg_path, 'launch', 'rsp.launch.py')

    # 2. Create the command to Include that launch file
    #    We force 'use_sim_time' to false because we are using real hardware (Pico)
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_path),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # 3. Create the Node for your Odom -> TF bridge
    #    (This runs the python script you created in the previous step)
    odom_to_tf_node = Node(
        package='warehouse_navigation',
        executable='broadcast',  # This matches the entry point in setup.py
        name='odom_tf_broadcaster',
        output='screen'
    )

    # YOUR new custom node
    odom_to_joint_state_node = Node(
        package='warehouse_navigation',
        executable='odom_to_joint', # Whatever you named it in setup.py
        name='joint_state_publisher',
        output='screen'
    )

    odom_bridge_node = Node(
        package='warehouse_navigation',
        executable='odom_bridge',
        name='odom_raw_to_odom',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    scan_relay_node = Node(
        package='warehouse_navigation',
        executable='scan_relay',
        name='scan_relay',
        output='screen'
    )
    # 4. Return the list of things to start
    return LaunchDescription([
        odom_to_joint_state_node,
        robot_state_publisher_launch,
        odom_to_tf_node,
        odom_bridge_node,
        scan_relay_node
        
    ])