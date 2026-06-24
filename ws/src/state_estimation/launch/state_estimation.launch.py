import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    if "sim:=ulab" in sys.argv[4:]:
        sim = True
        config_file = "state_estimation_ulab.yaml"
        common_config_file = "common_config_ulab.yaml"
    elif "sim:=go2" in sys.argv[4:]:
        sim = True
        config_file = "state_estimation_go2_sim.yaml"
        common_config_file = "common_config_go2.yaml"
    elif "real:=ulab" in sys.argv[4:]:
        sim = False
        config_file = "state_estimation_ulab.yaml"
        common_config_file = "common_config_ulab.yaml"
    elif "real:=go2" in sys.argv[4:]:
        sim = False
        config_file = "state_estimation_go2_real.yaml"
        common_config_file = "common_config_go2.yaml"
    else:
        print("Please specify param 'sim' or 'real' and robot. E.g. 'sim:=ulab' or 'real:=go2'.")
        exit()

    # Get the package share directory
    pkg_state_estimation = get_package_share_directory("state_estimation")
    pkg_common = get_package_share_directory("common")
    state_estimation_config_path = os.path.join(pkg_state_estimation, "config", config_file)
    common_config_path = os.path.join(pkg_common, "config", common_config_file)

    # Declare a launch argument to specify whether to use simulation time
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value=str(sim),
        description="Use simulation clock if true. Default is false."
    )

    # Get the value of the use_sim_time argument
    use_sim_time = LaunchConfiguration("use_sim_time", default=str(sim))

    # Define the launch description
    return LaunchDescription([
        # Add the declare_use_sim_time_cmd to the launch description
        declare_use_sim_time_cmd,
        # Node for your existing state_estimation_node
        Node(
            package="state_estimation",  # Replace with the actual package name
            executable="state_estimation",  # Replace with the executable name
            name="state_estimation_node",
            parameters=[state_estimation_config_path, common_config_path, {"use_sim_time": use_sim_time}],
            output='screen',
            arguments=['--ros-args', '--log-level', ["mit_controller_node:=", "debug"]], 
            prefix=['xterm -e gdb -ex run --args'],
        )
    ])
