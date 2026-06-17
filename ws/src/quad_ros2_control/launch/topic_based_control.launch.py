import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Development tooling
from rich.traceback import install
install(show_locals=True)

ROBOT_CONFIGS = {
    "go2": {
        "description_xacro": PathJoinSubstitution(
            [FindPackageShare("quad_ros2_control"), "urdf", "go2_topic_based.ros2_control.xacro"]
        ),
        "controller_config": os.path.join(
            get_package_share_directory("quad_ros2_control"), "config", "controller_manager_go2.yaml"
        ),
    },
}


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    robot = LaunchConfiguration("robot").perform(context)
    if robot not in ROBOT_CONFIGS:
        supported = ", ".join(sorted(ROBOT_CONFIGS))
        raise RuntimeError(f"Unsupported robot '{robot}'. Supported robots: {supported}")

    robot_config = ROBOT_CONFIGS[robot]
    use_sim_time = LaunchConfiguration("use_sim_time")
    joint_commands_topic = LaunchConfiguration("joint_commands_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    controller_config = LaunchConfiguration("controller_config").perform(context)
    if not controller_config:
        controller_config = robot_config["controller_config"]
    controllers = [
        controller.strip()
        for controller in LaunchConfiguration("controllers").perform(context).split(",")
        if controller.strip()
    ]

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                robot_config["description_xacro"],
                " joint_commands_topic:=",
                joint_commands_topic,
                " joint_states_topic:=",
                joint_states_topic,
            ]
        )
    }

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, controller_config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ]

    for controller in controllers:
        actions.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--controller-manager", "/controller_manager"],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot", default_value="go2", description="Robot model key, e.g. go2."),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("controller_config", default_value=""),
            DeclareLaunchArgument("controllers", default_value="joint_state_broadcaster,mit_controller"),
            DeclareLaunchArgument("joint_commands_topic", default_value="/joint_cmd"),
            DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
            OpaqueFunction(function=launch_setup),
        ]
    )
