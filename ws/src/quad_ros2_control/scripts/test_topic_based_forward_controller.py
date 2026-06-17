#!/usr/bin/env python3
"""Exercise topic-based ros2_control commands with a forward position controller.

Typical workflow:
  1. Start the simulator.
  2. Start ros2_control_node with config/forward_position_test_go2.yaml.
  3. Run this script with --spawn and --adapter-for-current-simulator.

The adapter is temporary: the current simulator subscribes to interfaces/JointCmd,
while topic_based_hardware_interfaces commonly exchanges sensor_msgs/JointState.
"""

import argparse
import subprocess
import sys
import time
from typing import Dict, List

import rclpy
from interfaces.msg import JointCmd
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


GO2_JOINTS = [
    "fl_abad",
    "fl_shoulder",
    "fl_knee",
    "fr_abad",
    "fr_shoulder",
    "fr_knee",
    "bl_abad",
    "bl_shoulder",
    "bl_knee",
    "br_abad",
    "br_shoulder",
    "br_knee",
]

GO2_STAND_POSE = [
    0.0,
    0.75,
    -1.5,
    0.0,
    0.75,
    -1.5,
    0.0,
    0.75,
    -1.5,
    0.0,
    0.75,
    -1.5,
]


class TopicBasedForwardTest(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("topic_based_forward_controller_test")
        self.args = args
        self.command_pub = self.create_publisher(Float64MultiArray, args.controller_topic, 10)
        self.adapter_pub = None
        self.joint_index: Dict[str, int] = {joint: index for index, joint in enumerate(GO2_JOINTS)}

        if args.adapter_for_current_simulator:
            self.adapter_pub = self.create_publisher(JointCmd, args.simulator_joint_cmd_topic, 10)
            self.create_subscription(JointState, args.hardware_commands_topic, self.hardware_command_callback, 10)
            self.get_logger().info(
                "Adapter enabled: %s sensor_msgs/JointState -> %s interfaces/JointCmd"
                % (args.hardware_commands_topic, args.simulator_joint_cmd_topic)
            )

    def hardware_command_callback(self, msg: JointState) -> None:
        joint_cmd = JointCmd()
        joint_cmd.header.stamp = self.get_clock().now().to_msg()
        joint_cmd.header.frame_id = msg.header.frame_id

        position = [0.0] * len(GO2_JOINTS)
        velocity = [0.0] * len(GO2_JOINTS)
        effort = [0.0] * len(GO2_JOINTS)

        for source_index, joint_name in enumerate(msg.name):
            target_index = self.joint_index.get(joint_name)
            if target_index is None:
                continue
            if source_index < len(msg.position):
                position[target_index] = msg.position[source_index]
            if source_index < len(msg.velocity):
                velocity[target_index] = msg.velocity[source_index]
            if source_index < len(msg.effort):
                effort[target_index] = msg.effort[source_index]

        joint_cmd.position = position
        joint_cmd.velocity = velocity
        joint_cmd.effort = effort
        joint_cmd.kp = [self.args.kp] * len(GO2_JOINTS)
        joint_cmd.kd = [self.args.kd] * len(GO2_JOINTS)
        self.adapter_pub.publish(joint_cmd)

    def publish_position(self, position: List[float]) -> None:
        msg = Float64MultiArray()
        msg.data = position
        self.command_pub.publish(msg)


def run_ros2_control_command(command: List[str], timeout: float) -> None:
    completed = subprocess.run(command, timeout=timeout, check=False)
    if completed.returncode != 0:
        raise RuntimeError("Command failed with exit code %d: %s" % (completed.returncode, " ".join(command)))


def spawn_controller(controller_name: str, timeout: float) -> None:
    run_ros2_control_command(
        ["ros2", "control", "load_controller", "--set-state", "active", controller_name],
        timeout,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="go2", choices=["go2"], help="Robot joint ordering to use.")
    parser.add_argument("--spawn", action="store_true", help="Load and activate the test controllers before publishing.")
    parser.add_argument("--controller-name", default="forward_position_controller")
    parser.add_argument("--controller-topic", default="/forward_position_controller/commands")
    parser.add_argument("--hardware-commands-topic", default="/topic_based_joint_cmd")
    parser.add_argument("--simulator-joint-cmd-topic", default="/joint_cmd")
    parser.add_argument(
        "--adapter-for-current-simulator",
        action="store_true",
        help=(
            "Bridge sensor_msgs/JointState hardware commands to interfaces/JointCmd. "
            "Use this when topic_based_hardware_interfaces publishes on a topic that "
            "is remapped away from the simulator's native /joint_cmd."
        ),
    )
    parser.add_argument("--kp", type=float, default=20.0)
    parser.add_argument("--kd", type=float, default=1.0)
    parser.add_argument("--hold-seconds", type=float, default=5.0)
    parser.add_argument("--publish-rate", type=float, default=50.0)
    parser.add_argument("--spawn-timeout", type=float, default=10.0)
    parser.add_argument("--amplitude", type=float, default=0.15, help="Added offset on shoulder joints for visible motion.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.robot != "go2":
        print("Only go2 is currently configured for this test.", file=sys.stderr)
        return 2

    if args.spawn:
        spawn_controller("joint_state_broadcaster", args.spawn_timeout)
        spawn_controller(args.controller_name, args.spawn_timeout)

    rclpy.init()
    node = TopicBasedForwardTest(args)

    command = list(GO2_STAND_POSE)
    for index in [1, 4, 7, 10]:
        command[index] += args.amplitude

    period = 1.0 / args.publish_rate
    end_time = time.monotonic() + args.hold_seconds
    node.get_logger().info("Publishing forward position command on %s" % args.controller_topic)
    try:
        while rclpy.ok() and time.monotonic() < end_time:
            node.publish_position(command)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
