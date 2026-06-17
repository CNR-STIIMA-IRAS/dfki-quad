# quad_ros2_control

This package is the refactor home for the quadruped `ros2_control` integration.
It keeps the controller-manager YAML, topic-based hardware xacro macros, robot
selection metadata, and the running migration changelog together.

The intended command path is:

```text
MIT chainable controller plugin
  -> ros2_control command interfaces
  -> topic_based_hardware_interfaces
  -> simulator command topic
```

The simulator and the controller are allowed to use different robot model files.
Select the robot once in launch, then resolve the simulator URDF and controller
description independently from `config/robot_models.yaml`.

Example:

```bash
ros2 launch quad_ros2_control topic_based_control.launch.py robot:=go2
```

Forward-controller smoke test with the current simulator:

```bash
ros2 launch quad_ros2_control topic_based_control.launch.py \
  robot:=go2 \
  controller_config:=$(ros2 pkg prefix quad_ros2_control)/share/quad_ros2_control/config/forward_position_test_go2.yaml \
  controllers:=joint_state_broadcaster,forward_position_controller \
  joint_commands_topic:=/topic_based_joint_cmd
```

Then, in another terminal:

```bash
ros2 run quad_ros2_control test_topic_based_forward_controller.py \
  --adapter-for-current-simulator
```

The adapter is only for the current simulator command API. It converts the
topic-based hardware command stream into `interfaces/msg/JointCmd` on
`/joint_cmd`.

The launch file is intentionally staged as bringup scaffolding. It becomes fully
runtime-ready after the MIT controller node is converted into a
`controller_interface::ChainableControllerInterface` plugin and the simulator
accepts the selected topic-based command message path.
