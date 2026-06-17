# MIT Controller ros2_control Refactor Changelog

This file is the living migration log for moving the MIT controller from a
standalone ROS 2 node into a chainable `ros2_control` controller plugin connected
to `topic_based_hardware_interfaces`.

## 2026-06-17 - Bringup Scaffolding and Refactor Contract

### Added

- Added the `quad_ros2_control` package as the refactor home for:
  - controller-manager YAML files,
  - topic-based hardware xacro macros,
  - robot model selection metadata,
  - bringup launch scaffolding,
  - this detailed migration changelog.
- Added `config/robot_models.yaml` to make robot selection explicit and
  extensible. The simulator URDF and controller robot description are separate
  entries because they can legitimately be different models of the same robot.
- Added `urdf/quad_topic_based_system.ros2_control.xacro`, reusable macros for
  declaring topic-based ros2_control hardware and individual joint interfaces.
- Added `urdf/go2_topic_based.ros2_control.xacro`, a GO2-specific macro wrapper
  using the joint order already used by the MIT controller configuration:
  `fl_abad`, `fl_shoulder`, `fl_knee`, `fr_abad`, `fr_shoulder`, `fr_knee`,
  `bl_abad`, `bl_shoulder`, `bl_knee`, `br_abad`, `br_shoulder`, `br_knee`.
- Added `config/controller_manager_go2.yaml` as the first controller-manager
  config template. It defines `joint_state_broadcaster` and a future
  `mit_controller` plugin entry.
- Added `config/forward_position_test_go2.yaml` for smoke-testing the topic-based
  hardware interface with a `forward_command_controller/ForwardCommandController`.
- Added `launch/topic_based_control.launch.py` with normal ROS launch arguments:
  `robot`, `use_sim_time`, `controller_config`, `controllers`,
  `joint_commands_topic`, and `joint_states_topic`.
- Added `scripts/test_topic_based_forward_controller.py`, which can load the
  forward controller, publish a visible GO2 joint position step, and optionally
  adapt topic-based `sensor_msgs/JointState` commands to the simulator's current
  `interfaces/msg/JointCmd` command topic.

### Design Decisions

- The new bringup package uses `robot:=go2` instead of the existing pattern that
  scans `sys.argv` for `sim:=go2` or `real:=go2`. This is deliberate because
  launch arguments compose cleanly when bringup starts controller manager,
  robot-state publisher, simulator, and spawners together.
- Robot selection is centralized in `robot_models.yaml`. Adding a new robot
  should mean adding a new top-level key and matching controller-manager YAML,
  not editing C++ controller code.
- The simulator model and the controller model are modeled separately:
  - `simulator_urdf` points to the robot model consumed by Drake simulation.
  - `controller_description_xacro` points to the robot description used by
    `robot_state_publisher` and `controller_manager`.
  This supports using Drake-specific collision/proximity URDFs in simulation
  while using a ros2_control-enhanced description for controller bringup.
- The topic-based hardware command topic defaults to `/joint_cmd` to preserve the
  current simulator-facing topic name. The message/type compatibility still has
  to be resolved during the simulator integration step.
- The forward-controller smoke test uses `/topic_based_joint_cmd` for the
  topic-based hardware output and bridges that to `/joint_cmd` when testing
  against the current simulator. This avoids publishing two different message
  types on the same topic.

### Current Architecture Observed

- `src/controllers/src/mit_controller_node.cpp` currently implements
  `MITController` as an `rclcpp::Node`.
- The controller subscribes to `/quad_state` and `/quad_control_target`.
- The controller publishes either `leg_joint_cmd` or `leg_cmd`, depending on
  `leg_control_mode`.
- `src/drivers/src/leg_driver.cpp` bridges `leg_joint_cmd` / `leg_cmd` to
  `joint_cmd`.
- `src/simulator/src/drake_simulator.cpp` subscribes directly to `joint_cmd`.

### Migration Target

- Convert `MITController` into a plugin inheriting from
  `controller_interface::ChainableControllerInterface`.
- Move final joint command output from ROS publishers into ros2_control command
  interfaces.
- Keep non-critical telemetry publishers such as `gait_state`, `solve_time`,
  `wbc_target`, and `controller_heartbeat` as controller-node publishers.
- Keep `/quad_state` as an auxiliary input in the first refactor round because
  the controller needs base pose, twist, acceleration, contacts, contact forces,
  and joint states. Later, this should be replaced by a chainable state-estimator
  controller that exports reference/state interfaces to downstream controllers.

### Open Technical Work

- Confirm the exact message type emitted by
  `joint_state_topic_hardware_interface/JointStateTopicSystem` for command
  topics in the ROS distribution used by this workspace.
- Decide how to represent `kp` and `kd`, because the current simulator command
  message `interfaces::msg::JointCmd` contains position, velocity, effort, `kp`,
  and `kd`, while standard ros2_control joint interfaces usually expose
  position, velocity, and effort.
- Update the simulator command subscriber if the topic-based hardware output is
  not `interfaces::msg::JointCmd`.
- Export the MIT controller with `pluginlib`.
- Move the existing MIT controller parameters from the `mit_controller_node`
  namespace to the future controller plugin namespace `mit_controller`.
- Replace direct launch of `mitcontrollernode` with controller-manager spawners
  once the plugin exists.

### Build Notes

- `ament_cmake` runs a Python helper that imports `catkin_pkg` to parse
  `package.xml`. This is not a dependency on the ROS 1 catkin build system.
- On this workstation, CMake selected `/home/feynman/.local/bin/python3.11`,
  which did not provide `catkin_pkg`, while `/usr/bin/python3` did. The package
  now pins `Python3_EXECUTABLE` to the system Python when available and seeds
  `PYTHONPATH` with ROS site-packages under `/opt/ros/*/lib/python*/site-packages`
  so ament's Python helpers can import both `catkin_pkg` and `ament_package`.

### Deferred Round Two

- Split the monolithic MIT controller into cascade chainable controllers:
  - state estimator,
  - gait scheduler,
  - swing planner,
  - MPC,
  - WBC,
  - final joint command writer.
- Define the reference interfaces passed between these controllers before
  decomposing the implementation. This avoids baking temporary topic contracts
  into the final controller chain.
