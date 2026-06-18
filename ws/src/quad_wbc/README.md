# quad_wbc

`quad_wbc` provides whole-body control interfaces for the quadruped controller stack. The package exposes a small common API in `WBCInterface` and two implementations:

- `WBCArcOPT`: an ARC-OPT/WBC based controller that solves a quadratic program for joint torque, velocity, and position commands.
- `InverseDynamics`: a lighter Cartesian-command controller that maps contact legs to wrench/position commands and swing legs to Cartesian position/velocity commands.

## Public Interface

`WBCInterface<JointCommandType>` is the common controller contract. Implementations consume the current robot state, model data, feet references, desired contact forces, contact schedule, and body target, then write a joint command type chosen by the implementation.

The update methods are intentionally separated:

- `UpdateState(...)` provides the latest floating-base pose, twist, joint states, and contact state.
- `UpdateModel(...)` provides robot dynamic parameters that can change at runtime.
- `UpdateFeetTarget(...)` provides desired foot position, velocity, and acceleration references.
- `UpdateWrenches(...)` provides desired contact forces for stance feet.
- `UpdateFootContact(...)` activates stance or swing behavior for each leg.
- `UpdateTarget(...)` provides desired body pose and twist.
- `GetJointCommand(...)` computes the next command and returns timing/success metadata in `WBCReturn`.

## WBCArcOPT Algorithm

`WBCArcOPT` is the solver-backed whole-body controller. It loads the robot model through `wbc::RobotModelPinocchio`, configures contact points from the feet names, and selects a QP solver by name. `QPOasesSolver` is always supported by this package; optional solvers are compiled in when their pkg-config targets are available.

At construction time, the class:

1. Configures a floating-base WBC robot model from the URDF.
2. Builds either `AccelerationSceneReducedTSID` or `AccelerationSceneTSID`.
3. Adds a base spatial acceleration task, one contact-force task per foot, and one foot spatial acceleration task per foot.
4. Configures Cartesian PD controllers for the body and swing-foot references.
5. Maps the repository's joint-name order onto the WBC robot model joint order.

At runtime, the controller loop is:

1. `UpdateState` copies the floating-base state and joint state into WBC types, then updates the WBC robot model.
2. `UpdateModel` refreshes the base mass, center of mass, and inertia inside the Pinocchio model.
3. `UpdateFootContact` activates contact-force tasks for stance feet and foot-pose tasks for swing feet, then updates the active contact set in the WBC robot model.
4. `UpdateWrenches`, `UpdateFeetTarget`, and `UpdateTarget` store the desired force, foot, and body references.
5. `GetJointCommand` evaluates the body and foot PD controllers, updates task references, builds the QP through the WBC scene, solves it, and maps the solved WBC joint order back into leg-indexed torque/velocity/position commands.

If the QP solve throws, `WBCArcOPT` reports `success = false`, sends zero torque, and leaves velocity/position in the WBC unset state before mapping the command.

## InverseDynamics Algorithm

`InverseDynamics` implements the same interface with `CartesianCommands`. It is useful when the caller wants per-leg Cartesian targets rather than a QP solution.

For stance legs, it:

- sets Cartesian foot velocity to zero,
- expresses the foot target in the selected body target frame,
- transforms the desired world wrench into the body frame and negates it for the command.

For swing legs, it:

- computes the foot target position in the current body frame,
- combines measured body velocity with the target body velocity using `target_velocity_blend`,
- adds the swing-foot target velocity and body angular-velocity contribution,
- sets the contact force command to zero.

The `foot_position_based_on_target_height` and `foot_position_based_on_target_orientation` flags decide whether stance foot position commands use target body height/orientation or the current state. `target_velocity_blend` is clamped to `[0, 1]`.

## Tests

The unit tests in `test/test_inverse_dynamics.cpp` use small mock model and state classes to check the deterministic `InverseDynamics` behavior:

- stance-leg commands use the target frame and transformed wrench,
- swing-leg commands use the current body frame and target foot velocity,
- `target_velocity_blend` clamps values outside `[0, 1]`.

Run them with:

```bash
colcon build --packages-select quad_wbc --cmake-args -DWITH_VICON=OFF
colcon test --packages-select quad_wbc
colcon test-result --verbose
```

If `colcon test-result --verbose` reports failures for tests that are no longer registered, clear the old result XML for this package and rerun the tests:

```bash
colcon test-result --delete-yes --test-result-base build/quad_wbc
colcon test --packages-select quad_wbc
colcon test-result --verbose
```
