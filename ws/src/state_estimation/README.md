# State Estimation

This package estimates the floating-base state of a quadruped by combining:

- IMU angular velocity and linear acceleration;
- joint positions, velocities, efforts, and derived joint accelerations;
- forward kinematics from the robot model;
- detected or measured foot contacts;
- the commanded gait contact schedule; and
- optionally, Vicon pose measurements instead of the Kalman filter.

The default estimator is a **contact-aided invariant extended Kalman filter
(InEKF)**. IMU measurements propagate the floating-base state, while feet that
are believed to be stationary provide kinematic corrections. Contact points are
added to and removed from the filter state as the robot enters and leaves
stance.

The ROS 2 executable is `state_estimation`, and the node name is
`state_estimation_node`.

## Contents

- [Estimated state](#estimated-state)
- [Coordinate frames and conventions](#coordinate-frames-and-conventions)
- [Algorithm overview](#algorithm-overview)
- [InEKF integration](#inekf-integration)
- [Initialization](#initialization)
- [IMU propagation](#imu-propagation)
- [Contact and kinematic correction](#contact-and-kinematic-correction)
- [Contact detection](#contact-detection)
- [ROS interfaces](#ros-interfaces)
- [Parameters](#parameters)
- [Building](#building)
- [Running](#running)
- [Resetting the estimator](#resetting-the-estimator)
- [Tuning guide](#tuning-guide)
- [Current implementation notes and limitations](#current-implementation-notes-and-limitations)
- [Relevant source files](#relevant-source-files)
- [References](#references)

## Estimated state

The base InEKF state contains orientation, linear velocity, position, and IMU
biases:

$$
\mathbf{R}\in SO(3),\qquad
\mathbf{v}\in\mathbb{R}^3,\qquad
\mathbf{p}\in\mathbb{R}^3,
$$

$$
\mathbf{b}_g\in\mathbb{R}^3,\qquad
\mathbf{b}_a\in\mathbb{R}^3.
$$

The group-valued part is initialized as

$$
\mathbf{X} =
\begin{bmatrix}
\mathbf{R} & \mathbf{v} & \mathbf{p}\\
\mathbf{0} & 1 & 0\\
\mathbf{0} & 0 & 1
\end{bmatrix},
$$

and the Euclidean bias vector is

$$
\boldsymbol{\theta} =
\begin{bmatrix}
\mathbf{b}_g\\
\mathbf{b}_a
\end{bmatrix}.
$$

The initial covariance is a 15-by-15 block-diagonal matrix ordered as:

1. orientation error;
2. velocity error;
3. position error;
4. gyroscope bias error; and
5. accelerometer bias error.

Each active contact adds a three-dimensional world-frame contact position to
the group state and three corresponding covariance dimensions. Consequently,
the filter state grows during stance and shrinks when a foot leaves contact.

The published `interfaces/msg/QuadState` additionally contains:

- the body pose and pose covariance;
- body linear and angular velocity;
- linear and angular acceleration;
- filtered joint position, velocity, acceleration, and effort;
- one contact flag per foot;
- estimated or measured ground-contact forces; and
- a belly-contact flag.

## Coordinate frames and conventions

The InEKF state represents the **IMU frame** in the world frame:

- `R` rotates vectors from IMU coordinates into world coordinates;
- `v` is the IMU linear velocity in world coordinates;
- `p` is the IMU position in world coordinates;
- gyroscope and accelerometer measurements are expressed in the IMU frame; and
- augmented foot contact positions are expressed in the world frame.

The kinematic measurement passed to InEKF is the foot position relative to the
IMU:

$$
{}^I\mathbf{p}_{IF_i}
=
{}^I\mathbf{T}_B\,
{}^B\mathbf{p}_{BF_i}.
$$

In the implementation this is computed from
`CalcFootPositionInBodyFrame()` and `GetBodyToIMU().inverse()`.

The published pose is converted back from the estimated IMU position to the
robot body position using the configured body-to-IMU translation. The current
robot model exposes this extrinsic as a translation only; no body-to-IMU
rotation is applied by this package.

Leg IDs `0` through `3` are used as InEKF contact IDs. Their physical ordering
is the ordering defined by `ModelInterface` and the robot model configuration.

## Algorithm overview

The estimator is asynchronous. It does not collect all sensor measurements into
one synchronized callback.

```text
IMU measurement
    |
    +--> remove estimated IMU biases
    +--> InEKF inertial propagation
    +--> filter angular velocity and acceleration for QuadState

Joint-state measurement
    |
    +--> moving-average joint velocity, acceleration, and effort
    +--> estimate foot forces and contact state
    +--> compute each foot position relative to the IMU
    +--> combine measured contact with the gait schedule
    +--> add/remove contact states and apply InEKF kinematic correction
    +--> optionally constrain stance-foot height to the ground plane

Publish timers
    |
    +--> read the latest InEKF state and covariance
    +--> publish quad_state and quad_state/lf
```

The main sequence is:

1. Wait for at least one IMU message and one valid joint-state message.
2. Estimate the initial base height from forward kinematics, assuming all four
   feet are on the ground.
3. Construct the initial `inekf::RobotState` and configure process noise.
4. Propagate on every subsequent IMU callback.
5. Detect contacts and correct using leg kinematics on every subsequent valid
   joint-state callback.
6. Read and publish the latest state at the configured publication periods.

The timer named by `kalman_update_rate` currently calls an empty callback.
Actual prediction and correction are driven directly by IMU and joint-state
callbacks.

## InEKF integration

### Library connection

The package finds the external `inekf` CMake package and links:

```cmake
find_package(inekf)
target_link_libraries(state_estimation inekf::inekf)
```

`KalmanFilter` is the local adapter between ROS/model data and the external
library. It owns:

```cpp
inekf::InEKF filter_;
std::unique_ptr<inekf::RobotState> init_state_;
```

The integration uses the following InEKF API:

- `setNoiseParams()` to configure continuous process-noise covariances;
- `setState()` to install the initial state;
- `Propagate()` for IMU prediction;
- `setContacts()` to report the active contact set;
- `CorrectKinematics()` to augment, correct, or remove foot contact states;
- `CorrectContactPosition()` for the optional planar-ground constraint;
- `getState()` to read pose, velocity, biases, and covariance;
- `clear()` and `setState()` during a full reset; and
- contact-state accessors for adaptive measurement covariance.

This workspace therefore requires an InEKF version that provides these methods,
including `clear()`, `CorrectContactPosition()`, and
`RobotState::getVector()`.

Mutex support is disabled for the included InEKF headers by defining
`INEKF_USE_MUTEX` as `false` before including `InEKF.h`. The node is normally
spun with the single-threaded ROS executor used by `rclcpp::spin`.

### Why an invariant EKF is used

A conventional EKF linearizes pose errors in ordinary local coordinates. An
InEKF instead defines the navigation error using the symmetry of the underlying
matrix Lie group. For this inertial/contact model, the resulting error dynamics
have a structured form and the right-invariant correction is applied as

$$
\mathbf{X}^{+}
=
\exp(\delta\boldsymbol{\xi})\,\mathbf{X},
\qquad
\boldsymbol{\theta}^{+}
=
\boldsymbol{\theta}+\delta\boldsymbol{\theta}.
$$

The external library computes the Kalman gain, Lie-group exponential, and
Joseph-form covariance update. This package supplies the measurements,
kinematics, contact decisions, noise values, initialization, and ROS message
conversion.

### Dynamic contact-state augmentation

For each foot, `setContacts()` tells InEKF whether that contact is active.
`CorrectKinematics()` then performs one of three operations:

- **new stance contact:** initialize the world contact point as
  \(\mathbf{d}_i=\mathbf{p}+\mathbf{R}\,{}^I\mathbf{p}_{IF_i}\) and augment
  the state and covariance;
- **continuing stance contact:** correct the base/contact state using the
  measured IMU-to-foot displacement; or
- **contact lost:** remove that foot's world contact point and covariance block
  from the state.

For a continuing contact, the measurement expresses that the world-frame foot
position stored in the state and the position reconstructed from the current
base pose plus leg kinematics must agree:

$$
\mathbf{d}_i
\approx
\mathbf{p}
+
\mathbf{R}\,{}^I\mathbf{p}_{IF_i}.
$$

The local code provides a 6-by-6 kinematics covariance whose translational block
is

$$
\mathbf{\Sigma}_{F_i}
=
\sigma_{\text{foot}}^2\mathbf{I}_3.
$$

The InEKF library rotates this translational covariance into the world frame
before applying the correction.

## Initialization

The filter is lazily initialized after:

- an IMU message has been received; and
- a valid 12-joint state has been received.

The current initialization procedure is:

1. Assume all four feet are in contact.
2. Compute the body height from the mean vertical foot position.
3. Set initial horizontal position to zero.
4. Convert the body position to an IMU position using the body-to-IMU
   translation.
5. Set initial linear velocity to zero.
6. Set orientation to the identity quaternion.
7. Initialize IMU biases and covariance from `kalman.prior.*`.

Although the first raw IMU orientation is copied into the outgoing message and
unlocks initialization, the current filter initialization explicitly uses the
identity orientation. The IMU message's orientation covariance is not consumed.

The initial covariance blocks are:

$$
\mathbf{P}_0 =
\operatorname{diag}\left(
\sigma_R^2\mathbf{I}_3,
\sigma_v^2\mathbf{I}_3,
\sigma_p^2\mathbf{I}_3,
\sigma_{b_g}^2\mathbf{I}_3,
\sigma_{b_a}^2\mathbf{I}_3
\right).
$$

## IMU propagation

`ImuCallback()` calls `KalmanFilter::Predict()`. The first IMU sample after
filter initialization is stored only to establish a timestamp. For each later
sample, the previous sample is propagated over

$$
\Delta t = t_k-t_{k-1}.
$$

The measurement vector passed to InEKF is

$$
\mathbf{m} =
\begin{bmatrix}
\boldsymbol{\omega}_m\\
\mathbf{a}_m
\end{bmatrix}.
$$

The external filter subtracts the estimated biases:

$$
\boldsymbol{\omega}
=
\boldsymbol{\omega}_m-\mathbf{b}_g,
\qquad
\mathbf{a}
=
\mathbf{a}_m-\mathbf{b}_a.
$$

Its strapdown propagation is:

$$
\mathbf{R}_{k+1}
=
\mathbf{R}_k\operatorname{Exp}
\left(\boldsymbol{\omega}\Delta t\right),
$$

$$
\mathbf{v}_{k+1}
=
\mathbf{v}_k+
\left(\mathbf{R}_k\mathbf{a}+\mathbf{g}\right)\Delta t,
$$

$$
\mathbf{p}_{k+1}
=
\mathbf{p}_k+\mathbf{v}_k\Delta t+
\frac{1}{2}
\left(\mathbf{R}_k\mathbf{a}+\mathbf{g}\right)\Delta t^2,
$$

where the InEKF library uses
\(\mathbf{g}=[0,0,-9.81]^T\ \mathrm{m/s^2}\).

If `kalman.imu_measures_g` is `false`, this package adds the gravity vector in
IMU coordinates before passing acceleration to the library. This converts a
gravity-compensated input into the specific-force convention expected by the
InEKF propagation. If it is `true`, the raw acceleration is passed directly.

The process noise is configured from:

- gyroscope white noise;
- accelerometer white noise;
- gyroscope bias random walk;
- accelerometer bias random walk; and
- contact-point process noise.

After propagation, the estimated IMU biases are also removed from the signals
placed in `QuadState`. Five-sample moving averages are used for angular velocity
and linear acceleration.

## Contact and kinematic correction

### Foot kinematics

Each valid joint-state callback computes the four foot positions in the body
frame using the configured `QuadModelPino`, then translates them into the IMU
frame. If any foot position is non-finite, the complete kinematic update is
skipped.

### Contact gating

The contact sent to InEKF is not simply the raw force contact. For foot \(i\),
the implemented condition is:

```text
(detected_contact[i] OR use_only_gait_contacts)
AND gait_state.contact[i]
AND (
  gait_state.phase[i] / gait_state.duty_factor[i]
      >= min_stance_percentage_for_contact
  OR duty_factor[i] is effectively 1.0
)
```

This prevents a newly scheduled stance foot from being used immediately. The
delay allows the foot to settle before it becomes a stationary contact
constraint.

When `kalman.use_only_gait_contacts` is `true`, the measured/detected-contact
part of the first condition is bypassed, but the gait contact and stance-phase
conditions still apply.

### Planar-ground correction

When `kalman.assume_planar_ground` is enabled, every active contact receives an
additional correction on its world-frame `z` coordinate:

$$
d_{i,z}=0.
$$

Only the vertical component is selected, and its covariance is

$$
\sigma_{\text{plane}}^2.
$$

This is appropriate only when the estimator's world origin is defined on a
flat ground plane.

### Adaptive foot covariance

When `kalman.adapt_covariances` is enabled, the package attempts to increase the
foot kinematics measurement standard deviation using:

- variation between current kinematic foot positions and previously estimated
  contact positions; and
- changes in vertical contact force.

The implemented form is:

$$
\sigma_r =
\sqrt{
\sigma_0^2+
\left(
\alpha\,\overline{\sigma}_{\Delta p}
+
(1-\alpha)\sigma_0\Delta f_z
\right)^2
},
\qquad \alpha=0.5.
$$

If the result is NaN, the configured
`kalman.foot_step_measurement_std` is retained. This path currently prints
verbose diagnostic output and should be considered experimental.

### Belly contact

The node detects whether the belly is close to the ground and computes
candidate belly contact points. These values are passed to
`KalmanFilter::Update()`, but the current filter wrapper intentionally discards
them. Belly contact therefore appears in `QuadState` but does not currently
correct the InEKF state.

## Contact detection

The package can obtain foot contacts in two ways.

### Measured-force contact

If `contact_detection.use_measured_forces` is `true`, the node subscribes to
`contact_state`. Each scalar measured ground-contact force is compared with the
per-leg threshold.

The measured contact flags are used for filtering, while the model-based
`ContactDetection` object still calculates the three-dimensional force vectors
placed in `QuadState`.

If `contact_detection.update_threshold` is enabled, each threshold is updated
from moving minimum and maximum filters:

```text
threshold = min(
  moving_min(force) + threshold_offset,
  moving_max(force) - max_threshold_offset
)
```

### Model-based force contact

Without a measured-force subscription, the model estimates foot force from
joint position, velocity, acceleration, and effort. The force is smoothed with
a ten-sample moving average, and a foot is considered in contact when:

$$
\lVert\mathbf{f}_i\rVert
\geq
f_{\text{threshold},i}.
$$

### Energy-observer contact

An optional energy observer computes a filtered residual from the leg kinetic
energy and power:

$$
r_i =
\left|
k_d\left(
E_i-(\dot{E}_i+r_{i,\mathrm{prev}})\Delta t
\right)
\right|.
$$

For a leg classified as moving, a rising threshold crossing starts a swing
state. Contact is restored after at least half the configured swing time and
after the residual falls below the threshold.

The energy observer is selected only when both:

- the leg is classified as moving; and
- `contact_detection.use_energy_obs_contact_detection` is `true`.

Otherwise, force-threshold contact detection is used.

Robot-model implementations must provide meaningful kinetic-energy and energy
derivative calculations for this mode. In the current `QuadModelPino`
implementation these two methods return zero, so the energy-observer mode is
not useful with that model as presently implemented.

## ROS interfaces

All relative names below are resolved in the node's namespace.

### Subscriptions

| Topic | Type | Purpose |
|---|---|---|
| `imu_measurement` | `sensor_msgs/msg/Imu` | InEKF propagation and reported inertial quantities |
| `joint_states` | `interfaces/msg/JointState` | Kinematics, effort-based force estimation, and InEKF correction |
| `gait_state` | `interfaces/msg/GaitState` | Contact schedule and stance phase gating |
| `contact_state` | `interfaces/msg/ContactState` | Optional measured scalar contact forces |
| configured `vicon_topic` | `vicon_receiver/msg/Position` | Optional replacement for the InEKF output |

`imu_measurement`, `joint_states`, `gait_state`, `contact_state`, and Vicon use
the workspace's best-effort/no-depth QoS profile.

### Publishers

| Topic | Type | QoS | Description |
|---|---|---|---|
| `quad_state` | `interfaces/msg/QuadState` | reliable/no-depth | Main state output |
| `quad_state/lf` | `interfaces/msg/QuadState` | best-effort/no-depth | Lower-frequency copy of the latest state |

The main message timestamp is normally inherited from the latest accepted
joint-state message. The code does not currently restamp the message at publish
time.

### Services

| Service | Type | Description |
|---|---|---|
| `reset_state_estimation` | `std_srvs/srv/Trigger` | Clear dynamic InEKF state and restore the original initial state |
| `reset_state_estimation_covariances` | `std_srvs/srv/Trigger` | Request restoration of prior covariance and biases |

Both services fail cleanly if the filter has not yet initialized.

### Vicon replacement mode

If the package is compiled with `WITH_VICON=ON` and
`replace_kalman_filter_by_vicon` is `true`:

- the IMU subscription is not created;
- Vicon position and orientation are filtered with five-sample moving averages;
- velocity and acceleration are obtained by finite differences; and
- `quad_state` is published directly from the Vicon callback.

Vicon translations are converted from millimetres to metres.

This is a replacement path, not a Vicon measurement update inside the InEKF.
When the package is built with `WITH_VICON=OFF`, replacement mode is forcibly
disabled.

## Parameters

Despite their names, the three `*_rate` parameters are passed directly to
`std::chrono::duration<double>` and therefore represent **periods in seconds**,
not frequencies in hertz.

### Node timing and mode

| Parameter | Type | Meaning |
|---|---|---|
| `kalman_update_rate` | double | Period of the currently empty Kalman timer callback |
| `state_publish_rate` | double | Main state publication period |
| `state_lf_publish_rate` | double | Low-frequency state publication period |
| `laying_on_ground_detection_offset` | double[4] | Per-leg belly-contact geometry offset |
| `replace_kalman_filter_by_vicon` | bool | Replace IMU/InEKF output with Vicon |
| `vicon_topic` | string | Vicon input topic |

### InEKF process and measurement noise

All `*_std` parameters are standard deviations. The wrapper squares them when
constructing initial or measurement covariance matrices; the external
`NoiseParams` setters likewise interpret their arguments as standard
deviations.

| Parameter | Meaning |
|---|---|
| `kalman.gyroscope_std` | Per-axis gyroscope white-noise standard deviation |
| `kalman.accelerometer_std` | Per-axis accelerometer white-noise standard deviation |
| `kalman.gyroscope_bias_std` | Gyroscope bias random-walk standard deviation |
| `kalman.accelerometer_bias_std` | Accelerometer bias random-walk standard deviation |
| `kalman.contact_std` | Contact-point process-noise standard deviation |
| `kalman.foot_step_measurement_std` | Foot kinematics measurement standard deviation |
| `kalman.foot_on_plane_std` | Vertical ground-plane measurement standard deviation |
| `kalman.belly_contact_point_measurement_std` | Reserved belly-contact measurement standard deviation |
| `kalman.imu_measures_g` | Whether raw acceleration includes gravity/specific force |
| `kalman.assume_planar_ground` | Constrain active contact heights to world `z=0` |
| `kalman.adapt_covariances` | Enable experimental foot covariance adaptation |
| `kalman.min_stance_percentage_for_contact` | Fraction of scheduled stance elapsed before using a contact |
| `kalman.use_only_gait_contacts` | Use the gait schedule without requiring detected contact |

### InEKF prior

| Parameter | Meaning |
|---|---|
| `kalman.prior.gyroscope_bias` | Initial gyroscope bias |
| `kalman.prior.accelerometer_bias` | Initial accelerometer bias |
| `kalman.prior.base_orientation_std` | Initial orientation-error standard deviation |
| `kalman.prior.base_velocity_std` | Initial velocity-error standard deviation |
| `kalman.prior.base_position_std` | Initial position-error standard deviation |
| `kalman.prior.gyroscope_bias_std` | Initial gyroscope-bias standard deviation |
| `kalman.prior.accelerometer_bias_std` | Initial accelerometer-bias standard deviation |

### Contact detection

| Parameter | Meaning |
|---|---|
| `contact_detection.use_measured_forces` | Subscribe to `contact_state` for contact flags |
| `contact_detection.force_threshold` | Per-leg force threshold |
| `contact_detection.update_threshold` | Adapt measured-force thresholds online |
| `contact_detection.threshold_update_rate` | Threshold-update period in seconds |
| `contact_detection.threshold_offset` | Offset above the moving minimum |
| `contact_detection.max_threshold_offset` | Offset below the moving maximum |
| `contact_detection.threshold_filter_size` | Moving-minimum window |
| `contact_detection.max_threshold_filter_size` | Moving-maximum window |
| `contact_detection.energy_obs_kd` | Energy-observer gain |
| `contact_detection.energy_obs_threshold` | Energy-residual threshold |
| `contact_detection.leg_in_motion_joint_velocity_threshold` | Per-joint velocity threshold used to classify a moving leg |
| `contact_detection.use_energy_obs_contact_detection` | Enable energy-observer decisions for moving legs |
| `contact_detection.energy_obs_filter_size` | Energy-residual moving-average window |
| `contact_detection.leg_swing_time` | Expected swing duration used by the observer state machine |

Example configurations are installed from:

- `config/state_estimation_go2_real.yaml`;
- `config/state_estimation_go2_sim.yaml`; and
- `config/state_estimation_ulab.yaml`.

The node also constructs `QuadModelPino`, so it must receive the model
parameters required by the `common` package, usually by loading a matching
`common_config_*.yaml` file alongside the estimator configuration.

## Building

### Dependencies

The package requires:

- ROS 2 and `ament_cmake`;
- `rclcpp`;
- `sensor_msgs`;
- `std_srvs`;
- the workspace `interfaces` package;
- the workspace `common` package;
- Pinocchio and Eigen3; and
- the external `inekf` library exporting the target `inekf::inekf`.

Vicon support additionally requires `vicon_receiver`.

### Without Vicon

Vicon is optional at runtime, but the CMake option currently defaults to `ON`.
Disable it explicitly when `vicon_receiver` is unavailable:

```bash
source /opt/ros/rolling/setup.bash
source /path/to/inekf/install/setup.bash  # if provided by its workspace

colcon build \
  --packages-select state_estimation \
  --cmake-args \
    -DWITH_VICON=OFF \
    -DPython3_EXECUTABLE=/usr/bin/python3
```

If InEKF is installed in a non-ROS CMake prefix, add that prefix:

```bash
colcon build \
  --packages-select state_estimation \
  --cmake-args \
    -DWITH_VICON=OFF \
    -DCMAKE_PREFIX_PATH=/path/to/inekf/install \
    -DPython3_EXECUTABLE=/usr/bin/python3
```

Use `--cmake-clean-cache` if a previous configure selected a different Python
interpreter or stale InEKF installation.

### With Vicon

```bash
source /opt/ros/rolling/setup.bash
source /path/to/dependencies/install/setup.bash

colcon build \
  --packages-select state_estimation \
  --cmake-args \
    -DWITH_VICON=ON \
    -DPython3_EXECUTABLE=/usr/bin/python3
```

## Running

Source the dependency and local workspaces:

```bash
source /opt/ros/rolling/setup.bash
source /path/to/dependencies/install/setup.bash
source install/setup.bash
```

The package can be run directly with one estimator configuration and one common
robot-model configuration:

```bash
ros2 run state_estimation state_estimation --ros-args \
  --params-file "$(ros2 pkg prefix state_estimation)/share/state_estimation/config/state_estimation_go2_sim.yaml" \
  --params-file "$(ros2 pkg prefix common)/share/common/config/common_config_go2.yaml" \
  -p use_sim_time:=true
```

The included launch file selects the Go2 simulation configuration with:

```bash
ros2 launch state_estimation state_estimation.launch.py sim:=go2
```

Before relying on the output, verify that the required inputs exist:

```bash
ros2 topic hz /imu_measurement
ros2 topic hz /joint_states
ros2 topic echo /gait_state --once
ros2 topic info /contact_state
```

Inspect the estimator:

```bash
ros2 node info /state_estimation_node
ros2 topic echo /quad_state
ros2 topic hz /quad_state
```

## Resetting the estimator

Reset the complete filter to the state saved at initialization:

```bash
ros2 service call \
  /reset_state_estimation \
  std_srvs/srv/Trigger
```

Request a covariance and bias reset:

```bash
ros2 service call \
  /reset_state_estimation_covariances \
  std_srvs/srv/Trigger
```

See the limitation below concerning the current covariance-reset
implementation.

## Tuning guide

1. **Confirm acceleration convention first.** An incorrect
   `kalman.imu_measures_g` setting produces immediate vertical acceleration and
   position drift.
2. **Calibrate the static IMU biases.** Set `kalman.prior.*_bias` from a
   stationary data set, then choose prior standard deviations that reflect the
   calibration confidence.
3. **Tune IMU noise from stationary data.** Use per-axis standard deviations
   for the gyro and accelerometer.
4. **Validate contact decisions visually.** Compare `quad_state.foot_contact`
   with the real gait before tuning the filter. False stance contacts impose an
   incorrect zero-foot-motion constraint and can move the base estimate.
5. **Tune `foot_step_measurement_std`.** Smaller values trust rigid,
   non-slipping leg kinematics more strongly. Increase it for compliant legs,
   encoder noise, model error, or foot slip.
6. **Tune `contact_std`.** This controls how much an augmented world contact
   point is allowed to wander during propagation.
7. **Use gait-phase gating.** Increase
   `min_stance_percentage_for_contact` if touchdown transients create jumps.
8. **Enable planar ground only when its frame assumption is true.** On uneven
   terrain, ramps, or an incorrectly placed world origin, the `z=0` correction
   is systematically wrong.
9. **Check extrinsics and URDF/model parameters.** Incorrect body-to-IMU or
   body-to-foot geometry appears as estimator drift even with good noise
   tuning.

## Current implementation notes and limitations

This section describes the code as it currently exists.

- `kalman_update_rate` is a period in seconds, and its callback is empty.
  InEKF propagation and correction occur in sensor callbacks.
- The initial filter orientation is identity. The orientation in the first IMU
  message is not used to initialize `R`.
- No checks currently reject negative, zero, or very large IMU timestamp
  differences before calling `Propagate()`.
- The planar-ground correction assumes active contacts have world height zero.
- Belly-contact measurements are computed but are not fused.
- `kalman.belly_contact_point_measurement_std` is therefore currently unused.
- `ResetCovariances()` constructs a replacement covariance and restores the
  local state bias vector, but it does not call `setP()` or `filter_.setState()`.
  The service does not currently apply the intended reset to the live filter.
- `QuadState` angular acceleration uses the current quaternion-derivative
  expression multiplied by angular acceleration; it is not a direct
  world-frame angular-acceleration rotation.
- The output twist covariance only fills the leading 3-by-3 linear-velocity
  block. Angular-velocity covariance remains unchanged/default.
- The pose covariance is rearranged from InEKF ordering
  `[orientation, velocity, position, ...]` into ROS pose ordering
  `[position, orientation]`.
- Contact covariance adaptation is experimental and emits verbose standard
  output.
- With `QuadModelPino`, the energy-observer model functions currently return
  zero.
- Vicon replacement computes derivatives from arrival time and does not guard
  its first callback against an uninitialized or zero time interval.
- Vicon is a complete replacement output path; it is not fused with IMU,
  kinematics, or InEKF.

## Relevant source files

| File | Responsibility |
|---|---|
| `src/state_estimation_node.cpp` | ROS parameters, subscriptions, initialization, callback sequencing, publishing, and services |
| `include/state_estimation_node.hpp` | Node state and ROS interface declarations |
| `src/kalman_filter.cpp` | Local InEKF adapter, initialization, prediction, correction, covariance extraction, and reset |
| `include/kalman_filter.hpp` | Filter parameters and public wrapper API |
| `src/contact_detection.cpp` | Force-based and energy-observer contact detection |
| `include/contact_detection.hpp` | Contact detector state and API |
| `config/*.yaml` | Robot/simulation-specific tuning |
| `launch/state_estimation.launch.py` | Package launch entry point |

The external InEKF library contains the Lie-group propagation, contact-state
augmentation/removal, invariant correction, and covariance update.

## References

The implementation is based on:

- R. Hartley, M. G. Jadidi, J. W. Grizzle, and R. M. Eustice,
  “Contact-Aided Invariant Extended Kalman Filtering for Legged Robot State
  Estimation,” *Robotics: Science and Systems*, 2018.
  <https://doi.org/10.15607/RSS.2018.XIV.050>
- A. Barrau and S. Bonnabel, “The Invariant Extended Kalman Filter as a Stable
  Observer,” *IEEE Transactions on Automatic Control*, vol. 62, no. 4,
  pp. 1797–1812, 2017.
  <https://arxiv.org/abs/1410.1465>

