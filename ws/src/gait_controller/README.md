# Gait Controller

The `gait_controller` package generates the finite-horizon locomotion plan used by the quadruped control stack. It converts the current robot state and a body-motion target into:

- a future stance/swing schedule for every leg;
- the remaining and total swing time of every leg;
- future footholds in the world frame;
- a reference body pose, velocity, and angular-velocity trajectory;
- a `KEEP` or `MOVE` locomotion mode.

These values are stored in a `GaitSequence`. In `MITController`, that sequence is the common plan shared by the model predictive controller (MPC), swing-leg controller (SLC), whole-body controller (WBC), contact-recovery state machine, and optional model-adaptation layer.

The package does not directly command the robot. It defines **where the body should move, which feet may exert force, and where swinging feet should land**. The downstream controllers turn that plan into ground-reaction forces, swing trajectories, and actuator commands.

## Contents

- [Role in the locomotion stack](#role-in-the-locomotion-stack)
- [Timing and leg conventions](#timing-and-leg-conventions)
- [The `GaitSequence` data model](#the-gaitsequence-data-model)
- [Reference trajectory generation](#reference-trajectory-generation)
- [Gait algorithms](#gait-algorithms)
  - [Fixed periodic gait](#fixed-periodic-gait)
  - [Adaptive gait](#adaptive-gait)
  - [Bio-inspired gait](#bio-inspired-gait)
- [Early-contact handling inside the gait generators](#early-contact-handling-inside-the-gait-generators)
- [Raibert foothold planning](#raibert-foothold-planning)
- [How `MITController` uses the gait controller](#how-mitcontroller-uses-the-gait-controller)
- [Configuration](#configuration)
- [Published diagnostics](#published-diagnostics)
- [Source map](#source-map)
- [Current limitations and implementation notes](#current-limitations-and-implementation-notes)
- [Build](#build)

## Role in the locomotion stack

At each MPC update, the controller executes the following pipeline:

```text
QuadControlTarget + QuadState + QuadModel
                  |
                  v
    GaitReferenceTrajectoryPlanner
    - body pose/velocity horizon
    - KEEP or MOVE mode
                  |
                  v
          GaitSequencer
    - contact/swing horizon
    - swing timing
    - Raibert footholds
                  |
                  v
             GaitSequence
       /          |           \
      v           v            v
     MPC          SLC    contact/WBC logic
 ground forces  swing paths  final leg commands
```

The ordering is important. `MITController::MPCLoopCallback()` first calls:

```cpp
mpc_tp_->plan_trajectory(gait_sequence, target);
gs_->GetGaitSequence(gait_sequence);
```

The gait algorithms and Raibert planner therefore operate on an already populated reference body trajectory.

## Timing and leg conventions

The relevant constants are defined in `common/constants.hpp`:

| Quantity | Value | Meaning |
| --- | ---: | --- |
| `N_LEGS` | 4 | Number of legs |
| `GAIT_SEQUENCE_SIZE` | 100 | Internal gait horizon samples |
| `MPC_PREDICTION_HORIZON` | 10 | MPC prediction intervals |
| `MPC_DT` | 0.05 s | Spacing of gait/MPC horizon samples |
| `MPC_CONTROL_DT` | 0.01 s | Period of the MPC callback |
| `SWING_LEG_DT` | 0.002 s | Period of the swing-leg callback |
| `CONTROL_DT` | 0.002 s | Period of the WBC/command callback |

The arrays are ordered consistently across the controller. The configured shoulder positions in `MITController` establish the practical leg ordering; the standard configuration uses:

1. front left;
2. front right;
3. back left;
4. back right.

All positions in `GaitSequence` are represented in the world frame unless explicitly stated otherwise.

### Phase convention

A periodic gait is described by:

- period \(T\);
- duty factor \(d_i\) for leg \(i\);
- phase offset \(o_i\) for leg \(i\);
- horizon spacing \(\Delta t\).

For global gait phase \(\phi\), sample \(k\), and leg \(i\), the leg phase is:

\[
\phi_i(k) =
\operatorname{fmod}
\left(
\phi + 1 - o_i + \frac{k\Delta t}{T},
1
\right).
\]

The leg is in contact when:

\[
c_i(k) = \phi_i(k) < d_i.
\]

Consequently:

- \(d_i = 1\) means permanent stance;
- \(d_i = 0\) means permanent flight;
- \(0 < d_i < 1\) divides each cycle into stance followed by swing.

The nominal durations are:

\[
t_{\text{stance},i} = d_iT,
\qquad
t_{\text{swing},i} = (1-d_i)T.
\]

During swing, the remaining swing time is:

\[
t_{\text{remaining},i}(k) = (1-\phi_i(k))T.
\]

## The `GaitSequence` data model

`GaitSequence` is the central interface of this package.

| Field | Meaning | Main consumers |
| --- | --- | --- |
| `time_stamp` | State time at which the plan was generated | SLC timing |
| `target_position` | Current commanded body position | Adaptive gait, diagnostics |
| `target_velocity` | Current commanded body velocity | Adaptive gait, foothold planning |
| `target_orientation` | Current commanded body orientation | Diagnostics |
| `target_twist` | Current commanded angular velocity | Adaptive gait, foothold planning |
| `target_height` | Commanded height above the lowest known contact | Adaptive gait and diagnostics |
| `current_height` | Estimated current height above the lowest known contact | Adaptive gait |
| `contact_sequence[k][leg]` | Planned contact state | MPC, SLC, WBC/contact logic |
| `foot_position_sequence[k][leg]` | Current or future stance foothold | SLC and WBC |
| `reference_trajectory_position[k]` | Body position reference | MPC and foothold planner |
| `reference_trajectory_orientation[k]` | Body orientation reference | MPC and foothold planner |
| `desired_reference_trajectory_position[k]` | Stabilized/latched position reference | Adaptive standing correction |
| `desired_reference_trajectory_orientation[k]` | Stabilized/latched orientation reference | Adaptive standing correction |
| `reference_trajectory_velocity[k]` | Body linear-velocity reference | MPC, gait adaptation, footholds |
| `reference_trajectory_twist[k]` | Body angular-velocity reference | MPC, gait adaptation, footholds |
| `swing_time_sequence[k][leg]` | Remaining swing time | SLC and contact recovery |
| `gait_swing_time[leg]` | Full nominal swing duration | SLC trajectory timing |
| `sequence_mode` | `KEEP` or `MOVE` | MPC weight selection |

Although the internal sequence contains 100 samples, the ROS `interfaces/msg/GaitSequence` diagnostic message publishes the first 11 samples: the current sample plus the 10-step MPC horizon.

## Reference trajectory generation

`GaitReferenceTrajectoryPlanner` fills the body-reference portion of `GaitSequence`.

### Initial sample

Sample zero is copied from the measured state:

- world position;
- world orientation;
- world linear velocity;
- world angular velocity.

The planner also updates its estimate of the ground height from feet currently reported in contact.

### Future samples

For every future horizon sample, the planner applies the active fields of `Target`.

Command priorities are:

1. explicit position overrides integrated velocity;
2. explicit orientation overrides integrated angular velocity;
3. hybrid-frame planar velocity overrides world-frame planar velocity.

Hybrid planar commands are integrated using the yaw frame between the previous and next orientations. This lets a forward command follow the robot's heading instead of a fixed world axis.

The `QuadControlTarget` mapping used by `MITController` is:

| `QuadControlTarget` field | Internal target |
| --- | --- |
| `body_x_dot` | `hybrid_x_dot` |
| `body_y_dot` | `hybrid_y_dot` |
| `world_z` | `z` |
| `hybrid_theta_dot` | `wz` |
| `roll` | `roll` |
| `pitch` | `pitch` |

### Desired trajectory and standing-pose latching

The planner produces both a raw `reference_trajectory_*` and a `desired_reference_trajectory_*`.

When `fix_standing_position` is disabled, these trajectories are identical. When enabled, the desired trajectory can retain the previous target pose after velocity commands return to zero. The distance, angular, and velocity thresholds prevent small estimation changes from making the standing target drift continuously.

### `KEEP` and `MOVE`

The planner selects `MOVE` when a nonzero planar velocity or yaw-rate command is active. Otherwise it selects `KEEP`.

`MITController` uses this mode to switch between `mpc_state_weights_move` and `mpc_state_weights_stand`. A fixed all-stance gait can also force `KEEP`.

## Gait algorithms

All sequencers implement `GaitSequencerInterface` and own:

- a private state copy;
- a private model copy;
- the current `Target`;
- a `RaibertFootStepPlanner`.

They expose a common set of operations:

- update state;
- update model;
- update target;
- generate a `GaitSequence`;
- report a compact `GaitState`;
- report the sequencer type.

### Fixed periodic gait

`SimpleGaitSequencer` wraps a `Gait` with fixed period, duty factors, and offsets.

#### State update

The global phase is calculated from elapsed state time:

\[
\phi = \operatorname{fmod}
\left(
\frac{t-t_0}{T},
1
\right).
\]

Pure standing or pure flight gaits do not advance the global phase. Their phase is held at zero.

#### Horizon generation

For every horizon sample and leg, `Gait::update_sequence()`:

1. computes the leg phase;
2. determines contact from the duty factor;
3. optionally incorporates an early measured contact;
4. calculates remaining swing time;
5. stores the full nominal swing duration.

The sequencer then asks the Raibert planner to populate future footholds and stamps the sequence with the state time.

#### Built-in gait database

| Gait | Period [s] | Duty factor | Phase offsets `[FL, FR, BL, BR]` |
| --- | ---: | ---: | --- |
| `STAND` | 0.50 | 1.00 | `[0.0, 0.0, 0.0, 0.0]` |
| `STATIC_WALK` | 1.25 | 0.80 | `[0.0, 0.5, 0.75, 0.25]` |
| `WALKING_TROT` | 0.50 | 0.60 | `[0.0, 0.5, 0.5, 0.0]` |
| `TROT` | 0.50 | 0.50 | `[0.0, 0.5, 0.5, 0.0]` |
| `FLYING_TROT` | 0.40 | 0.40 | `[0.0, 0.5, 0.5, 0.0]` |
| `PACE` | 0.35 | 0.50 | `[0.0, 0.5, 0.0, 0.5]` |
| `BOUND` | 0.40 | 0.40 | `[0.0, 0.0, 0.5, 0.5]` |
| `ROTARY_GALLOP` | 0.40 | 0.20 | `[0.0, 0.8571, 0.3571, 0.5]` |
| `TRAVERSE_GALLOP` | 0.50 | 0.20 | `[0.0, 0.8571, 0.3571, 0.5]` |
| `PRONK` | 0.50 | 0.50 | `[0.0, 0.0, 0.0, 0.0]` |

`MITController` also supports a `Manual` fixed gait whose period, duty factors, and phase offsets come directly from parameters.

### Adaptive gait

`AdaptiveGaitSequencer` wraps `AdaptiveGait`. Instead of using one fixed period and duty factor, it adapts them online from velocity, height, turning, commanded motion, and optional foothold correction.

#### Effective locomotion speed

The current planar reference velocity is moving-average filtered. The algorithm then forms an effective speed:

\[
v_{\text{eff}} =
\max\left(
\lVert \bar{v}_{xy} \rVert,
0.3|\omega_z|,
\alpha_v \lVert v_{\text{cmd},xy} \rVert,
v_{\min}
\right)
+ k_d d,
\]

where:

- \(\alpha_v\) is `min_v_cmd_factor`;
- \(k_d\) is `disturbance_correction`;
- \(d\) is the amount by which measured body-frame speed exceeds commanded speed;
- \(v_{\min}\) is `min_v`.

The `0.3` turning-radius factor is currently hard-coded.

#### Froude number and stride length

The dimensionless Froude number is:

\[
Fr = \frac{v_{\text{eff}}^2}{gh}.
\]

The nominal stride length is:

\[
L = 2.3 Fr^{0.3} h.
\]

If `max_stride_length` is positive, \(L\) is clipped to that value.

For motion above `zero_velocity_threshold`, the period is:

\[
T = \frac{L}{|v_{\text{eff}}|}.
\]

The swing duration is kept approximately constant, so each duty factor becomes:

\[
d_i = \frac{T-t_{\text{swing}}}{T}.
\]

This makes stance duration and step frequency adapt to speed while preserving the configured swing time.

#### Offset switching

When `switch_offsets` is enabled, the adaptive gait switches between:

- static-walk offsets: `[0.0, 0.5, 0.75, 0.25]`;
- trot offsets: `[0.0, 0.5, 0.5, 0.0]`.

The selection uses `gait_change_froude`. Separate thresholds are used depending on the current offset family, which provides hysteresis. `offset_delay` can postpone applying a newly selected offset set.

#### Standing foot correction

Below `zero_velocity_threshold`, the adaptive gait normally sets every duty factor to `1.0`.

If a Raibert foothold for a leg is farther than `standing_foot_position_threshold` from the current foot position, that leg receives a correction step:

- period becomes `correction_period`;
- duty factor becomes `(correction_period - swing_time) / correction_period`.

With `correct_all = true`, all legs participate when any one leg requires correction. Otherwise only the displaced legs step.

#### Phase continuity

When duty factors change, `nextPhase()` remaps the old stance and swing sub-intervals into the new ones. This avoids resetting every leg whenever the period or duty factor changes.

It also:

- advances per-leg phases;
- incorporates early measured contact;
- estimates current phase-offset error;
- gradually corrects offsets over at most `max_correction_cycles`;
- performs those corrections during stance to avoid jumping a leg unexpectedly into swing.

The current adaptive state is exposed through `GaitState`: period, duty factors, offsets, phases, and contacts.

### Bio-inspired gait

The package also implements `BioGaitSequencer` and `BioInspiredGait`.

It selects a gait from the Froude number:

| Froude range | Gait |
| --- | --- |
| `< 0.0009` | stand |
| `< 0.0024` | static walk |
| `< 0.1517` | walking trot |
| `< 0.74` | trot |
| otherwise | flying trot |

When the selected gait changes, period, duty factors, and phase offsets are blended using transition weights. Phase-offset differences are wrapped onto the shortest interval so transitions do not unnecessarily rotate almost a full cycle.

This implementation is part of the library, but `MITController::GetGaitSequencerFromParams()` currently creates only `SimpleGaitSequencer` or `AdaptiveGaitSequencer`. Selecting the bio-inspired sequencer requires adding it to that factory and its parameter schema.

## Early-contact handling inside the gait generators

When enabled, the fixed gait generator accepts a measured contact during the second half of a planned swing. It then marks that foot as being in contact for the rest of the current swing interval, until the first nominal stance sample.

The adaptive gait instead resets the affected leg phase to the beginning of stance when contact is detected in the second half of swing.

This planning-level handling is separate from the faster `MITController` contact state machine. The gait generator makes subsequent MPC plans aware of the touchdown, while the fast loop reacts immediately by modifying WBC inputs.

## Raibert foothold planning

Every sequencer uses `RaibertFootStepPlanner` to convert the contact schedule and body reference into future stance locations.

For leg \(i\), the nominal foothold is:

\[
p_{\text{foot},i} =
p_{\text{shoulder},i}
+ p_{\text{symmetry}}
+ p_{\text{centrifugal}}.
\]

### Shoulder term

\[
p_{\text{shoulder},i} =
p_{\text{body}} + R_{\text{yaw}}p_{\text{shoulder},i}^{B}.
\]

Only the reference yaw is used when projecting the shoulder into the world frame.

### Symmetry and velocity-feedback term

\[
p_{\text{symmetry}} =
\frac{t_{\text{stance}}}{2}v
+ k(v-v_{\text{cmd}}).
\]

The first term places the foot ahead of the body by half a stance traversal. The second term corrects velocity error using the configurable Raibert gain `k`.

### Turning term

\[
p_{\text{centrifugal}} =
\frac{1}{2}\sqrt{\frac{h}{g}}
\left(v \times \omega_{\text{cmd}}\right).
\]

This shifts the foothold during turning. The estimated height is clamped between zero and the maximum reachable leg length.

### Horizon behavior

For each leg:

- at sample zero in stance, the measured world foot position is used;
- while stance continues, the same foothold is retained;
- at the first stance sample after swing, a new foothold is calculated;
- during swing, the sequence stores a zero placeholder because the SLC interpolates between the last stance point and the next planned stance point.

The planner clips horizontal placement when the requested shoulder-to-foot distance exceeds the estimated maximum leg length.

When `raibert.z_on_plane` is enabled, a plane is fitted through the four most recently observed stance-foot positions and new foothold heights are projected onto it. Otherwise each leg reuses its last contact height.

## How `MITController` uses the gait controller

### 1. Construction

`MITController::GetGaitSequencerFromParams()` creates either:

- `SimpleGaitSequencer`, with a built-in or manual gait; or
- `AdaptiveGaitSequencer`, with online timing and offset adaptation.

The same factory passes:

- shoulder positions;
- a copy of the current model and state;
- Raibert parameters;
- the early-contact option.

A separate `GaitReferenceTrajectoryPlanner` is built from the standing-pose parameters.

### 2. Target and state update

The latest `QuadControlTarget` is converted to the internal `Target`. At the start of every MPC callback, the gait sequencer and MPC receive a copy of the newest `QuadState`.

The sequencer stores private state/model copies because its Raibert planner keeps references to those objects.

### 3. Gait and reference planning

The MPC callback:

1. fills the body reference trajectory;
2. updates contact and swing timing;
3. calculates footholds;
4. sends the complete sequence to MPC.

The sequence is generated every `MPC_CONTROL_DT`, while its samples are spaced by `MPC_DT`.

### 4. MPC use

MPC consumes:

- body pose and velocity references;
- the contact schedule;
- future foothold information available through the shared sequence.

The contact schedule determines which feet may carry ground-reaction forces at every horizon sample. The MPC output is a wrench sequence and predicted body-state trajectory.

When `sequence_mode` changes, `MITController` swaps the MPC state weights between standing and moving configurations.

### 5. Swing-leg controller use

The SLC searches each leg's contact horizon for:

- the first swing sample;
- the first subsequent stance sample.

It uses:

- the previous stance foothold as the swing start;
- the next stance foothold as the swing target;
- `gait_swing_time` as the total trajectory duration;
- `swing_time_sequence[0]` to determine current swing progress;
- the gait sequence timestamp to compensate for elapsed time.

It generates position, velocity, and acceleration targets using the configured swing trajectory. A target may be revised only until `maximum_swing_leg_progress_to_update_target`.

### 6. Fast contact-recovery and WBC use

The fast control callback starts with:

- `contact_sequence[0]`;
- `foot_position_sequence[0]`;
- the first MPC wrench sample;
- SLC foot targets.

It reconciles planned and measured contact using five states:

| State | Meaning | Main action |
| --- | --- | --- |
| `STANCE` | Planned support | Use stance foothold and MPC wrench |
| `SWING` | Planned flight | Use SLC target and zero the wrench |
| `EARLY_CONTACT` | Touchdown before planned stance | Hold touchdown, mark contact, use transformed future stance wrench |
| `LATE_CONTACT` | Planned stance without measured contact | Hold body-relative foot pose and zero the wrench |
| `LOST_CONTACT` | Contact disappeared during stance | Currently handled like late contact |

For early contact, the next planned stance index is derived from:

\[
k_{\text{next stance}} =
\left\lfloor
\frac{t_{\text{remaining swing}}}{MPC\_DT}
\right\rfloor + 1.
\]

The future stance wrench is rotated from its future reference orientation into the current measured orientation before being applied.

Finally, WBC receives a coherent set of:

- effective contacts;
- foot position/velocity/acceleration targets;
- desired foot wrenches;
- the next MPC body prediction at index 1.

It converts these targets into Cartesian or joint commands for the leg driver.

### 7. Model adaptation

When model adaptation is enabled, the active `GaitSequence` is provided to the estimator. If the model is updated, the new model is propagated to:

- MPC;
- gait sequencer and Raibert planner;
- WBC;
- SLC.

This keeps planning and control consistent with the adapted mass and inertia parameters.

### 8. Runtime parameter updates

`MITController` supports live updates of adaptive gait parameters. The current `AdaptiveGait` object is modified through setters for:

- phase offsets;
- swing time;
- filter size;
- velocity and Froude thresholds;
- standing correction;
- phase correction;
- disturbance compensation;
- offset delay;
- maximum stride length.

Changing parameters that require reconstruction causes the gait sequencer and reference trajectory planner to be rebuilt from the latest state and model.

## Configuration

The complete typed parameter schema is in:

```text
controllers/src/mit_controller_parameters.yaml
```

### Sequencer selection

```yaml
gait_sequencer: "Simple"  # or "Adaptive"
```

### Simple gait

Use a database gait:

```yaml
simple_gait_sequencer:
  gait: "TROT"
```

Supported names are:

```text
STAND
STATIC_WALK
WALKING_TROT
TROT
FLYING_TROT
PACE
BOUND
ROTARY_GALLOP
TRAVERSE_GALLOP
PRONK
```

Or define a manual gait:

```yaml
simple_gait_sequencer:
  gait: "Manual"
  manual_gait:
    period: 0.5
    duty_factor: [0.6, 0.6, 0.6, 0.6]
    phase_offset: [0.0, 0.5, 0.5, 0.0]
```

### Adaptive gait

```yaml
gait_sequencer: "Adaptive"

adaptive_gait_sequencer:
  gait:
    swing_time: 0.2
    filter_size: 10
    zero_velocity_threshold: 0.03
    min_v_cmd_factor: 0.5
    min_v: 0.0
    max_stride_length: 0.0
    switch_offsets: true
    phase_offset: [0.0, 0.5, 0.5, 0.0]
    gait_change_froude: [0.02, 0.006]
    offset_delay: 0.0
    standing_foot_position_threshold: 0.08
    correction_period: 0.6
    correct_all: false
    max_correction_cycles: 2.0
    disturbance_correction: 0.0
```

Parameter effects:

| Parameter | Effect |
| --- | --- |
| `swing_time` | Full swing duration retained while period changes |
| `filter_size` | Moving-average window for planar velocity |
| `zero_velocity_threshold` | Boundary between locomotion and standing correction |
| `min_v_cmd_factor` | Lower bound derived from commanded speed |
| `min_v` | Absolute lower bound on effective speed |
| `max_stride_length` | Positive values cap the computed stride |
| `switch_offsets` | Enables static-walk/trot offset switching |
| `phase_offset` | Initial/commanded offsets |
| `gait_change_froude` | Offset-switch thresholds and hysteresis |
| `offset_delay` | Delay before applying a new offset set |
| `standing_foot_position_threshold` | Foothold error that triggers a correction step |
| `correction_period` | Period used for standing correction steps |
| `correct_all` | Moves all feet when one correction is required |
| `max_correction_cycles` | Limits phase-offset correction rate |
| `disturbance_correction` | Adds measured excess velocity to gait speed |

### Raibert planner

```yaml
raibert:
  k: 0.03
  filtersize: 20
  z_on_plane: false
```

| Parameter | Effect |
| --- | --- |
| `k` | Velocity-error feedback gain |
| `filtersize` | Moving-average window for measured velocity |
| `z_on_plane` | Projects future footholds onto the estimated support plane |

The shoulder positions are configured as four consecutive XYZ vectors:

```yaml
gs_shoulder_positions: [
  0.167,  0.1738, 0.0,
  0.167, -0.1738, 0.0,
 -0.197,  0.1738, 0.0,
 -0.197, -0.1738, 0.0
]
```

### Standing reference

```yaml
fix_standing_position: true
fix_position_distance_threshold: 0.1
fix_position_angular_threshold: 0.26
fix_position_velocity_threshold: 0.1
```

### Contact handling

```yaml
early_contact_detection: true
late_contact_detection: false
lost_contact_detection: false
late_contact_reschedule_swing_phase: true
```

`early_contact_detection` affects both gait generation and the fast contact state machine. The late/lost-contact options are handled by `MITController` rather than by the gait package itself.

## Published diagnostics

When the corresponding publishers are enabled, `MITController` publishes:

- `gait_state`: current period, phase, offsets, duty factors, contacts, and sequencer type;
- `gait_sequence`: the first 11 samples of the generated gait horizon;
- the MPC open-loop state prediction;
- swing trajectory start/end vectors;
- WBC targets and solver diagnostics.

`quad_tools/src/plot_gs.cpp` subscribes to the gait-sequence message and can be used as a starting point for visualizing the horizon.

## Source map

| File | Responsibility |
| --- | --- |
| `include/gait_controller/gait_sequence.hpp` | Shared horizon data structure |
| `include/gait_controller/gait_sequencer_interface.hpp` | Common sequencer API and owned planner/state/model |
| `src/gait_controller/gait.cpp` | Fixed, adaptive, and bio-inspired gait mathematics |
| `src/gait_controller/simple_gait_sequencer.cpp` | Fixed-gait sequencing |
| `src/gait_controller/adaptive_gait_sequencer.cpp` | Adaptive-gait sequencing |
| `src/gait_controller/bio_gait_sequencer.cpp` | Froude-based gait selection and transitions |
| `src/gait_controller/raibert_foot_step_planner.cpp` | Future foothold generation |
| `src/gait_controller/gait_reference_trajectory_planner.cpp` | Body reference and `KEEP`/`MOVE` generation |
| `include/gait_controller/gait_sequence_to_msg.hpp` | Conversion to the ROS diagnostic message |
| `controllers/src/mit_controller_node.cpp` | Integration with MPC, SLC, WBC, and contact recovery |
| `controllers/src/mit_controller_parameters.yaml` | Typed parameter definitions |

## Current limitations and implementation notes

- `BioGaitSequencer` is compiled into the library but is not selectable by the current `MITController` factory.
- The adaptive sequencer advances with `MPC_CONTROL_DT` and contains a TODO to use the measured elapsed time.
- The adaptive turning-speed approximation uses a hard-coded radius of `0.3 m`.
- Terrain-plane fitting uses the four last remembered stance-foot positions and does not currently evaluate fit quality.
- Without terrain-plane projection, new footholds reuse the last height recorded for that leg.
- The Raibert reachability correction clips horizontal placement but leaves a FIXME for cases where the requested vertical displacement alone exceeds the maximum leg length.
- The internal horizon is longer than the MPC horizon; only 11 samples are exported in the ROS message.
- Contact handling is split intentionally across two rates: the sequencer modifies future plans, while the fast `MITController` state machine modifies immediate WBC inputs.

## Build

From the workspace root:

```bash
source /opt/ros/rolling/setup.bash
colcon build --packages-select gait_controller controllers
```

To build only this library:

```bash
source /opt/ros/rolling/setup.bash
colcon build --packages-select gait_controller
```

After building:

```bash
source install/setup.bash
```

