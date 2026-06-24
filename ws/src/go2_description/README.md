# Go2 description

This package contains the checked-in URDF and mesh assets used to simulate the
Go2 robot.

The Drake simulator loads:

```text
package://go2_description/urdf/go2_description.urdf
```

The simulation model is self-contained. It does not include or require a
vendor ROS 2 SDK, vendor message packages, motor drivers, Gazebo controllers,
or Gazebo plugins.

Only the files required by the active URDF are installed:

- `urdf/go2_description.urdf`
- the DAE files referenced through relative paths under `dae/`

The model originated from the existing `common/model/urdf/go2` tree and retains
its declared BSD license and accompanying model documentation.
