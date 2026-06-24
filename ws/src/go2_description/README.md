# Go2 description

This package contains the checked-in URDF and mesh assets used to simulate the
Unitree Go2 robot.

The Drake simulator loads:

```text
package://go2_description/urdf/go2_description.urdf
```

The simulation model is self-contained and does not require the Unitree ROS 2
SDK, Unitree message packages, motor drivers, or Gazebo plugins.

The model originated from the existing `common/model/urdf/go2` tree and retains
its declared BSD license and accompanying model documentation.
