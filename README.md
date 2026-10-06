# admittance_control

ROS-free, frame-agnostic admittance for limbed climbing robots: the diagonal M-D-K dynamics

$$M \ddot{x} + D \dot{x} + K x = u$$

of a 3-D offset $x$ from a reference, driven by a force or moment $u$ in the same axes.
The caller picks the frame and composes the offset with its reference. It also bounds the offset
and its rate and applies an input deadband.

## Prerequisites

* Ubuntu 22.04, ROS 2 Humble (colcon / ament_cmake only; the library itself uses Eigen3 alone)

## Build

Add this repository to a colcon workspace (e.g. as a git submodule), then:

```bash
colcon build --packages-select admittance_control
colcon test --packages-select admittance_control && colcon test-result --verbose
```

## Use

```cmake
find_package(admittance_control REQUIRED)
target_link_libraries(my_target admittance_control::admittance_control)
```

```cpp
#include <admittance_control/admittance_control.hpp>

admittance_control::Params params;  // every field is required; unset fields fail isValid()
params.mass.setConstant(2.0);
params.damping.setConstant(65.0);
params.stiffness.setConstant(500.0);
params.deadband = 0.2;
params.max_offset = 0.02;
params.max_velocity = 0.1;
admittance_control::AdmittanceControl translation(params);

if (translation.update(force_in_reference, dt)) {
  target.translation() += reference.linear() * translation.offset();
}
```
