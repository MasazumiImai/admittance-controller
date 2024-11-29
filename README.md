# limbed_climbing_robot_admittance_controller

Admittance controller for limbed-climbing robots.

## Prerequisites

* Ubuntu 22.04
* ROS 2 Humble
* Eigen3

## Installation

* Install Ubuntu 22.04
* Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
* Clone limbed_climbing_robot_admittance_controller repository

  ```bash
  mkdir -p ~/path/to/workspace/src
  cd ~/path/to/workspace
  git clone git@github.com:MasazumiImai/limbed_climbing_robot_admittance_controller.git
  ```

* Build limbed_climbing_robot_admittance_controller workspace

  ```bash
  cd ~/path/to/workspace
  colcon build --symlink-install
  source install/setup.bash
  ```
