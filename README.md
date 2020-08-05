# PTU Control

## Overview

This package makes the control of a pan tilt unit (PTU) easy. It finds the pan and tilt joints from a robot description based on the name of the joints. Later features such as automatic panning, position control and more shall be implemented using actions.

**Keywords:** PTU, control, camera

### License

The source code is released under a [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.en.html).

**Author: Miro Voellmy<br />
Affiliation: [European Space Agency](https://www.esa.int/)<br />
Maintainer: Miro Voellmy, miro.voellmy@esa.int**

The PTU Control package has been tested under [ROS2] Foxy Fitzroy and Ubuntu 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- ([rover_msgs]) (message definitions for ESA-PRL rovers)
- ([rover_config]) (config files and models for ESA-PRL rovers)
- [urdf](http://wiki.ros.org/urdf)


#### Building

To build from source, clone the latest version from this repository into your ros2 workspace and compile the package using

	cd ros2_ws/src
	git clone https://github.com/esa-prl/ptu_control.git
	cd ../
	colcon build

## Usage

Run the main node with

    ros2 run ptu_control ptu_control_node --ros-args--params-file PATH_TO_PARAMSFILE

## Config files

* **ptu.yaml** Contains the path to the model and the identifiers for the pan and tilt joint. This is only an example file. Have a look at [rover_config] to see how ptu is used in context.

## Nodes

### ptu_control_node

Reads ptu velocity commands and sends motor commands.


#### Subscribed Topics

* **`/ptu_cmd`**  ([geometry_msgs/Twist])

	The desired PTU speeds.
        * lin.x, .y, .z - Not applicable
        * ang.x: roll velocity - Not applicable
        * ang.y: tilt velocity
        * ang.z: pan velocity


#### Published Topics
* **`/joint_cmds`** ([rover_msgs/JointCommandArray])

    Desired joint positions and velocities of ptu.


#### Parameters

* **`robot_description`** (string, default: "-")

	The name of the robot description (`*.urdf`) file.


* **`pan_joint_identifier`** (string, default: "PAN")

    String which is used to identify pan joint.


* **`tilt_joint_identifier`** (string, default: "TLT")

    String which is used to identify tilt joint.

## Bugs & Feature Requests

Please report bugs and request features using the github issue tracker.


[ROS2]: http://www.ros.org
[rover_msgs]: https://github.com/esa-prl/rover_msgs
[rover_config]: https://github.com/esa-prl/rover_config.git
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
[geometry_msgs/Twist]: https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
[rover_msgs/JointCommandArray]: https://github.com/esa-prl/rover_msgs/blob/master/msg/JointCommandArray.msg
