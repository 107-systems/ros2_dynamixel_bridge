<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_ros_dynamixel_bridge`
=========================================
[![Build Status](https://github.com/107-systems/l3xz_ros_dynamixel_bridge/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/l3xz_ros_dynamixel_bridge/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/l3xz_ros_dynamixel_bridge/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/l3xz_ros_dynamixel_bridge/actions/workflows/spell-check.yml)

This package provides the interface between [ROS](https://github.com/ros2) and [L3X-Z](https://github.com/107-systems/l3xz)'s Robotis Dynamixel servos.

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

#### How-to-build
```bash
colcon_ws/src$ git clone --recursive https://github.com/107-systems/l3xz_ros_dynamixel_bridge
colcon_ws$ source /opt/ros/galactic/setup.bash
colcon_ws$ colcon build --packages-select l3xz_ros_dynamixel_bridge
```

#### How-to-run
```bash
colcon_ws$ . install/setup.bash
colcon_ws$ ros2 launch l3xz_ros_dynamixel_bridge bridge.py
```

#### Interface Documentation
##### Subscribed Topics
| Default name | Type |
|:-:|:-:|
| `/l3xz/leg/left_front/coxa/angular_velocity/target` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |
| `/l3xz/leg/left_middle/coxa/angular_velocity/target` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |
| `/l3xz/leg/left_back/coxa/angular_velocity/target` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |
| `/l3xz/leg/right_front/coxa/angular_velocity/target` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |
| `/l3xz/leg/right_middle/coxa/angular_velocity/target` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |
| `/l3xz/leg/right_back/coxa/angular_velocity/target` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |
| `/l3xz/head/pan/angular_velocity/target` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |
| `/l3xz/head/pan/angular_velocity/target` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |

##### Published Topics
| Default name | Type |
|:-:|:-:|
| `/l3xz/leg/left_front/coxa/angle/actual` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |
| `/l3xz/leg/left_middle/coxa/angle/actual` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |
| `/l3xz/leg/left_back/coxa/angle/actual` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |
| `/l3xz/leg/right_front/coxa/angle/actual` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |
| `/l3xz/leg/right_middle/coxa/angle/actual` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |
| `/l3xz/leg/right_back/coxa/angle/actual` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |
| `/l3xz/head/pan/angle/actual` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |
| `/l3xz/head/pan/angle/actual` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) |

##### Parameters
| Name | Default | Description |
|:-:|:-:|-|
| `serial_port` | `/dev/ttyUSB0` | Serial port of RS485 bus. |
| `serial_port_baudrate` | 115200 | Serial baud rate of RS485 bus. |
| `left_front_coxa_servo_id` | 1 | Dynamixel ID of coxa servo of the left front leg. |
| `left_front_coxa_servo_initial_angle` | 180.0 | |
| `left_front_coxa_servo_min_angle` | 170.0 | |
| `left_front_coxa_servo_max_angle` | 190.0 | |
| ... | ... | ... |
| `tilt_servo_id` | 8 | Dynamixel ID of tilt servo. |
| `tilt_servo_initial_angle` | 180.0 | |
| `tilt_servo_min_angle` | 170.0 | |
| `tilt_servo_max_angle` | 190.0 | |
