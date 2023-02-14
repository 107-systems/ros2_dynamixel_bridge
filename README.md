<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_io_dynamixel`
=================================
[![Build Status](https://github.com/107-systems/l3xz_io_dynamixel/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/l3xz_io_dynamixel/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/l3xz_io_dynamixel/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/l3xz_io_dynamixel/actions/workflows/spell-check.yml)

This repository contains the `l3xz_io_dynamixel` node which allows [L3X-Z](https://github.com/107-systems/l3xz)'s other ROS nodes to communicate with its Robotis Dynamixel servos.

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

#### How-to-build
```bash
colcon_ws/src$ git clone https://github.com/107-systems/l3xz_io_dynamixel
colcon_ws$ source /opt/ros/galactic/setup.bash
colcon_ws$ colcon build --packages-select l3xz_io_dynamixel
```

#### How-to-run
```bash
colcon_ws$ . install/setup.bash
colcon_ws$ ros2 launch l3xz_io_dynamixel io_dynamixel.py
```

#### Interface Documentation
##### Subscribed Topics
| Default name | Type |
|:-:|:-:|
| `/l3xz/io/cmd_vel_head` | [`HeadVelocity`](msg/HeadVelocity.msg) |

##### Parameters
| Name | Default | Description |
|:-:|:-:|-|
| `serial_port` | `/dev/ttyUSB0` | Serial port of RS485 bus. |
| `serial_port_baudrate` | 115200 | Serial baud rate of RS485 bus. |
| `pan_servo_id` | 7 | Dynamixel ID of pan servo. |
| `tilt_servo_id` | 8 | Dynamixel ID of tilt servo. |
| `pan_servo_initial_angle` | 180.0 | |
| `pan_servo_min_angle` | 170.0 | |
| `pan_servo_max_angle` | 190.0 | |
| `tilt_servo_initial_angle` | 180.0 | |
| `tilt_servo_min_angle` | 170.0 | |
| `tilt_servo_max_angle` | 190.0 | |
