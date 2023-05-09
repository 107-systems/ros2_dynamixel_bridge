<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `ros2_dynamixel_bridge`
=====================================
[![Build Status](https://github.com/107-systems/ros2_dynamixel_bridge/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/ros2_dynamixel_bridge/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/ros2_dynamixel_bridge/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/ros2_dynamixel_bridge/actions/workflows/spell-check.yml)

This package provides the interface between [ROS2](https://github.com/ros2) and [Robotis Dynamixel](https://www.robotis.us/dynamixel/) servos.

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

#### How-to-build
```bash
cd $COLCON_WS/src
git clone --recursive https://github.com/107-systems/ros2_dynamixel_bridge
cd $COLCON_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_dynamixel_bridge
```

#### How-to-run
```bash
cd $COLCON_WS
. install/setup.bash
ros2 launch ros2_dynamixel_bridge bridge-default.py
```
or
```bash
colcon_ws$ ros2 launch ros2_dynamixel_bridge bridge-l3xz.py
```
Upon start-up the ROS2 node will scan the connected bus and automatically create topics for each discovered [Robotis Dynamixel](https://www.robotis.us/dynamixel/) servo.

#### Interface Documentation
##### Subscribed Topics
| Default name | Type | Description |
|:-:|:-:|-|
| `/dynamixel/servo_1/angle/target` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) | Servo **#1** target angle / rad |
| `/dynamixel/servo_1/angular_velocity/target` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) | Servo **#1** target angular velocity / rad/sec |
| `/dynamixel/servo_1/mode/set` | [`msg/Mode.msg`](msg/Mode.msg) |  Servo **#1** operation mode (Position Control / Angular Velocity Control) |
| `/dynamixel/servo_2/angle/target` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) | Servo **#2** target angle / rad |
| `/dynamixel/servo_2/angular_velocity/target` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) | Servo **#2** target angular velocity / rad/sec |
| `/dynamixel/servo_2/mode/set` | [`msg/Mode.msg`](msg/Mode.msg) |  Servo **#2** operation mode (Position Control / Angular Velocity Control) |
| ... | ... | ... |
| `/dynamixel/servo_n/angle/target` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) | Servo **#n** target angle / rad |
| `/dynamixel/servo_n/angular_velocity/target` | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) | Servo **#n** target angular velocity / rad/sec |
| `/dynamixel/servo_n/mode/set` | [`msg/Mode.msg`](msg/Mode.msg) |  Servo **#n** operation mode (Position Control / Angular Velocity Control) |

##### Published Topics
|              Default name               |                                      Type                                      | Description                                             |
|:---------------------------------------:|:------------------------------------------------------------------------------:|---------------------------------------------------------|
| `/l3xz/ros2_dynamixel_bridge/heartbeat` |  [`std_msgs/UInt64`](https://docs.ros2.org/foxy/api/std_msgs/msg/UInt64.html)  | Heartbeat signal containing the node uptime in seconds. |
|    `/dynamixel/servo_1/angle/actual`    | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) | Servo **#1** current angle / rad                        |
|    `/dynamixel/servo_2/angle/actual`    | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) | Servo **#1** current angle / rad                        |
|                   ...                   |                                      ...                                       | ...                                                     |
|    `/dynamixel/servo_n/angle/actual`    | [`std_msgs/Float32`](https://docs.ros2.org/foxy/api/std_msgs/msg/Float32.html) | Servo **#n** current angle / rad                        |

##### Parameters
| Name | Default | Description |
|:-:|:-:|-|
| `serial_port` | `/dev/ttyUSB0` | Serial port of RS485 bus. |
| `serial_port_baudrate` | 2 Mbps | Serial baud rate of RS485 bus. |
| `required_node_id_list` | {1, 2, 3, 4, 5, 6, 7, 8} | A list of required Dynamixel servo IDs to be uncovered during startup. |
| `check_required_node_id_list` | `True` | If this parameter is `True` then the discovered servo IDs are compared with the `required_node_id_list` during startup. |
