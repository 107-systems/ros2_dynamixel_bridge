/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_dynamixel_bridge/graphs/contributors.
 */

#ifndef L3XZ_HEAD_CTRL_NODE_H_
#define L3XZ_HEAD_CTRL_NODE_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <dynamixel++/dynamixel++.h>

#include <std_msgs/msg/float32.hpp>

#include <l3xz_ros_dynamixel_bridge/msg/head_velocity.hpp>

#include "MX28ARSyncGroup.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::head
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
   Node();
  ~Node();

private:
  enum class Servo
  {
    Coxa_Left_Front,
    Coxa_Left_Middle,
    Coxa_Left_Back,
    Coxa_Right_Front,
    Coxa_Right_Middle,
    Coxa_Right_Back,
    Pan,
    Tilt,
  };

  std::map<Servo,
           rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> _angle_pub;

  l3xz_ros_dynamixel_bridge::msg::HeadVelocity _head_vel_msg;
  rclcpp::Subscription<l3xz_ros_dynamixel_bridge::msg::HeadVelocity>::SharedPtr _head_vel_sub;

  std::shared_ptr<MX28AR::HeadSyncGroup> _mx28_head_sync_ctrl;
  std::shared_ptr<MX28AR::CoxaSyncGroup> _mx28_coxa_sync_ctrl;

  std::chrono::steady_clock::time_point _prev_io_loop_timepoint;
  static std::chrono::milliseconds constexpr IO_LOOP_RATE{10};
  rclcpp::TimerBase::SharedPtr _io_loop_timer;
  void io_loop();

  void declare_parameter_all();
  void init_pan_tilt_servos();
  void init_coxa_servos();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */

#endif /* L3XZ_HEAD_CTRL_NODE_H_ */
