/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io_dynamixel/graphs/contributors.
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

#include <l3xz_io_dynamixel/msg/head_velocity.hpp>

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
  dynamixelplusplus::Dynamixel::Id _pan_servo_id, _tilt_servo_id;
  float _pan_angular_velocity_rad_per_sec, _tilt_angular_velocity_rad_per_sec;

  rclcpp::Subscription<l3xz_io_dynamixel::msg::HeadVelocity>::SharedPtr _head_io_sub;
  rclcpp::TimerBase::SharedPtr _io_loop_timer;

  std::shared_ptr<MX28AR::SyncGroup> _mx28_head_sync_ctrl;

  std::chrono::steady_clock::time_point _prev_io_loop_timepoint;
  static std::chrono::milliseconds constexpr IO_LOOP_RATE{10};
  void io_loop();

  static int                              constexpr DEFAULT_SERIAL_BAUDRATE          = 115200;
  static dynamixelplusplus::Dynamixel::Id constexpr DEFAULT_PAN_SERVO_ID             = 7;
  static dynamixelplusplus::Dynamixel::Id constexpr DEFAULT_TILT_SERVO_ID            = 8;
  static float                            constexpr DEFAULT_PAN_SERVO_INITIAL_ANGLE  = 180.0f;
  static float                            constexpr DEFAULT_PAN_SERVO_MIN_ANGLE      = DEFAULT_PAN_SERVO_INITIAL_ANGLE - 10.f;
  static float                            constexpr DEFAULT_PAN_SERVO_MAX_ANGLE      = DEFAULT_PAN_SERVO_INITIAL_ANGLE + 10.f;
  static float                            constexpr DEFAULT_TILT_SERVO_INITIAL_ANGLE = 180.0f;
  static float                            constexpr DEFAULT_TILT_SERVO_MIN_ANGLE     = DEFAULT_TILT_SERVO_INITIAL_ANGLE - 10.f;
  static float                            constexpr DEFAULT_TILT_SERVO_MAX_ANGLE     = DEFAULT_TILT_SERVO_INITIAL_ANGLE + 10.f;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */

#endif /* L3XZ_HEAD_CTRL_NODE_H_ */
