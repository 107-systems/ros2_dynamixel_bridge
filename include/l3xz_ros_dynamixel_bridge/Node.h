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

#include <l3xz_ros_dynamixel_bridge/srv/set_mode.hpp>

#include "MX28ARSingle.h"
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
  std::map<dynamixelplusplus::Dynamixel::Id, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> _angle_deg_pub;
  std::map<dynamixelplusplus::Dynamixel::Id, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> _angle_vel_sub;
  std::map<dynamixelplusplus::Dynamixel::Id, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> _angle_deg_sub;

  rclcpp::Service<l3xz_ros_dynamixel_bridge::srv::SetMode>::SharedPtr _set_mode_srv;

  class ServoConfiguration
  {
  public:
    ServoConfiguration(MX28AR::OperatingMode const op_mode_,
                       float const initial_target_angle_deg_,
                       float const target_angular_velocity_dps_,
                       float const target_angle_deg_)
    : op_mode{op_mode_}
    , initial_target_angle_deg{initial_target_angle_deg_}
    , target_angular_velocity_dps{target_angular_velocity_dps_}
    , target_angle_deg{target_angle_deg_}
    { }

    MX28AR::OperatingMode op_mode;
    float const initial_target_angle_deg;
    float target_angular_velocity_dps;
    float target_angle_deg;
  };

  std::map<dynamixelplusplus::Dynamixel::Id, std::shared_ptr<ServoConfiguration>> _mx28_cfg_map;
  std::map<dynamixelplusplus::Dynamixel::Id, std::shared_ptr<MX28AR::Single>> _mx28_ctrl_map;

  std::shared_ptr<MX28AR::SyncGroup> _mx28_sync_ctrl;

  std::chrono::steady_clock::time_point _prev_io_loop_timepoint;
  static std::chrono::milliseconds constexpr IO_LOOP_RATE{10};
  rclcpp::TimerBase::SharedPtr _io_loop_timer;
  void io_loop();

  void set_mode(std::shared_ptr<l3xz_ros_dynamixel_bridge::srv::SetMode::Request> const request, std::shared_ptr<l3xz_ros_dynamixel_bridge::srv::SetMode::Response> response);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */

#endif /* L3XZ_HEAD_CTRL_NODE_H_ */
