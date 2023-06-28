/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_dynamixel_bridge/graphs/contributors.
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
#include <std_msgs/msg/u_int64.hpp>

#include <ros2_heartbeat/publisher/Publisher.h>
#include <ros2_loop_rate_monitor/Monitor.h>

#include <ros2_dynamixel_bridge/msg/mode.hpp>

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
  heartbeat::Publisher::SharedPtr _heartbeat_pub;
  void init_heartbeat();

  std::map<dynamixelplusplus::Dynamixel::Id, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> _angle_actual_rad_pub;
  std::map<dynamixelplusplus::Dynamixel::Id, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> _angle_target_vel_sub;
  std::map<dynamixelplusplus::Dynamixel::Id, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> _angle_target_rad_sub;
  std::map<dynamixelplusplus::Dynamixel::Id, rclcpp::Subscription<ros2_dynamixel_bridge::msg::Mode>::SharedPtr> _mode_sub;

  class ServoConfig
  {
  private:
    MX28AR::OperatingMode _mode;
  public:
    ServoConfig() : _mode{MX28AR::OperatingMode::ExtendedPositionControlMode} { }
    [[nodiscard]] MX28AR::OperatingMode mode() const { return _mode; }
    void set_mode(MX28AR::OperatingMode const mode) { _mode = mode; }
  };

  class ServoTarget
  {
  private:
    float _target_angular_velocity_dps, _target_angle_deg;
  public:
    ServoTarget(float const target_angular_velocity_dps,
                float const target_angle_deg)
    : _target_angular_velocity_dps{target_angular_velocity_dps}
    , _target_angle_deg{target_angle_deg} { }
    [[nodiscard]] float angular_velocity_dps() const { return _target_angular_velocity_dps; }
    [[nodiscard]] float angle_deg() const { return _target_angle_deg; }
    void set_angular_velocity_dps(float const ang_vel) { _target_angular_velocity_dps = ang_vel; }
    void set_angle_deg(float const ang_deg) { _target_angle_deg = ang_deg; }
  };

  typedef struct
  {
    std::shared_ptr<MX28AR::Single> ctrl;
    std::shared_ptr<ServoConfig> cfg;
    std::shared_ptr<ServoTarget> target;
  } ServoMapValue;

  std::map<dynamixelplusplus::Dynamixel::Id, ServoMapValue> _mx28_map;

  std::shared_ptr<MX28AR::SyncGroup> _mx28_sync_ctrl;

  static std::chrono::milliseconds constexpr IO_LOOP_RATE{10};
  loop_rate::Monitor::SharedPtr _io_loop_rate_monitor;
  rclcpp::TimerBase::SharedPtr _io_loop_timer;
  void io_loop();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */

#endif /* L3XZ_HEAD_CTRL_NODE_H_ */
