/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_dynamixel_bridge/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <ros2_dynamixel_bridge/MX28ARSyncGroup.h>

#include <assert.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::MX28AR
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void SyncGroup::setTorqueEnable(TorqueEnable const torque_enable)
{
  write(static_cast<uint16_t>(ControlTable::TorqueEnable), static_cast<uint8_t>(torque_enable));
}

void SyncGroup::setOperatingMode(OperatingMode const operating_mode)
{
  write(static_cast<uint16_t>(ControlTable::OperatingMode), static_cast<uint8_t>(operating_mode));
}

void SyncGroup::setGoalPosition(std::map<dynamixelplusplus::Dynamixel::Id, float> const & angle_deg_map)
{
  auto limit_angle = [](float const angle_deg)
  {
    if      (angle_deg < 0.0f)   return 0.0f;
    else if (angle_deg > 360.0f) return 360.0f;
    else                         return angle_deg;
  };

  auto toRegValue = [](float const angle_deg) { return static_cast<uint32_t>((angle_deg * 4096.0f) / 360.0f); };

  std::vector<uint32_t> raw_goal_position_vect;
  for (auto [id, angle_deg] : angle_deg_map)
    raw_goal_position_vect.push_back(toRegValue(limit_angle(angle_deg)));

  assert(raw_goal_position_vect.size() == _dyn_id_vect.size());

  write(static_cast<uint16_t>(ControlTable::GoalPosition), raw_goal_position_vect);
}

void SyncGroup::setGoalVelocity(std::map<dynamixelplusplus::Dynamixel::Id, float> const & velocity_rpm_map)
{
  static float const RPM_per_LSB = 0.229f;
  static float const MAX_VELOCITY_rpm = RPM_per_LSB * 1023.0f;
  static float const MIN_VELOCITY_rpm = RPM_per_LSB * 1023.0f * (-1.0);

  auto limit_velocity = [](float const rpm)
  {
    if      (rpm < MIN_VELOCITY_rpm) return MIN_VELOCITY_rpm;
    else if (rpm > MAX_VELOCITY_rpm) return MAX_VELOCITY_rpm;
    else                             return rpm;
  };

  auto toRegValue = [](float const rpm)
  {
    int32_t const rpm_lsb_signed = static_cast<int32_t>(rpm / RPM_per_LSB);
    return static_cast<uint32_t>(rpm_lsb_signed);
  };

  std::vector<uint32_t> raw_goal_velocity_vect;
  for (auto [id, vel_rpm] : velocity_rpm_map)
    raw_goal_velocity_vect.push_back(toRegValue(limit_velocity(vel_rpm)));

  assert(raw_goal_velocity_vect.size() == _dyn_id_vect.size());

  write(static_cast<uint16_t>(ControlTable::GoalVelocity), raw_goal_velocity_vect);
}

std::map<dynamixelplusplus::Dynamixel::Id, float> SyncGroup::getPresentPosition()
{
  std::vector<uint32_t> const angle_raw_vect = read<uint32_t>(static_cast<uint16_t>(ControlTable::PresentPosition));

  auto fromRegValue = [](uint32_t const angle_raw) { return static_cast<float>(angle_raw) * 360.0f / 4096.0f; };

  std::vector<float> angle_deg_vect;
  for (auto angle_raw : angle_raw_vect)
    angle_deg_vect.push_back(fromRegValue(angle_raw));

  assert(_dyn_id_vect.size() == angle_deg_vect.size());

  std::map<dynamixelplusplus::Dynamixel::Id, float> angle_deg_map;
  for (size_t i = 0; i < _dyn_id_vect.size(); i++)
    angle_deg_map[_dyn_id_vect.at(i)] = angle_deg_vect.at(i);

  return angle_deg_map;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::MX28AR */
