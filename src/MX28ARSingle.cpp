/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_dynamixel_bridge/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <ros2_dynamixel_bridge/MX28ARSingle.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::MX28AR
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Single::Single(dynamixelplusplus::SharedDynamixel dyn_ctrl,
               dynamixelplusplus::Dynamixel::Id const id)
: _dyn_ctrl{dyn_ctrl}
, _id{id}
{ }

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Single::reboot()
{
  _dyn_ctrl->reboot(_id);
}

void Single::setTorqueEnable(TorqueEnable const torque_enable)
{
  _dyn_ctrl->write(static_cast<uint16_t>(ControlTable::TorqueEnable), _id, static_cast<uint8_t>(torque_enable));
}

void Single::setOperatingMode(OperatingMode const operating_mode)
{
  _dyn_ctrl->write(static_cast<uint16_t>(ControlTable::OperatingMode), _id, static_cast<uint8_t>(operating_mode));
}

void Single::setGoalPosition(float const angle_deg)
{
  uint32_t const angle_raw = static_cast<uint32_t>((angle_deg * 4096.0f) / 360.0f);
  _dyn_ctrl->write(static_cast<uint16_t>(ControlTable::GoalPosition), _id, angle_raw);
}

void Single::setGoalVelocity(float const velocity_rpm)
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

  uint32_t const raw_goal_velocity = toRegValue(limit_velocity(velocity_rpm)); ;
  _dyn_ctrl->write(static_cast<uint16_t>(ControlTable::GoalVelocity), _id, raw_goal_velocity);
}

float Single::getPresentPosition()
{
  uint32_t const angle_raw = _dyn_ctrl->read<uint32_t>(static_cast<uint16_t>(ControlTable::PresentPosition), _id);
  float const angle_deg = static_cast<float>(angle_raw) * 360.0f / 4096.0f;
  return angle_deg;
}

uint8_t Single::getHardwareErrorCode()
{
  uint8_t const hw_err_code = _dyn_ctrl->read<uint8_t>(static_cast<uint16_t>(ControlTable::HardwareErrorStatus), _id);
  return hw_err_code;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::MX28AR */
