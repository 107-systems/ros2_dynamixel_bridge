/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_dynamixel_bridge/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_ros_dynamixel_bridge/MX28ARSingle.h>

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

float Single::getPresentPosition()
{
  uint32_t const angle_raw = _dyn_ctrl->read<uint32_t>(static_cast<uint16_t>(ControlTable::PresentPosition), _id);
  float const angle_deg = static_cast<float>(angle_raw) * 360.0f / 4096.0f;
  return angle_deg;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::MX28AR */
