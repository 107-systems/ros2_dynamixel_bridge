/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io_dynamixel/graphs/contributors.
 */

#ifndef L3XZ_IO_DYNAMIXEL_MX28AR_H
#define L3XZ_IO_DYNAMIXEL_MX28AR_H

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <cstdint>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::MX28AR
{

enum class ControlTable : uint16_t
{
  OperatingMode = 11,
  TorqueEnable = 64,
  GoalVelocity = 104,
  GoalPosition = 116,
  PresentPosition = 132,
};

enum class OperatingMode : uint8_t
{
  VelocityControlMode = 1,
  PositionControlMode = 3,
  ExtendedPositionControlMode = 4,
  PwmControlMode = 16,
};

enum class TorqueEnable : uint8_t
{
  Off = 0,
  On = 1,
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::MX28AR */

#endif //L3XZ_IO_DYNAMIXEL_MX28AR_H
