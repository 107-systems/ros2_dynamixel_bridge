/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io_dynamixel/graphs/contributors.
 */

#ifndef L3XZ_HEAD_CTRL_MX28ARCONTROL_H
#define L3XZ_HEAD_CTRL_MX28ARCONTROL_H

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <map>
#include <memory>

#include <dynamixel++/dynamixel++.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class MX28AR : private dynamixelplusplus::SyncGroup
{
public:
  /* TYPEDEF **************************************************************************/

  enum class ControlTable : uint16_t
  {
    OperatingMode   =  11,
    TorqueEnable    =  64,
    GoalVelocity    = 104,
    GoalPosition    = 116,
    PresentPosition = 132,
  };

  enum class OperatingMode : uint8_t
  {
    VelocityControlMode         = 1,
    PositionControlMode         = 3,
    ExtendedPositionControlMode = 4,
    PwmControlMode              = 16,
  };

  enum class TorqueEnable : uint8_t
  {
    Off = 0,
    On  = 1,
  };


  /* CTOR/DTOR ************************************************************************/
  MX28AR(dynamixelplusplus::SharedDynamixel dyn_ctrl,
         dynamixelplusplus::Dynamixel::Id const pan_servo_id,
         dynamixelplusplus::Dynamixel::Id const tilt_servo_id)
  : dynamixelplusplus::SyncGroup{dyn_ctrl, dynamixelplusplus::Dynamixel::IdVect{pan_servo_id, tilt_servo_id}}
  { }


  /* MEMBER FUNCTIONS *****************************************************************/
  void setTorqueEnable (TorqueEnable const torque_enable);
  void setOperatingMode(OperatingMode const operating_mode);
  void setGoalPosition (float const pan_angle_deg, float const tilt_angle_deg);
  void setGoalVelocity (float const pan_velocity_rpm, float const tilt_velocity_rpm);

  std::tuple<float, float> getPresentPosition();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif //L3XZ_HEAD_CTRL_MX28ARCONTROL_H
