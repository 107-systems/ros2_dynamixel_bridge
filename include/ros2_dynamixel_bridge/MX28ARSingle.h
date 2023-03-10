/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_dynamixel_bridge/graphs/contributors.
 */

#ifndef L3XZ_ROS_DYNAMIXEL_BRIDGE_MX28ARSINGLE_H
#define L3XZ_ROS_DYNAMIXEL_BRIDGE_MX28ARSINGLE_H

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <dynamixel++/dynamixel++.h>

#include "MX28AR.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::MX28AR
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Single
{
public:
  /* CTOR/DTOR ************************************************************************/
  Single(dynamixelplusplus::SharedDynamixel dyn_ctrl,
         dynamixelplusplus::Dynamixel::Id const id);

  /* MEMBER FUNCTIONS *****************************************************************/
  void reboot();

  void setTorqueEnable   (TorqueEnable const torque_enable);
  void setOperatingMode  (OperatingMode const operating_mode);
  void setGoalPosition   (float const angle_deg);
  void setGoalVelocity   (float const velocity_rpm);

  [[nodiscard]] float   getPresentPosition();
  [[nodiscard]] uint8_t getHardwareErrorCode();
  [[nodiscard]] dynamixelplusplus::Dynamixel::Id id() const {  return _id; }


private:
  dynamixelplusplus::SharedDynamixel _dyn_ctrl;
  dynamixelplusplus::Dynamixel::Id const _id;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::MX28AR */

#endif //L3XZ_ROS_DYNAMIXEL_BRIDGE_MX28ARSINGLE_H
