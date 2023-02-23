/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_dynamixel_bridge/graphs/contributors.
 */

#ifndef L3XZ_HEAD_CTRL_MX28ARCONTROL_H
#define L3XZ_HEAD_CTRL_MX28ARCONTROL_H

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <map>

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

class SyncGroup : private dynamixelplusplus::SyncGroup
{
public:
  /* CTOR/DTOR ************************************************************************/
  SyncGroup(dynamixelplusplus::SharedDynamixel dyn_ctrl,
            dynamixelplusplus::Dynamixel::IdVect const & dyn_id_vect)
  : dynamixelplusplus::SyncGroup{dyn_ctrl, dyn_id_vect}
  , _dyn_id_vect{dyn_id_vect}
  { }


  /* MEMBER FUNCTIONS *****************************************************************/
  void setTorqueEnable (TorqueEnable const torque_enable);
  void setOperatingMode(OperatingMode const operating_mode);
  void setGoalPosition (std::map<dynamixelplusplus::Dynamixel::Id, float> const & angle_deg_map);
  void setGoalVelocity (std::map<dynamixelplusplus::Dynamixel::Id, float> const & velocity_rpm_map);

  std::map<dynamixelplusplus::Dynamixel::Id, float> getPresentPosition();

private:
  dynamixelplusplus::Dynamixel::IdVect const _dyn_id_vect;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif //L3XZ_HEAD_CTRL_MX28ARCONTROL_H
