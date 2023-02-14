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

#include <tuple>

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
  { }


  /* MEMBER FUNCTIONS *****************************************************************/
  void setTorqueEnable (TorqueEnable const torque_enable);
  void setOperatingMode(OperatingMode const operating_mode);
  void setGoalPosition (std::vector<float> const & angle_deg_vect);
  void setGoalVelocity (std::vector<float> const & velocity_rpm_vect);

  std::tuple<float, float> getPresentPosition();
};

class HeadSyncGroup : public SyncGroup
{
public:
  HeadSyncGroup(dynamixelplusplus::SharedDynamixel dyn_ctrl,
                dynamixelplusplus::Dynamixel::Id const pan_servo_id,
                dynamixelplusplus::Dynamixel::Id const tilt_servo_id)
  : SyncGroup(dyn_ctrl, dynamixelplusplus::Dynamixel::IdVect{pan_servo_id, tilt_servo_id})
  { }

  void setGoalPosition(float const pan_angle_deg, float const tilt_angle_deg)
  {
    return SyncGroup::setGoalPosition(std::vector<float>{pan_angle_deg, tilt_angle_deg});
  }
  void setGoalVelocity (float const pan_velocity_rpm, float const tilt_velocity_rpm)
  {
    return SyncGroup::setGoalVelocity(std::vector<float>{pan_velocity_rpm, tilt_velocity_rpm});
  }
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif //L3XZ_HEAD_CTRL_MX28ARCONTROL_H
