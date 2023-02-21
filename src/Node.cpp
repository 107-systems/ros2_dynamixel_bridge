/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ros_dynamixel_bridge/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_ros_dynamixel_bridge/Node.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::head
{

using namespace dynamixelplusplus;

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Node::Node()
: rclcpp::Node("l3xz_ros_dynamixel_bridge")
, _prev_io_loop_timepoint{std::chrono::steady_clock::now()}
{
  /* Declare parameter. */
  declare_parameter("serial_port", "/dev/ttyUSB0");
  declare_parameter("serial_port_baudrate", (2*1000*1000));

  /* Configure the Dynamixel MX-28AR servos of the pan/tilt head. */
  std::string const serial_port  = get_parameter("serial_port").as_string();
  int const serial_port_baudrate = get_parameter("serial_port_baudrate").as_int();

  RCLCPP_INFO(get_logger(), "configuring Dynamixel RS485 bus:\n\tDevice:   %s\n\tBaudrate: %d", serial_port.c_str(), serial_port_baudrate);

  SharedDynamixel dyn_ctrl = std::make_shared<Dynamixel>(serial_port, Dynamixel::Protocol::V2_0, serial_port_baudrate);

  /* Determine which/if any servos can be reached via the connected network. */
  auto dyn_id_vect = dyn_ctrl->broadcastPing();
  std::sort(dyn_id_vect.begin(), dyn_id_vect.end());

  std::stringstream dyn_id_list;
  for (auto id : dyn_id_vect)
    dyn_id_list << static_cast<int>(id) << " ";
  RCLCPP_INFO(get_logger(), "detected Dynamixel MX-28AR: { %s}.", dyn_id_list.str().c_str());

  /* Create a map for individually controlling all the servos as well as all publishers and subscribers. */
  for (auto servo_id : dyn_id_vect)
  {
    /* TODO: Retrieve default parameter from param file, if available. */
    auto servo_cfg = std::make_shared<ServoConfiguration>(MX28AR::OperatingMode::PositionControlMode, 180.0f);
    auto servo_ctrl = std::make_shared<MX28AR::Single>(dyn_ctrl, servo_id);

    /* Reboot all servo to start from a clean slate. */
    servo_ctrl->reboot();
    /* Wait a little so we can be sure that all servos are online again. */
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    _mx28_cfg_map [servo_id] = servo_cfg;
    _mx28_ctrl_map[servo_id] = servo_ctrl;

    /* Configure initial velocity and target angle. */
    _target_angle_deg_map           [servo_id] = servo_cfg->initial_target_angle_deg;
    _target_angular_velocity_dps_map[servo_id] = 0.0f;

    std::stringstream
      angle_deg_pub_topic,
      angle_deg_sub_topic,
      angle_vel_sub_topic;

    angle_deg_pub_topic << "/l3xz/dynamixel/servo_" << static_cast<int>(servo_id) << "/angle/actual";
    angle_deg_sub_topic << "/l3xz/dynamixel/servo_" << static_cast<int>(servo_id) << "/angle/target";
    angle_vel_sub_topic << "/l3xz/dynamixel/servo_" << static_cast<int>(servo_id) << "/angular_velocity/target";

    RCLCPP_INFO(get_logger(),
                "initialize servo #%d\n\tInit. Pos. = %0.2f\n\tPub:       = %s\n\tSub:       = %s\n\tSub:       = %s",
                static_cast<int>(servo_id),
                servo_cfg->initial_target_angle_deg,
                angle_deg_pub_topic.str().c_str(),
                angle_deg_sub_topic.str().c_str(),
                angle_vel_sub_topic.str().c_str());

    servo_ctrl->setTorqueEnable (MX28AR::TorqueEnable::Off);
    servo_ctrl->setOperatingMode(MX28AR::OperatingMode::PositionControlMode);
    servo_ctrl->setTorqueEnable (MX28AR::TorqueEnable::On);

    float const target_angle_deg = servo_cfg->initial_target_angle_deg;
    servo_ctrl->setGoalPosition(target_angle_deg);

    bool target_angle_reached = false;
    float actual_angle_deg = 0.0f;
    for (auto const start = std::chrono::system_clock::now();
         (std::chrono::system_clock::now() - start) < std::chrono::seconds(5) && !target_angle_reached;)
    {
      std::this_thread::sleep_for(std::chrono::seconds(1));

      actual_angle_deg = servo_ctrl->getPresentPosition();
      static float constexpr
      INITIAL_ANGLE_EPSILON_deg = 2.0f;
      target_angle_reached = fabs(actual_angle_deg - target_angle_deg) < INITIAL_ANGLE_EPSILON_deg;
    }

    if (!target_angle_reached)
    {
      RCLCPP_ERROR(get_logger(),
                   "could not reach initial position for servo #%d, target: %0.2f, actual: %0.2f.",
                   static_cast<int>(servo_id),
                   target_angle_deg,
                   actual_angle_deg);
      rclcpp::shutdown();
      return;
    }

    /* Create per-servo publisher/subscriber. */
    _angle_deg_pub[servo_id] = this->create_publisher<std_msgs::msg::Float32>(angle_deg_pub_topic.str(), 1);

    _angle_deg_sub[servo_id] = create_subscription<std_msgs::msg::Float32>
      (angle_deg_sub_topic.str(),
       1,
       [this, servo_id](std_msgs::msg::Float32::SharedPtr const msg)
       {
         _target_angle_deg_map[servo_id] = msg->data * 180.0f / M_PI;
       });

    _angle_vel_sub[servo_id] = create_subscription<std_msgs::msg::Float32>
      (angle_vel_sub_topic.str(),
       1,
       [this, servo_id](std_msgs::msg::Float32::SharedPtr const msg)
       {
         _target_angular_velocity_dps_map[servo_id] = msg->data * 180.0f / M_PI;
       });
  }

  /* Create sync group consisting of all dynamixel servos. */
  _mx28_sync_ctrl = std::make_shared<MX28AR::SyncGroup>(dyn_ctrl, dyn_id_vect);

  /* Configure subscribers and publishers. */
  _set_mode_srv = create_service<l3xz_ros_dynamixel_bridge::srv::SetMode>(
    "/l3xz/dynamixel/mode/set",
    [this](std::shared_ptr<l3xz_ros_dynamixel_bridge::srv::SetMode::Request> const request, std::shared_ptr<l3xz_ros_dynamixel_bridge::srv::SetMode::Response> response)
    { set_mode(request, response); });

  /* Configure periodic control loop function. */
  _io_loop_timer = create_wall_timer
    (std::chrono::milliseconds(IO_LOOP_RATE.count()),
     [this]() { this->io_loop(); });

  RCLCPP_INFO(get_logger(), "node initialization complete.");
}

Node::~Node()
{
  /* Switch back to position control mode - and hold position. */
  _mx28_sync_ctrl->setTorqueEnable (MX28AR::TorqueEnable::Off);
  _mx28_sync_ctrl->setOperatingMode(MX28AR::OperatingMode::PositionControlMode);
  _mx28_sync_ctrl->setTorqueEnable (MX28AR::TorqueEnable::On);
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 ****************************************************dd**********************************/

void Node::io_loop()
{
  auto const now = std::chrono::steady_clock::now();
  auto const io_loop_rate = (now - _prev_io_loop_timepoint);
  if (io_loop_rate > (IO_LOOP_RATE + std::chrono::milliseconds(1)))
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         10*1000UL, /* 10 sec. */
                         "io_loop should be called every %ld ms, but is %ld ms instead",
                         IO_LOOP_RATE.count(),
                         std::chrono::duration_cast<std::chrono::milliseconds>(io_loop_rate).count());
  _prev_io_loop_timepoint = now;

  /* This function contains the general error handling and recovery code. *************/
  auto dynamixel_error_hdl = [this](Dynamixel::Id const err_id)
  {
    RCLCPP_ERROR(get_logger(), "hardware alert for servo #%d caught.", static_cast<int>(err_id));
    auto iter = _mx28_ctrl_map.find(err_id);
    if (iter == _mx28_ctrl_map.end()) {
      RCLCPP_ERROR(get_logger(), "no servo with id #%d found.", static_cast<int>(err_id));
      return;
    }
    auto const servo_ctrl = iter->second;
    auto const servo_cfg = _mx28_cfg_map.at(servo_ctrl->id());
    uint8_t const hw_err_code = servo_ctrl->getHardwareErrorCode();
    RCLCPP_ERROR(get_logger(), "\thardware error code for servo #%d caught: %02X", static_cast<int>(servo_ctrl->id()), hw_err_code);
    servo_ctrl->reboot();
    servo_ctrl->setTorqueEnable (MX28AR::TorqueEnable::Off);
    servo_ctrl->setOperatingMode(servo_cfg->op_mode);
    servo_ctrl->setTorqueEnable (MX28AR::TorqueEnable::On);
    servo_ctrl->setGoalVelocity (0.0f);
  };

  /* Synchronously retrieve the current position of each servo. ***********************/
  std::map<Dynamixel::Id, float> actual_angle_deg_map;
  try {
    actual_angle_deg_map = _mx28_sync_ctrl->getPresentPosition();
  }
  catch (dynamixelplusplus::HardwareAlert const & err)
  {
    dynamixel_error_hdl(err.id());
    return;
  }

  /* Publish the current position via various ROS topics (one per joint). *************/
  for (auto [servo_id, angle_deg] : actual_angle_deg_map)
  {
    std_msgs::msg::Float32 msg;
    msg.data = angle_deg;
    _angle_deg_pub.at(servo_id)->publish(msg);
  };

  /* Calculate RPMs and limit them for all servos. ************************************/
  std::map<Dynamixel::Id, float> target_velocity_rpm_map;
  for (auto [servo_id, servo_cfg] : _mx28_cfg_map)
  {
    static float constexpr DEADZONE_RPM = 1.0f;
    static float constexpr DPS_per_RPM = 360.0f / 60.0f;

    float const actual_angle_deg    = actual_angle_deg_map.at(servo_id);
    float const target_velocity_dps = _target_angular_velocity_dps_map.at(servo_id);
    float       target_velocity_rpm = target_velocity_dps / DPS_per_RPM;

    /* Checking if the target velocity exceeds the configured dead-zone.
     * Only then we should actually write a value != 0 to the servos,
     * otherwise very slow drift can occur.
     */
    if (fabs(target_velocity_rpm) < DEADZONE_RPM)
      target_velocity_rpm = 0.0f;

    /* Checking current head position and stopping if either
     * pan or tilt angle would exceed the maximum allowed angle.
     */
    if ((actual_angle_deg < 160.0f) && (target_velocity_dps < 0.0f))
      target_velocity_rpm = 0.0f;
    if ((actual_angle_deg > 200.0f) && (target_velocity_dps > 0.0f))
      target_velocity_rpm = 0.0f;

    target_velocity_rpm_map[servo_id] = target_velocity_rpm;
  }

  /* Write the computed RPM values to the servos. *************************************/
  try {
    _mx28_sync_ctrl->setGoalVelocity(target_velocity_rpm_map);
  }
  catch (dynamixelplusplus::HardwareAlert const & err) {
    dynamixel_error_hdl(err.id());
    return;
  }

  /* Write the computed angle values to the servos. ***********************************/
  try {
    _mx28_sync_ctrl->setGoalPosition(_target_angle_deg_map);
  }
  catch (dynamixelplusplus::HardwareAlert const & err) {
    dynamixel_error_hdl(err.id());
    return;
  }
}

void Node::set_mode(std::shared_ptr<l3xz_ros_dynamixel_bridge::srv::SetMode::Request> const request,
                    std::shared_ptr<l3xz_ros_dynamixel_bridge::srv::SetMode::Response> response)
{
  /* Check if we have such a servo id registered. */
  Dynamixel::Id const servo_id = request->servo_id;
  auto iter = _mx28_ctrl_map.find(servo_id);
  if (iter == _mx28_ctrl_map.end()) {
    RCLCPP_ERROR(get_logger(), "no servo with id #%d found.", static_cast<int>(servo_id));
    response->result = l3xz_ros_dynamixel_bridge::srv::SetMode::Response::SET_MODE_ERROR;
    return;
  }
  auto const servo_ctrl = iter->second;
  auto const servo_cfg = _mx28_cfg_map.at(servo_ctrl->id());

  /* Obtain the desired operation mode. */
  if      (request->servo_mode == l3xz_ros_dynamixel_bridge::srv::SetMode::Request::SERVO_MODE_VELOCITY_CONTROL)
    servo_cfg->op_mode = MX28AR::OperatingMode::VelocityControlMode;
  else if (request->servo_mode == l3xz_ros_dynamixel_bridge::srv::SetMode::Request::SERVO_MODE_POSITION_CONTROL)
    servo_cfg->op_mode = MX28AR::OperatingMode::PositionControlMode;
  else {
    RCLCPP_ERROR(get_logger(), "invalid value (%d) for parameter op mode.", request->servo_mode);
    response->result = l3xz_ros_dynamixel_bridge::srv::SetMode::Response::SET_MODE_ERROR;
    return;
  }

  /* Perform the desired change in operation mode. */
  servo_ctrl->setTorqueEnable (MX28AR::TorqueEnable::Off);
  servo_ctrl->setOperatingMode(_mx28_cfg_map.at(servo_id)->op_mode);
  servo_ctrl->setTorqueEnable (MX28AR::TorqueEnable::On);

  /* Set the return code. */
  response->result = l3xz_ros_dynamixel_bridge::srv::SetMode::Response::SET_MODE_SUCCESS;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */
