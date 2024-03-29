/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/ros2_dynamixel_bridge/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <ros2_dynamixel_bridge/Node.h>

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
: rclcpp::Node("ros2_dynamixel_bridge")
{
  init_heartbeat();

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

  /* Optionally check if all required node ids are online. */
  declare_parameter("required_node_id_list", std::vector<long int>{});
  declare_parameter("check_required_node_id_list", false);

  if (get_parameter("check_required_node_id_list").as_bool())
  {
    RCLCPP_INFO(get_logger(), "checking if all required servos are online ...");

    std::vector<long int> const required_node_id_list = get_parameter("required_node_id_list").as_integer_array();

    bool all_servos_online = true;
    std::stringstream offline_id_list;
    for (auto servo_id : required_node_id_list)
      if (std::none_of(std::cbegin(dyn_id_vect), std::cend(dyn_id_vect), [servo_id, &all_servos_online, &offline_id_list](int const id) { return (static_cast<Dynamixel::Id>(id) == servo_id); }))
      {
        offline_id_list << servo_id << " ";
        all_servos_online = false;
      }
    if (!all_servos_online) {
      RCLCPP_ERROR(get_logger(), "one or more MX-28AR OFF-line: { %s}, shutting down.", offline_id_list.str().c_str());
      rclcpp::shutdown();
      return;
    }
  }

  /* Create a map for individually controlling all the servos as well as all publishers and subscribers. */
  for (auto servo_id : dyn_id_vect)
  {
    auto servo_ctrl   = std::make_shared<MX28AR::Single>(dyn_ctrl, servo_id);

    /* Reboot servo to start from a clean slate. */
    servo_ctrl->reboot();
    /* Wait a little so we can be sure that the servo is online again. */
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    /* Configure for extended position control mode. */
    servo_ctrl->setOperatingMode(MX28AR::OperatingMode::ExtendedPositionControlMode);
    /* Enable torque. */
    servo_ctrl->setTorqueEnable (MX28AR::TorqueEnable::On);

    auto servo_config = std::make_shared<ServoConfig>();
    auto servo_target = std::make_shared<ServoTarget>(0.0f, servo_ctrl->getPresentPosition());

    _mx28_map[servo_id] = ServoMapValue{servo_ctrl, servo_config, servo_target};

    /* Automagically create ROS topics for Publishers and Subscribers. */
    std::stringstream
      angle_actual_rad_pub_topic,
      angle_target_rad_sub_topic,
      angle_target_vel_sub_topic,
      mode_sub_topic;

    angle_actual_rad_pub_topic << "/dynamixel/servo_" << static_cast<int>(servo_id) << "/angle/actual";
    angle_target_rad_sub_topic << "/dynamixel/servo_" << static_cast<int>(servo_id) << "/angle/target";
    angle_target_vel_sub_topic << "/dynamixel/servo_" << static_cast<int>(servo_id) << "/angular_velocity/target";
    mode_sub_topic             << "/dynamixel/servo_" << static_cast<int>(servo_id) << "/mode/set";

    RCLCPP_INFO(get_logger(),
                "initialize servo #%d\n\tInit. Pos. = %0.2f\n\tInit. Vel. = %0.2f\n\tPub:       = %s\n\tSub:       = %s\n\tSub:       = %s\n\tSub:       = %s",
                static_cast<int>(servo_id),
                servo_target->angle_deg(),
                servo_target->angular_velocity_dps(),
                angle_actual_rad_pub_topic.str().c_str(),
                angle_target_rad_sub_topic.str().c_str(),
                angle_target_vel_sub_topic.str().c_str(),
                mode_sub_topic.str().c_str());

    /* Create per-servo publisher/subscriber. */
    _angle_actual_rad_pub[servo_id] = this->create_publisher<std_msgs::msg::Float32>(angle_actual_rad_pub_topic.str(), 1);

    _angle_target_rad_sub[servo_id] = create_subscription<std_msgs::msg::Float32>
      (angle_target_rad_sub_topic.str(),
       1,
       [this, servo_target](std_msgs::msg::Float32::SharedPtr const msg)
       {
         servo_target->set_angle_deg(msg->data * 180.0f / M_PI);
       });

    _angle_target_vel_sub[servo_id] = create_subscription<std_msgs::msg::Float32>
      (angle_target_vel_sub_topic.str(),
       1,
       [this, servo_target](std_msgs::msg::Float32::SharedPtr const msg)
       {
         servo_target->set_angular_velocity_dps(msg->data * 180.0f / M_PI);
       });

    _mode_sub[servo_id] = create_subscription<ros2_dynamixel_bridge::msg::Mode>
      (mode_sub_topic.str(),
       1,
       [this, servo_config, servo_ctrl](ros2_dynamixel_bridge::msg::Mode::SharedPtr const msg)
       {
         /* Obtain the desired operation mode. */
         MX28AR::OperatingMode next_op_mode = servo_config->mode();

         if      (msg->servo_mode == ros2_dynamixel_bridge::msg::Mode::SERVO_MODE_VELOCITY_CONTROL)
           next_op_mode = MX28AR::OperatingMode::VelocityControlMode;
         else if (msg->servo_mode == ros2_dynamixel_bridge::msg::Mode::SERVO_MODE_POSITION_CONTROL)
           next_op_mode = MX28AR::OperatingMode::ExtendedPositionControlMode;
         else {
           RCLCPP_ERROR(get_logger(), "invalid value (%d) for parameter op mode.", static_cast<int>(msg->servo_mode));
           return;
         }

         /* Only configure the servo if the operational mode has changed. */
         if (next_op_mode != servo_config->mode())
         {
           servo_config->set_mode(next_op_mode);

           RCLCPP_INFO(get_logger(), "servo #%d is set to mode %d.", static_cast<int>(servo_ctrl->id()) , static_cast<int>(servo_config->mode()));

           servo_ctrl->setTorqueEnable (MX28AR::TorqueEnable::Off);
           servo_ctrl->setOperatingMode(servo_config->mode());
           servo_ctrl->setTorqueEnable (MX28AR::TorqueEnable::On);
         }
       });
  }

  /* Create sync group consisting of all dynamixel servos. */
  _mx28_sync_ctrl = std::make_shared<MX28AR::SyncGroup>(dyn_ctrl, dyn_id_vect);

  /* Configure periodic control loop function. */
  _io_loop_rate_monitor = loop_rate::Monitor::create(
    IO_LOOP_RATE,
    std::chrono::milliseconds(1)
    );

  _io_loop_timer = create_wall_timer(
    IO_LOOP_RATE,
    [this]() { this->io_loop(); }
    );

  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

Node::~Node()
{
  /* Switch back to position control mode - and hold position. */
  for (auto const & [id, servo] : _mx28_map)
  {
    /* Ensure that the goal position is zero. */
    servo.ctrl->setGoalVelocity(0.0f);
    /* Ensure that the goal position is the current position. */
    servo.ctrl->setGoalPosition(servo.ctrl->getPresentPosition());
    /* Configure for position control mode. */
    servo.ctrl->setTorqueEnable (MX28AR::TorqueEnable::Off);
    servo.ctrl->setOperatingMode(MX28AR::OperatingMode::ExtendedPositionControlMode);
    servo.ctrl->setTorqueEnable (MX28AR::TorqueEnable::On);
  }

  RCLCPP_INFO(get_logger(), "%s shut down successfully.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 ****************************************************dd**********************************/

void Node::init_heartbeat()
{
  std::stringstream heartbeat_topic;
  heartbeat_topic << "/l3xz/" << get_name() << "/heartbeat";

  _heartbeat_pub = heartbeat::Publisher::create(*this, heartbeat_topic.str());
}

void Node::io_loop()
{
  _io_loop_rate_monitor->update();
  if (auto const [timeout, opt_timeout_duration] = _io_loop_rate_monitor->isTimeout();
    timeout == loop_rate::Monitor::Timeout::Yes)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         1000,
                         "io_loop should be called every %ld ms, but is %ld ms instead",
                         IO_LOOP_RATE.count(),
                         opt_timeout_duration.value().count());
  }


  /* This function contains the general error handling and recovery code. *************/
  auto dynamixel_error_hdl = [this](Dynamixel::Id const err_id)
  {
    RCLCPP_ERROR(get_logger(), "hardware alert for servo #%d caught.", static_cast<int>(err_id));
    auto iter = _mx28_map.find(err_id);
    if (iter == _mx28_map.end()) {
      RCLCPP_ERROR(get_logger(), "no servo with id #%d found.", static_cast<int>(err_id));
      return;
    }
    auto const servo_ctrl = iter->second.ctrl;
    auto const servo_cfg  = iter->second.cfg;
    uint8_t const hw_err_code = servo_ctrl->getHardwareErrorCode();
    RCLCPP_ERROR(get_logger(), "\thardware error code for servo #%d caught: %02X", static_cast<int>(servo_ctrl->id()), hw_err_code);
    servo_ctrl->reboot();
    servo_ctrl->setTorqueEnable (MX28AR::TorqueEnable::Off);
    servo_ctrl->setOperatingMode(servo_cfg->mode());
    servo_ctrl->setTorqueEnable (MX28AR::TorqueEnable::On);
    servo_ctrl->setGoalVelocity (0.0f);
  };

  /* Synchronously retrieve the current position of each servo. ***********************/
  std::map<Dynamixel::Id, float> actual_angle_deg_map;
  try {
    actual_angle_deg_map = _mx28_sync_ctrl->getPresentPosition();
  }
  catch (dynamixelplusplus::HardwareAlert const & err) {
    dynamixel_error_hdl(err.id());
    return;
  }
  catch (dynamixelplusplus::StatusError const & e) {
    RCLCPP_ERROR(get_logger(), "StatusError caught: %s", e.what());
    return;
  }


  /* Publish the current position via various ROS topics (one per joint). *************/
  for (auto [servo_id, angle_deg] : actual_angle_deg_map)
  {
    std_msgs::msg::Float32 msg;
    msg.data = angle_deg * M_PI / 180.0f;
    _angle_actual_rad_pub.at(servo_id)->publish(msg);
  };

  /* Calculate RPMs and limit them for all servos. ************************************/
  std::map<Dynamixel::Id, float> target_velocity_rpm_map;
  for (auto [servo_id, servo] : _mx28_map)
  {
    auto servo_target = servo.target;

    static float constexpr DEADZONE_RPM = 1.0f;
    static float constexpr DPS_per_RPM = 360.0f / 60.0f;

    float const target_velocity_dps = servo_target->angular_velocity_dps();
    float       target_velocity_rpm = target_velocity_dps / DPS_per_RPM;

    /* Checking if the target velocity exceeds the configured dead-zone.
     * Only then we should actually write a value != 0 to the servos,
     * otherwise very slow drift can occur.
     */
    if (fabs(target_velocity_rpm) < DEADZONE_RPM)
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
  catch (dynamixelplusplus::StatusError const & e) {
    RCLCPP_ERROR(get_logger(), "StatusError caught: %s", e.what());
    return;
  }

  /* Write the computed angle values to the servos. ***********************************/
  try {
    std::map<Dynamixel::Id, float> target_angle_deg_map;
    for (auto [servo_id, servo] : _mx28_map)
      target_angle_deg_map[servo_id] = servo.target->angle_deg();

    _mx28_sync_ctrl->setGoalPosition(target_angle_deg_map);
  }
  catch (dynamixelplusplus::HardwareAlert const & err) {
    dynamixel_error_hdl(err.id());
    return;
  }
  catch (dynamixelplusplus::StatusError const & e) {
    RCLCPP_ERROR(get_logger(), "StatusError caught: %s", e.what());
    return;
  }
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */
