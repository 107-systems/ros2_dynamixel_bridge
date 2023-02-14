/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_io_dynamixel/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_io_dynamixel/Node.h>

#include <l3xz_io_dynamixel/MX28AR.h>

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
: rclcpp::Node("l3xz_io_dynamixel")
, _pan_servo_id{DEFAULT_PAN_SERVO_ID}
, _tilt_servo_id{DEFAULT_TILT_SERVO_ID}
, _pan_angular_velocity_rad_per_sec{0.0f}
, _tilt_angular_velocity_rad_per_sec{0.0f}
{
  /* Configure the Dynamixel MX-28AR servos of the pan/tilt head. */

  declare_parameter("serial_port", "/dev/ttyUSB0");
  declare_parameter("serial_port_baudrate", DEFAULT_SERIAL_BAUDRATE);
  declare_parameter("pan_servo_id", DEFAULT_PAN_SERVO_ID);
  declare_parameter("tilt_servo_id", DEFAULT_TILT_SERVO_ID);
  declare_parameter("pan_servo_initial_angle", DEFAULT_PAN_SERVO_INITIAL_ANGLE);
  declare_parameter("pan_servo_min_angle", DEFAULT_PAN_SERVO_MIN_ANGLE);
  declare_parameter("pan_servo_max_angle", DEFAULT_PAN_SERVO_MAX_ANGLE);
  declare_parameter("tilt_servo_initial_angle", DEFAULT_TILT_SERVO_INITIAL_ANGLE);
  declare_parameter("tilt_servo_min_angle", DEFAULT_TILT_SERVO_MIN_ANGLE);
  declare_parameter("tilt_servo_max_angle", DEFAULT_TILT_SERVO_MAX_ANGLE);

  std::string const serial_port  = get_parameter("serial_port").as_string();
  int const serial_port_baudrate = get_parameter("serial_port_baudrate").as_int();

  RCLCPP_INFO(get_logger(), "configuring Dynamixel RS485 bus:\n\tDevice:   %s\n\tBaudrate: %d", serial_port.c_str(), serial_port_baudrate);

  SharedDynamixel dyn_ctrl = std::make_shared<Dynamixel>(serial_port, Dynamixel::Protocol::V2_0, serial_port_baudrate);

  /* Determine which/if any servos can be reached via the connected network. */
  auto const dyn_id_vect = dyn_ctrl->broadcastPing();

  std::stringstream dyn_id_list;
  for (auto id : dyn_id_vect)
    dyn_id_list << static_cast<int>(id) << " ";
  RCLCPP_INFO(get_logger(), "detected Dynamixel MX-28AR: { %s}.", dyn_id_list.str().c_str());

  _pan_servo_id  = static_cast<Dynamixel::Id>(get_parameter("pan_servo_id").as_int());
  _tilt_servo_id = static_cast<Dynamixel::Id>(get_parameter("tilt_servo_id").as_int());

  if (std::none_of(std::cbegin(dyn_id_vect), std::cend(dyn_id_vect), [this](Dynamixel::Id const id) { return (id == _pan_servo_id); })) {
    RCLCPP_ERROR(get_logger(), "pan servo with configured id %d not online.", static_cast<int>(_pan_servo_id));
    rclcpp::shutdown();
  }

  if (std::none_of(std::cbegin(dyn_id_vect), std::cend(dyn_id_vect), [this](Dynamixel::Id const id) { return (id == _tilt_servo_id); })) {
    RCLCPP_ERROR(get_logger(), "tilt servo with configured id %d not online.", static_cast<int>(_tilt_servo_id));
    rclcpp::shutdown();
  }

  RCLCPP_INFO(get_logger(), "initialize pan/servo in position control mode and set to initial angle.");

  /* Instantiate MX-28AR controller and continue with pan/tilt head initialization. */
  _mx28_ctrl = std::make_shared<MX28AR>(dyn_ctrl, _pan_servo_id, _tilt_servo_id);

  _mx28_ctrl->setTorqueEnable (MX28AR::TorqueEnable::Off);
  _mx28_ctrl->setOperatingMode(MX28AR::OperatingMode::PositionControlMode);
  _mx28_ctrl->setTorqueEnable (MX28AR::TorqueEnable::On);
  _mx28_ctrl->setGoalPosition (get_parameter("pan_servo_initial_angle").as_double(), get_parameter("tilt_servo_initial_angle").as_double());

  bool pan_target_reached = false, tilt_target_reached = false;
  float actual_pan_angle_deg = 0.0f, actual_tilt_angle_deg = 0.0f;
  for (auto const start = std::chrono::system_clock::now();
       (std::chrono::system_clock::now() - start) < std::chrono::seconds(5) && !pan_target_reached && !tilt_target_reached; )
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto [pan_angle_deg, tilt_angle_deg] = _mx28_ctrl->getPresentPosition();

    actual_pan_angle_deg  = pan_angle_deg;
    actual_tilt_angle_deg = tilt_angle_deg;

    static float constexpr INITIAL_ANGLE_EPSILON_deg = 2.0f;
    pan_target_reached  = fabs(actual_pan_angle_deg  - get_parameter("pan_servo_initial_angle").as_double())  < INITIAL_ANGLE_EPSILON_deg;
    tilt_target_reached = fabs(actual_tilt_angle_deg - get_parameter("tilt_servo_initial_angle").as_double()) < INITIAL_ANGLE_EPSILON_deg;
  }

  if (!pan_target_reached)
  {
    RCLCPP_ERROR(get_logger(),
                 "could not reach initial position for pan servo, target: %0.2f, actual: %0.2f.",
                 get_parameter("pan_servo_initial_angle").as_double(),
                 actual_pan_angle_deg);
    rclcpp::shutdown();
  }

  if (!tilt_target_reached)
  {
    RCLCPP_ERROR(get_logger(),
                 "could not reach initial position for tilt servo, target: %0.2f, actual: %0.2f.",
                 get_parameter("tilt_servo_initial_angle").as_double(),
                 actual_tilt_angle_deg);
    rclcpp::shutdown();
  }

  _mx28_ctrl->setTorqueEnable (MX28AR::TorqueEnable::Off);
  _mx28_ctrl->setOperatingMode(MX28AR::OperatingMode::VelocityControlMode);
  _mx28_ctrl->setTorqueEnable (MX28AR::TorqueEnable::On);
  _mx28_ctrl->setGoalVelocity (0.0, 0.0);

  /* Configure subscribers and publishers. */

  _head_io_sub = create_subscription<l3xz_io_dynamixel::msg::HeadVelocity>
    ("/l3xz/io/cmd_vel_head", 1,
    [this](l3xz_io_dynamixel::msg::HeadVelocity::SharedPtr const msg)
    {
      _pan_angular_velocity_rad_per_sec  = msg->pan_vel_rad_per_sec;
      _tilt_angular_velocity_rad_per_sec = msg->tilt_vel_rad_per_sec;
    });

  /* Configure periodic control loop function. */

  _io_loop_timer = create_wall_timer
    (std::chrono::milliseconds(50),
     [this]()
     {
      this->io_loop();
     });

  RCLCPP_INFO(get_logger(), "node initialization complete.");
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::io_loop()
{
  std::map<dynamixelplusplus::Dynamixel::Id, float> goal_velocity_rpm;

  float const pan_angular_velocity_dps  = _pan_angular_velocity_rad_per_sec  * 180.0f / M_PI;
  float const tilt_angular_velocity_dps = _tilt_angular_velocity_rad_per_sec * 180.0f / M_PI;

  static float constexpr DPS_per_RPM = 360.0f / 60.0f;
  goal_velocity_rpm[_pan_servo_id]  = pan_angular_velocity_dps / DPS_per_RPM;
  goal_velocity_rpm[_tilt_servo_id] = tilt_angular_velocity_dps / DPS_per_RPM;

  /* Checking current head position and stopping if either
   * pan or tilt angle would exceed the maximum allowed angle.
   */
  auto [pan_angle_deg, tilt_angle_deg] = _mx28_ctrl->getPresentPosition();

  if ((pan_angle_deg < get_parameter("pan_servo_min_angle").as_double()) && (pan_angular_velocity_dps < 0.0f))
    goal_velocity_rpm[_pan_servo_id] = 0.0f;
  if ((pan_angle_deg > get_parameter("pan_servo_max_angle").as_double()) && (pan_angular_velocity_dps > 0.0f))
    goal_velocity_rpm[_pan_servo_id] = 0.0f;
  if ((tilt_angle_deg < get_parameter("tilt_servo_min_angle").as_double()) && (tilt_angular_velocity_dps < 0.0f))
    goal_velocity_rpm[_tilt_servo_id] = 0.0f;
  if ((tilt_angle_deg > get_parameter("tilt_servo_max_angle").as_double()) && (tilt_angular_velocity_dps > 0.0f))
    goal_velocity_rpm[_tilt_servo_id] = 0.0f;

  /* Write the computed RPM value to the Dynamixel MX-28AR
   * servos of the pan/tilt head.
   */
  _mx28_ctrl->setGoalVelocity(goal_velocity_rpm[_pan_servo_id], goal_velocity_rpm[_tilt_servo_id]);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */
