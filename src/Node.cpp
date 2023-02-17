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
, _head_vel_msg
{
  []()
  {
    l3xz_ros_dynamixel_bridge::msg::HeadVelocity msg;
    msg.pan_vel_rad_per_sec = 0.0f;
    msg.tilt_vel_rad_per_sec = 0.0f;
    return msg;
  } ()
}
, _prev_io_loop_timepoint{std::chrono::steady_clock::now()}
{
  declare_parameter_all();

  /* Configure the Dynamixel MX-28AR servos of the pan/tilt head. */
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

  Dynamixel::Id const left_front_coxa_servo_id   = static_cast<Dynamixel::Id>(get_parameter("left_front_coxa_servo_id").as_int());
  Dynamixel::Id const left_middle_coxa_servo_id  = static_cast<Dynamixel::Id>(get_parameter("left_middle_coxa_servo_id").as_int());
  Dynamixel::Id const left_back_coxa_servo_id    = static_cast<Dynamixel::Id>(get_parameter("left_back_coxa_servo_id").as_int());
  Dynamixel::Id const right_front_coxa_servo_id  = static_cast<Dynamixel::Id>(get_parameter("right_front_coxa_servo_id").as_int());
  Dynamixel::Id const right_middle_coxa_servo_id = static_cast<Dynamixel::Id>(get_parameter("right_middle_coxa_servo_id").as_int());
  Dynamixel::Id const right_back_coxa_servo_id   = static_cast<Dynamixel::Id>(get_parameter("right_back_coxa_servo_id").as_int());
  Dynamixel::Id const pan_servo_id               = static_cast<Dynamixel::Id>(get_parameter("pan_servo_id").as_int());
  Dynamixel::Id const tilt_servo_id              = static_cast<Dynamixel::Id>(get_parameter("tilt_servo_id").as_int());

  std::vector<std::tuple<std::string, Dynamixel::Id>> const L3XZ_DYNAMIXEL_ID_VECT =
  {
    std::make_tuple("left front coxa",   left_front_coxa_servo_id),
    std::make_tuple("left middle coxa",  left_middle_coxa_servo_id),
    std::make_tuple("left back coxa",    left_back_coxa_servo_id),
    std::make_tuple("right front coxa",  right_front_coxa_servo_id),
    std::make_tuple("right middle coxa", right_middle_coxa_servo_id),
    std::make_tuple("right back coxa",   right_back_coxa_servo_id),
    std::make_tuple("pan",               pan_servo_id),
  };

  for (auto [servo_str, servo_id] : L3XZ_DYNAMIXEL_ID_VECT)
    if (std::none_of(std::cbegin(dyn_id_vect), std::cend(dyn_id_vect), [servo_id](Dynamixel::Id const id) { return (id == servo_id); }))
    {
      RCLCPP_ERROR(get_logger(), "%s servo with configured id %d not online.", servo_str.c_str(), static_cast<int>(servo_id));
      rclcpp::shutdown();
    }

  /* Initialize pan/tilt head. */
  RCLCPP_INFO(get_logger(), "initialize pan/tilt servo.");
  _mx28_head_sync_ctrl = std::make_shared<MX28AR::HeadSyncGroup>(dyn_ctrl, pan_servo_id, tilt_servo_id);
  init_pan_tilt_servos();

  /* Initialize coxa servos. */
  RCLCPP_INFO(get_logger(), "initialize all coxa servos.");
  _mx28_coxa_sync_ctrl = std::make_shared<MX28AR::CoxaSyncGroup>(dyn_ctrl,
                                                                 left_front_coxa_servo_id,
                                                                 left_middle_coxa_servo_id,
                                                                 left_back_coxa_servo_id,
                                                                 right_front_coxa_servo_id,
                                                                 right_middle_coxa_servo_id,
                                                                 right_back_coxa_servo_id);
  init_coxa_servos();

  /* Configure subscribers and publishers. */
  _head_io_sub = create_subscription<l3xz_ros_dynamixel_bridge::msg::HeadVelocity>
    ("/l3xz/head/velocity/target", 1,
    [this](l3xz_ros_dynamixel_bridge::msg::HeadVelocity::SharedPtr const msg)
    {
      _head_vel_msg = *msg;
    });

  /* Configure periodic control loop function. */

  _io_loop_timer = create_wall_timer
    (std::chrono::milliseconds(IO_LOOP_RATE.count()),
     [this]()
     {
      this->io_loop();
     });

  RCLCPP_INFO(get_logger(), "node initialization complete.");
}

Node::~Node()
{
  _mx28_head_sync_ctrl->setGoalVelocity(0.0, 0.0);
  _mx28_head_sync_ctrl->setTorqueEnable(MX28AR::TorqueEnable::Off);

  _mx28_coxa_sync_ctrl->setGoalVelocity(0.0);
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::io_loop()
{
  auto const now = std::chrono::steady_clock::now();
  auto const io_loop_rate = (now - _prev_io_loop_timepoint);
  if (io_loop_rate > (IO_LOOP_RATE + std::chrono::milliseconds(1)))
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         1000,
                         "io_loop should be called every %ld ms, but is %ld ms instead",
                         IO_LOOP_RATE.count(),
                         std::chrono::duration_cast<std::chrono::milliseconds>(io_loop_rate).count());
  _prev_io_loop_timepoint = now;


  float pan_goal_velocity_rpm = 0.0f,
        tilt_goal_velocity_rpm = 0.0f;

  float const pan_angular_velocity_dps  = _head_vel_msg.pan_vel_rad_per_sec  * 180.0f / M_PI;
  float const tilt_angular_velocity_dps = _head_vel_msg.tilt_vel_rad_per_sec * 180.0f / M_PI;

  static float constexpr DPS_per_RPM = 360.0f / 60.0f;
  pan_goal_velocity_rpm  = pan_angular_velocity_dps  / DPS_per_RPM;
  tilt_goal_velocity_rpm = tilt_angular_velocity_dps / DPS_per_RPM;

  /* Checking current head position and stopping if either
   * pan or tilt angle would exceed the maximum allowed angle.
   */
  auto [pan_angle_deg, tilt_angle_deg] = _mx28_head_sync_ctrl->getPresentPosition();

  if ((pan_angle_deg < get_parameter("pan_servo_min_angle").as_double()) && (pan_angular_velocity_dps < 0.0f))
    pan_goal_velocity_rpm = 0.0f;
  if ((pan_angle_deg > get_parameter("pan_servo_max_angle").as_double()) && (pan_angular_velocity_dps > 0.0f))
    pan_goal_velocity_rpm = 0.0f;
  if ((tilt_angle_deg < get_parameter("tilt_servo_min_angle").as_double()) && (tilt_angular_velocity_dps < 0.0f))
    tilt_goal_velocity_rpm = 0.0f;
  if ((tilt_angle_deg > get_parameter("tilt_servo_max_angle").as_double()) && (tilt_angular_velocity_dps > 0.0f))
    tilt_goal_velocity_rpm = 0.0f;

  /* Write the computed RPM value to the Dynamixel MX-28AR
   * servos of the pan/tilt head.
   */
  _mx28_head_sync_ctrl->setGoalVelocity(pan_goal_velocity_rpm, tilt_goal_velocity_rpm);
}

void Node::declare_parameter_all()
{
  declare_parameter("serial_port", "/dev/ttyUSB0");
  declare_parameter("serial_port_baudrate", 115200);

  declare_parameter("left_front_coxa_servo_id", 1);
  declare_parameter("left_front_coxa_servo_initial_angle", 180.0f);
  declare_parameter("left_front_coxa_servo_min_angle", 170.0f);
  declare_parameter("left_front_coxa_servo_max_angle", 190.0f);

  declare_parameter("left_middle_coxa_servo_id", 2);
  declare_parameter("left_middle_coxa_servo_initial_angle", 180.0f);
  declare_parameter("left_middle_coxa_servo_min_angle", 170.0f);
  declare_parameter("left_middle_coxa_servo_max_angle", 190.0f);

  declare_parameter("left_back_coxa_servo_id", 3);
  declare_parameter("left_back_coxa_servo_initial_angle", 180.0f);
  declare_parameter("left_back_coxa_servo_min_angle", 170.0f);
  declare_parameter("left_back_coxa_servo_max_angle", 190.0f);

  declare_parameter("right_back_coxa_servo_id", 4);
  declare_parameter("right_back_coxa_servo_initial_angle", 180.0f);
  declare_parameter("right_back_coxa_servo_min_angle", 170.0f);
  declare_parameter("right_back_coxa_servo_max_angle", 190.0f);

  declare_parameter("right_middle_coxa_servo_id", 5);
  declare_parameter("right_middle_coxa_servo_initial_angle", 180.0f);
  declare_parameter("right_middle_coxa_servo_min_angle", 170.0f);
  declare_parameter("right_middle_coxa_servo_max_angle", 190.0f);

  declare_parameter("right_front_coxa_servo_id", 6);
  declare_parameter("right_front_coxa_servo_initial_angle", 180.0f);
  declare_parameter("right_front_coxa_servo_min_angle", 170.0f);
  declare_parameter("right_front_coxa_servo_max_angle", 190.0f);

  declare_parameter("pan_servo_id", 7);
  declare_parameter("pan_servo_initial_angle", 180.0f);
  declare_parameter("pan_servo_min_angle", 170.0f);
  declare_parameter("pan_servo_max_angle", 190.0f);

  declare_parameter("tilt_servo_id", 8);
  declare_parameter("tilt_servo_initial_angle", 180.0f);
  declare_parameter("tilt_servo_min_angle", 170.0f);
  declare_parameter("tilt_servo_max_angle", 190.0f);
}

void Node::init_pan_tilt_servos()
{
  _mx28_head_sync_ctrl->setTorqueEnable (MX28AR::TorqueEnable::Off);
  _mx28_head_sync_ctrl->setOperatingMode(MX28AR::OperatingMode::PositionControlMode);
  _mx28_head_sync_ctrl->setTorqueEnable (MX28AR::TorqueEnable::On);
  _mx28_head_sync_ctrl->setGoalPosition (get_parameter("pan_servo_initial_angle").as_double(), get_parameter("tilt_servo_initial_angle").as_double());

  bool pan_target_reached = false, tilt_target_reached = false;
  float actual_pan_angle_deg = 0.0f, actual_tilt_angle_deg = 0.0f;
  for (auto const start = std::chrono::system_clock::now();
       (std::chrono::system_clock::now() - start) < std::chrono::seconds(5) && !pan_target_reached && !tilt_target_reached; )
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));

    auto [pan_angle_deg, tilt_angle_deg] = _mx28_head_sync_ctrl->getPresentPosition();

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

  _mx28_head_sync_ctrl->setTorqueEnable (MX28AR::TorqueEnable::Off);
  _mx28_head_sync_ctrl->setOperatingMode(MX28AR::OperatingMode::VelocityControlMode);
  _mx28_head_sync_ctrl->setTorqueEnable (MX28AR::TorqueEnable::On);
  _mx28_head_sync_ctrl->setGoalVelocity (0.0, 0.0);
}

void Node::init_coxa_servos()
{
  _mx28_coxa_sync_ctrl->setTorqueEnable (MX28AR::TorqueEnable::Off);
  _mx28_coxa_sync_ctrl->setOperatingMode(MX28AR::OperatingMode::PositionControlMode);
  _mx28_coxa_sync_ctrl->setTorqueEnable (MX28AR::TorqueEnable::On);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */
