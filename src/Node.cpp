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
, _target_angular_velocity_dps
{
  {Servo::Coxa_Left_Front,   0.0f},
  {Servo::Coxa_Left_Middle,  0.0f},
  {Servo::Coxa_Left_Back,    0.0f},
  {Servo::Coxa_Right_Front,  0.0f},
  {Servo::Coxa_Right_Middle, 0.0f},
  {Servo::Coxa_Right_Back,   0.0f},
  {Servo::Pan,               0.0f},
  {Servo::Tilt,              0.0f},
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

  std::vector<Dynamixel::Id> const L3XZ_DYNAMIXEL_ID_VECT =
  {
    left_front_coxa_servo_id,
    left_middle_coxa_servo_id,
    left_back_coxa_servo_id,
    right_back_coxa_servo_id,
    right_middle_coxa_servo_id,
    right_front_coxa_servo_id,
    pan_servo_id,
    tilt_servo_id,
  };

  bool all_servos_online = true;
  std::stringstream offline_id_list;
  for (auto servo_id : L3XZ_DYNAMIXEL_ID_VECT)
    if (std::none_of(std::cbegin(dyn_id_vect), std::cend(dyn_id_vect), [servo_id, &all_servos_online, &offline_id_list](Dynamixel::Id const id) { return (id == servo_id); }))
    {
      offline_id_list << static_cast<int>(servo_id) << " ";
      all_servos_online = false;
    }
  if (!all_servos_online) {
    RCLCPP_ERROR(get_logger(), "one or more MX-28AR OFF-line: { %s}, shutting down.", offline_id_list.str().c_str());
    rclcpp::shutdown();
    return;
  }

  /* Create a map for individually controlling all the servos. */
  _mx28_ctrl_map[Servo::Coxa_Left_Front]   = std::make_shared<MX28AR::Single>(dyn_ctrl, left_front_coxa_servo_id);
  _mx28_ctrl_map[Servo::Coxa_Left_Middle]  = std::make_shared<MX28AR::Single>(dyn_ctrl, left_middle_coxa_servo_id);
  _mx28_ctrl_map[Servo::Coxa_Left_Back]    = std::make_shared<MX28AR::Single>(dyn_ctrl, left_back_coxa_servo_id);
  _mx28_ctrl_map[Servo::Coxa_Right_Front]  = std::make_shared<MX28AR::Single>(dyn_ctrl, right_front_coxa_servo_id);
  _mx28_ctrl_map[Servo::Coxa_Right_Middle] = std::make_shared<MX28AR::Single>(dyn_ctrl, right_middle_coxa_servo_id);
  _mx28_ctrl_map[Servo::Coxa_Right_Back]   = std::make_shared<MX28AR::Single>(dyn_ctrl, right_back_coxa_servo_id);
  _mx28_ctrl_map[Servo::Pan]               = std::make_shared<MX28AR::Single>(dyn_ctrl, pan_servo_id);
  _mx28_ctrl_map[Servo::Tilt]              = std::make_shared<MX28AR::Single>(dyn_ctrl, tilt_servo_id);

  for (auto [servo, servo_ctrl] : _mx28_ctrl_map)
  {
    servo_ctrl->setTorqueEnable (MX28AR::TorqueEnable::Off);
    servo_ctrl->setOperatingMode(MX28AR::OperatingMode::PositionControlMode);
    servo_ctrl->setTorqueEnable (MX28AR::TorqueEnable::On);

    /* TODO: REMOVE IF */
    if (servo == Servo::Pan || servo == Servo::Tilt)
    {
      servo_ctrl->setGoalPosition(get_parameter("pan_servo_initial_angle").as_double());

      bool target_angle_reached = false;
      float actual_angle_deg = 0.0f;
      for (auto const start = std::chrono::system_clock::now();
           (std::chrono::system_clock::now() - start) < std::chrono::seconds(5) && !target_angle_reached; )
      {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        actual_angle_deg = servo_ctrl->getPresentPosition();

        static float constexpr INITIAL_ANGLE_EPSILON_deg = 2.0f;
        target_angle_reached  = fabs(actual_angle_deg  - get_parameter("pan_servo_initial_angle").as_double())  < INITIAL_ANGLE_EPSILON_deg;
      }

      if (!target_angle_reached)
      {
        RCLCPP_ERROR(get_logger(),
                     "could not reach initial position for servo #id, target: %0.2f, actual: %0.2f.",
                     get_parameter("pan_servo_initial_angle").as_double(),
                     actual_angle_deg);
        rclcpp::shutdown();
      }
    }

    servo_ctrl->setTorqueEnable (MX28AR::TorqueEnable::Off);
    servo_ctrl->setOperatingMode(MX28AR::OperatingMode::VelocityControlMode);
    servo_ctrl->setTorqueEnable (MX28AR::TorqueEnable::On);
    servo_ctrl->setGoalVelocity (0.0f);
  }

  /* Create sync group consisting of all dynamixel servos. */
  _mx28_sync_ctrl = std::make_shared<MX28AR::SyncGroup>(
    dyn_ctrl,
    dynamixelplusplus::Dynamixel::IdVect{left_front_coxa_servo_id,
                                         left_middle_coxa_servo_id,
                                         left_back_coxa_servo_id,
                                         right_front_coxa_servo_id,
                                         right_middle_coxa_servo_id,
                                         right_back_coxa_servo_id,
                                         pan_servo_id,
                                         tilt_servo_id});

  /* Configure subscribers and publishers. */
  _angle_pub[Servo::Coxa_Left_Front]       = create_publisher<std_msgs::msg::Float32>("/l3xz/leg/left_front/coxa/angle/actual", 1);
  _angle_pub[Servo::Coxa_Left_Middle]      = create_publisher<std_msgs::msg::Float32>("/l3xz/leg/left_middle/coxa/angle/actual", 1);
  _angle_pub[Servo::Coxa_Left_Back]        = create_publisher<std_msgs::msg::Float32>("/l3xz/leg/left_back/coxa/angle/actual", 1);
  _angle_pub[Servo::Coxa_Right_Front]      = create_publisher<std_msgs::msg::Float32>("/l3xz/leg/right_front/coxa/angle/actual", 1);
  _angle_pub[Servo::Coxa_Right_Middle]     = create_publisher<std_msgs::msg::Float32>("/l3xz/leg/right_middle/coxa/angle/actual", 1);
  _angle_pub[Servo::Coxa_Right_Back]       = create_publisher<std_msgs::msg::Float32>("/l3xz/leg/right_back/coxa/angle/actual", 1);
  _angle_pub[Servo::Pan]                   = create_publisher<std_msgs::msg::Float32>("/l3xz/head/pan/angle/actual", 1);
  _angle_pub[Servo::Tilt]                  = create_publisher<std_msgs::msg::Float32>("/l3xz/head/pan/angle/actual", 1);

  _angle_vel_sub[Servo::Coxa_Left_Front]   = create_subscription<std_msgs::msg::Float32>("/l3xz/leg/left_front/coxa/angle_velocity/target",   1, [this](std_msgs::msg::Float32::SharedPtr const msg) { _target_angular_velocity_dps[Servo::Coxa_Left_Front] = msg->data * 180.0f / M_PI; });
  _angle_vel_sub[Servo::Coxa_Left_Middle]  = create_subscription<std_msgs::msg::Float32>("/l3xz/leg/left_middle/coxa/angle_velocity/target",  1, [this](std_msgs::msg::Float32::SharedPtr const msg) { _target_angular_velocity_dps[Servo::Coxa_Left_Middle] = msg->data * 180.0f / M_PI; });
  _angle_vel_sub[Servo::Coxa_Left_Back]    = create_subscription<std_msgs::msg::Float32>("/l3xz/leg/left_back/coxa/angle_velocity/target",    1, [this](std_msgs::msg::Float32::SharedPtr const msg) { _target_angular_velocity_dps[Servo::Coxa_Left_Back] = msg->data * 180.0f / M_PI; });
  _angle_vel_sub[Servo::Coxa_Right_Front]  = create_subscription<std_msgs::msg::Float32>("/l3xz/leg/right_front/coxa/angle_velocity/target",  1, [this](std_msgs::msg::Float32::SharedPtr const msg) { _target_angular_velocity_dps[Servo::Coxa_Right_Front] = msg->data * 180.0f / M_PI; });
  _angle_vel_sub[Servo::Coxa_Right_Middle] = create_subscription<std_msgs::msg::Float32>("/l3xz/leg/right_middle/coxa/angle_velocity/target", 1, [this](std_msgs::msg::Float32::SharedPtr const msg) { _target_angular_velocity_dps[Servo::Coxa_Right_Middle] = msg->data * 180.0f / M_PI; });
  _angle_vel_sub[Servo::Coxa_Right_Back]   = create_subscription<std_msgs::msg::Float32>("/l3xz/leg/right_back/coxa/angle_velocity/target",   1, [this](std_msgs::msg::Float32::SharedPtr const msg) { _target_angular_velocity_dps[Servo::Coxa_Right_Back] = msg->data * 180.0f / M_PI; });
  _angle_vel_sub[Servo::Pan]               = create_subscription<std_msgs::msg::Float32>("/l3xz/head/pan/angle_velocity/target",              1, [this](std_msgs::msg::Float32::SharedPtr const msg) { _target_angular_velocity_dps[Servo::Pan] = msg->data * 180.0f / M_PI; });
  _angle_vel_sub[Servo::Tilt]              = create_subscription<std_msgs::msg::Float32>("/l3xz/head/pan/angle_velocity/target",              1, [this](std_msgs::msg::Float32::SharedPtr const msg) { _target_angular_velocity_dps[Servo::Tilt] = msg->data * 180.0f / M_PI; });

  /* Configure periodic control loop function. */
  _io_loop_timer = create_wall_timer
    (std::chrono::milliseconds(IO_LOOP_RATE.count()),
     [this]() { this->io_loop(); });

  RCLCPP_INFO(get_logger(), "node initialization complete.");
}

Node::~Node()
{
  _mx28_sync_ctrl->setGoalVelocity(std::vector<float>{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
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

  /* Synchronously retrieve the current position of each servo. ***********************/
  auto const actual_angle_deg_vect = _mx28_sync_ctrl->getPresentPosition();
  std::map<Servo, float> const actual_angle_deg_map =
  {
    {Servo::Coxa_Left_Front,   actual_angle_deg_vect.at(0)},
    {Servo::Coxa_Left_Middle,  actual_angle_deg_vect.at(1)},
    {Servo::Coxa_Left_Back,    actual_angle_deg_vect.at(2)},
    {Servo::Coxa_Right_Front,  actual_angle_deg_vect.at(3)},
    {Servo::Coxa_Right_Middle, actual_angle_deg_vect.at(4)},
    {Servo::Coxa_Right_Back,   actual_angle_deg_vect.at(5)},
    {Servo::Pan,               actual_angle_deg_vect.at(6)},
    {Servo::Tilt,              actual_angle_deg_vect.at(7)},
  };

  /* Publish the current position via various ROS topics (one per joint). *************/
  auto publishServoAngle = [](rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr const pub, float const angle_deg)
  {
    std_msgs::msg::Float32 msg;
    msg.data = angle_deg;
    pub->publish(msg);
  };

  publishServoAngle(_angle_pub.at(Servo::Coxa_Left_Front),   actual_angle_deg_map.at(Servo::Coxa_Left_Front));
  publishServoAngle(_angle_pub.at(Servo::Coxa_Left_Middle),  actual_angle_deg_map.at(Servo::Coxa_Left_Middle));
  publishServoAngle(_angle_pub.at(Servo::Coxa_Left_Back),    actual_angle_deg_map.at(Servo::Coxa_Left_Back));
  publishServoAngle(_angle_pub.at(Servo::Coxa_Right_Front),  actual_angle_deg_map.at(Servo::Coxa_Right_Front));
  publishServoAngle(_angle_pub.at(Servo::Coxa_Right_Middle), actual_angle_deg_map.at(Servo::Coxa_Right_Middle));
  publishServoAngle(_angle_pub.at(Servo::Coxa_Right_Back),   actual_angle_deg_map.at(Servo::Coxa_Right_Back));
  publishServoAngle(_angle_pub.at(Servo::Pan),               actual_angle_deg_map.at(Servo::Pan));
  publishServoAngle(_angle_pub.at(Servo::Tilt),              actual_angle_deg_map.at(Servo::Tilt));

  /* Calculate RPMs and limit them for all servos. ************************************/
  std::map<Servo, float> target_velocity_rpm_map =
  {
    {Servo::Coxa_Left_Front,   0.0f},
    {Servo::Coxa_Left_Middle,  0.0f},
    {Servo::Coxa_Left_Back,    0.0f},
    {Servo::Coxa_Right_Front,  0.0f},
    {Servo::Coxa_Right_Middle, 0.0f},
    {Servo::Coxa_Right_Back,   0.0f},
    {Servo::Pan,               0.0f},
    {Servo::Tilt,              0.0f},
  };

  static float constexpr DPS_per_RPM = 360.0f / 60.0f;
  target_velocity_rpm_map[Servo::Pan]  = _target_angular_velocity_dps[Servo::Pan]  / DPS_per_RPM;
  target_velocity_rpm_map[Servo::Tilt] = _target_angular_velocity_dps[Servo::Tilt] / DPS_per_RPM;

  /* Checking current head position and stopping if either
   * pan or tilt angle would exceed the maximum allowed angle.
   */

  if ((actual_angle_deg_map.at(Servo::Pan) < get_parameter("pan_servo_min_angle").as_double()) && (_target_angular_velocity_dps[Servo::Pan] < 0.0f))
    target_velocity_rpm_map[Servo::Pan] = 0.0f;
  if ((actual_angle_deg_map.at(Servo::Pan) > get_parameter("pan_servo_max_angle").as_double()) && (_target_angular_velocity_dps[Servo::Pan] > 0.0f))
    target_velocity_rpm_map[Servo::Pan] = 0.0f;
  if ((actual_angle_deg_map.at(Servo::Tilt) < get_parameter("tilt_servo_min_angle").as_double()) && (_target_angular_velocity_dps[Servo::Tilt] < 0.0f))
    target_velocity_rpm_map[Servo::Tilt] = 0.0f;
  if ((actual_angle_deg_map.at(Servo::Tilt) > get_parameter("tilt_servo_max_angle").as_double()) && (_target_angular_velocity_dps[Servo::Tilt] > 0.0f))
    target_velocity_rpm_map[Servo::Tilt] = 0.0f;

  /* Write the computed RPM value to the Dynamixel MX-28AR
   * servos of the pan/tilt head.
   */
  std::vector<float> target_velocity_rpm_vect =
  {
    target_velocity_rpm_map.at(Servo::Coxa_Left_Front),
    target_velocity_rpm_map.at(Servo::Coxa_Left_Middle),
    target_velocity_rpm_map.at(Servo::Coxa_Left_Back),
    target_velocity_rpm_map.at(Servo::Coxa_Right_Front),
    target_velocity_rpm_map.at(Servo::Coxa_Right_Middle),
    target_velocity_rpm_map.at(Servo::Coxa_Right_Back),
    target_velocity_rpm_map.at(Servo::Pan),
    target_velocity_rpm_map.at(Servo::Tilt),
  };
  _mx28_sync_ctrl->setGoalVelocity(target_velocity_rpm_vect);
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

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */
