from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_ros_dynamixel_bridge',
      namespace='l3xz',
      executable='l3xz_ros_dynamixel_bridge_node',
      name='l3xz_ros_dynamixel_bridge',
      output='screen',
      emulate_tty=True,
      parameters=[
          {'serial_port' : '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NNZ55-if00-port0'},
          {'serial_port_baudrate': 2*1000*1000},

          {'left_front_coxa_servo_id': 1},
          {'left_front_coxa_servo_initial_angle': 180.0},
          {'left_front_coxa_servo_min_angle': 180.0 - 35.0},
          {'left_front_coxa_servo_max_angle': 180.0 + 35.0},

          {'left_middle_coxa_servo_id': 2},
          {'left_middle_coxa_servo_initial_angle': 180.0},
          {'left_middle_coxa_servo_min_angle': 180.0 - 35.0},
          {'left_middle_coxa_servo_max_angle': 180.0 + 35.0},

          {'left_back_coxa_servo_id': 3},
          {'left_back_coxa_servo_initial_angle': 180.0},
          {'left_back_coxa_servo_min_angle': 180.0 - 35.0},
          {'left_back_coxa_servo_max_angle': 180.0 + 35.0},

          {'right_back_coxa_servo_id': 4},
          {'right_back_coxa_servo_initial_angle': 180.0},
          {'right_back_coxa_servo_min_angle': 180.0 - 35.0},
          {'right_back_coxa_servo_max_angle': 180.0 + 35.0},

          {'right_middle_coxa_servo_id': 5},
          {'right_middle_coxa_servo_initial_angle': 180.0},
          {'right_middle_coxa_servo_min_angle': 180.0 - 35.0},
          {'right_middle_coxa_servo_max_angle': 180.0 + 35.0},

          {'right_front_coxa_servo_id': 6},
          {'right_front_coxa_servo_initial_angle': 180.0},
          {'right_front_coxa_servo_min_angle': 180.0 - 35.0},
          {'right_front_coxa_servo_max_angle': 180.0 + 35.0},

          {'pan_servo_id': 7},
          {'pan_servo_initial_angle': 180.0},
          {'pan_servo_min_angle': 180.0 - 35.0},
          {'pan_servo_max_angle': 180.0 + 35.0},

          {'tilt_servo_id': 8},
          {'tilt_servo_initial_angle': 180.0},
          {'tilt_servo_min_angle': 180.0 - 35.0},
          {'tilt_servo_max_angle': 180.0 + 35.0},
      ]
    )
  ])
