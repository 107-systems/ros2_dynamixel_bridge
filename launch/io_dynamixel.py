from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='l3xz_io_dynamixel',
      namespace='l3xz',
      executable='l3xz_io_dynamixel_node',
      name='l3xz_io_dynamixel',
      output='screen',
      emulate_tty=True,
      parameters=[
          {'serial_port' : '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NNZ55-if00-port0'},
          {'serial_port_baudrate': 115200},
          {'pan_servo_id': 7},
          {'tilt_servo_id': 8},
          {'pan_servo_initial_angle': 180.0},
          {'pan_servo_min_angle': 180.0 - 35.0},
          {'pan_servo_max_angle': 180.0 + 35.0},
          {'tilt_servo_initial_angle': 180.0},
          {'tilt_servo_min_angle': 180.0 - 35.0},
          {'tilt_servo_max_angle': 180.0 + 35.0},
      ]
    )
  ])
