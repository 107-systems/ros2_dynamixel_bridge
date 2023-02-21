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
      ],
      remappings=[
          ('/l3xz/dynamixel/servo_7/angle/actual',            '/l3xz/head/pan/angle/actual'),
          ('/l3xz/dynamixel/servo_7/angle/target',            '/l3xz/head/pan/angle/target'),
          ('/l3xz/dynamixel/servo_7/angular_velocity/target', '/l3xz/head/pan/angular_velocity/target'),
          ('/l3xz/dynamixel/servo_8/angle/actual',            '/l3xz/head/tilt/angle/actual'),
          ('/l3xz/dynamixel/servo_8/angle/target',            '/l3xz/head/tilt/angle/target'),
          ('/l3xz/dynamixel/servo_8/angular_velocity/target', '/l3xz/head/tilt/angular_velocity/target'),
      ]
    )
  ])
