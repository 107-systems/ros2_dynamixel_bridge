from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='ros2_dynamixel_bridge',
      namespace='l3xz',
      executable='ros2_dynamixel_bridge_node',
      name='ros2_dynamixel_bridge',
      output='screen',
      emulate_tty=True,
      parameters=[
          {'serial_port' : '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4NNZ55-if00-port0'},
          {'serial_port_baudrate': 2*1000*1000},
          {'required_node_id_list': [1, 2, 3, 4, 5, 6, 7, 8]},
          {'check_required_node_id_list': True},
      ],
      remappings=[
          ('/dynamixel/servo_1/angle/actual',            '/l3xz/leg/left_front/coxa/angle/actual'),
          ('/dynamixel/servo_1/angle/target',            '/l3xz/leg/left_front/coxa/angle/target'),
          ('/dynamixel/servo_1/angular_velocity/target', '/l3xz/leg/left_front/coxa/angular_velocity/target'),
          ('/dynamixel/servo_1/mode/set',                '/l3xz/leg/left_front/coxa/mode/set'),
          ('/dynamixel/servo_2/angle/actual',            '/l3xz/leg/left_middle/coxa/angle/actual'),
          ('/dynamixel/servo_2/angle/target',            '/l3xz/leg/left_middle/coxa/angle/target'),
          ('/dynamixel/servo_2/angular_velocity/target', '/l3xz/leg/left_middle/coxa/angular_velocity/target'),
          ('/dynamixel/servo_2/mode/set',                '/l3xz/leg/left_middle/coxa/mode/set'),
          ('/dynamixel/servo_3/angle/actual',            '/l3xz/leg/left_back/coxa/angle/actual'),
          ('/dynamixel/servo_3/angle/target',            '/l3xz/leg/left_back/coxa/angle/target'),
          ('/dynamixel/servo_3/angular_velocity/target', '/l3xz/leg/left_back/coxa/angular_velocity/target'),
          ('/dynamixel/servo_3/mode/set',                '/l3xz/leg/left_back/coxa/mode/set'),
          ('/dynamixel/servo_4/angle/actual',            '/l3xz/leg/right_back/coxa/angle/actual'),
          ('/dynamixel/servo_4/angle/target',            '/l3xz/leg/right_back/coxa/angle/target'),
          ('/dynamixel/servo_4/angular_velocity/target', '/l3xz/leg/right_back/coxa/angular_velocity/target'),
          ('/dynamixel/servo_4/mode/set',                '/l3xz/leg/right_back/coxa/mode/set'),
          ('/dynamixel/servo_5/angle/actual',            '/l3xz/leg/right_middle/coxa/angle/actual'),
          ('/dynamixel/servo_5/angle/target',            '/l3xz/leg/right_middle/coxa/angle/target'),
          ('/dynamixel/servo_5/angular_velocity/target', '/l3xz/leg/right_middle/coxa/angular_velocity/target'),
          ('/dynamixel/servo_5/mode/set',                '/l3xz/leg/right_middle/coxa/mode/set'),
          ('/dynamixel/servo_6/angle/actual',            '/l3xz/leg/right_front/coxa/angle/actual'),
          ('/dynamixel/servo_6/angle/target',            '/l3xz/leg/right_front/coxa/angle/target'),
          ('/dynamixel/servo_6/angular_velocity/target', '/l3xz/leg/right_front/coxa/angular_velocity/target'),
          ('/dynamixel/servo_6/mode/set',                '/l3xz/leg/right_front/coxa/mode/set'),
          ('/dynamixel/servo_7/angle/actual',            '/l3xz/head/pan/angle/actual'),
          ('/dynamixel/servo_7/angle/target',            '/l3xz/head/pan/angle/target'),
          ('/dynamixel/servo_7/angular_velocity/target', '/l3xz/head/pan/angular_velocity/target'),
          ('/dynamixel/servo_7/mode/set',                '/l3xz/head/pan/mode/set'),
          ('/dynamixel/servo_8/angle/actual',            '/l3xz/head/tilt/angle/actual'),
          ('/dynamixel/servo_8/angle/target',            '/l3xz/head/tilt/angle/target'),
          ('/dynamixel/servo_8/angular_velocity/target', '/l3xz/head/tilt/angular_velocity/target'),
          ('/dynamixel/servo_8/mode/set',                '/l3xz/head/tilt/mode/set'),
      ],
    )
  ])
