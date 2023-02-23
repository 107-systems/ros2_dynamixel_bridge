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
          {'required_node_id_list': [1, 2, 3, 4, 5, 6, 7, 8]},
          {'check_required_node_id_list': True},
      ],
    )
  ])
