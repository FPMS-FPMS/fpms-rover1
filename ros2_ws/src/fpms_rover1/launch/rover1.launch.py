"""rover1.launch.py - everything Rover 1 publishes, plus the bridge the UI uses.

  d500_node          the D500 scanner, as sensor_msgs/LaserScan on /scan
  thermal_node       the Thermal Master P1, as Image + temperature + stats
  yahboom_node       the STM32 board: IMU, attitude, battery, encoders. READ ONLY.
  rosbridge_server   ws://<rover>:9090, which the FPMS Cortex panels connect
                     to directly from the browser

ROS_DOMAIN_ID is pinned to 20 by the systemd unit, not here, so a node started
by hand for debugging lands in the same domain as the service.

Humble only. A Jazzy/Iron/Kilted node on this network can crash the robot's
Humble nodes with std::bad_alloc.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = DeclareLaunchArgument(
        'port',
        default_value=('/dev/serial/by-id/'
                       'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'),
        description='LiDAR serial device. Use a by-id path: ttyUSBn reorders between boots.')

    frame_id = DeclareLaunchArgument(
        'frame_id', default_value='laser',
        description='TF frame the scan is stamped with.')

    d500 = Node(
        package='fpms_rover1',
        executable='d500_node',
        name='d500_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'baud': 230400,
            'range_min': 0.02,
            'range_max': 12.0,
            'invert': True,
        }],
    )

    thermal = Node(
        package='fpms_rover1',
        executable='thermal_node',
        name='thermal_node',
        output='screen',
        parameters=[{
            'model': 'p1',
            'frame_id': 'thermal',
            'publish_compressed': True,
            'max_fps': 9.0,
        }],
    )

    # READ ONLY: no /cmd_vel subscriber and no motor write path exist in this
    # node. Rover 1's set_motor sign convention is unverified and gets tested
    # with the wheels off the ground, not inferred over SSH.
    yahboom = Node(
        package='fpms_rover1',
        executable='yahboom_node',
        name='yahboom_node',
        output='screen',
        parameters=[{
            'port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            'frame_id': 'imu_link',
            'enable_auto_report': True,
        }],
    )

    # 0.0.0.0 so the ground station can reach it over FPMS_Net. This is a
    # private WPA2 AP; do not expose this on a venue network.
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090, 'address': '0.0.0.0'}],
    )

    return LaunchDescription([port, frame_id, d500, thermal, yahboom, rosbridge])
