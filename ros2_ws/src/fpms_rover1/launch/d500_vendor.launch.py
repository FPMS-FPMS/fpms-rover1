"""d500_vendor.launch.py - the D500 scanner via LDROBOT's OWN ROS 2 driver.

WHY THE LD19 LAUNCH FOR A D500
The D500 Developer Kit's scanner is an STL-19P. Neither vendor repo lists
"D500" or "STL-19P" by name, but the STL-19P speaks the LD19 wire protocol -
which is not a guess: our own LD19 frame decoder read this exact unit for
436,759 frames with ZERO CRC rejects before this driver was introduced. The
vendor's ld19 profile at 230400 baud is therefore the correct one.

WHY A LAUNCH FILE OF OUR OWN RATHER THAN THE VENDOR'S
Theirs hardcodes /dev/ttyUSB0. ttyUSBn reorders between boots, and on this
rover ttyUSB0 is the Yahboom STM32 board - pointing a LiDAR driver at a motor
controller is not a mistake worth risking. This pins the by-id path, which is
stable.

EVERY PARAMETER IS SET EXPLICITLY. The node aborts with
"Invalid topic name: topic name must not be empty string" if any topic name
parameter is left unset, because an unset string parameter defaults to "".
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SCANNER = ('/dev/serial/by-id/'
           'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')


def generate_launch_description():
    port = DeclareLaunchArgument(
        'port', default_value=SCANNER,
        description='Scanner serial device. by-id: ttyUSBn reorders and ttyUSB0 '
                    'is the STM32 board on this rover.')

    lidar = Node(
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='d500_vendor',
        output='screen',
        parameters=[{
            # The STL-19P uses the LD19 profile - see the header.
            'product_name': 'LDLiDAR_LD19',
            'laser_scan_topic_name': 'scan',
            'point_cloud_2d_topic_name': 'pointcloud2d',
            'frame_id': 'base_laser',
            'port_name': LaunchConfiguration('port'),
            'serial_baudrate': 230400,
            # Counter-clockwise, i.e. REP-103. The raw sensor sweeps clockwise;
            # this is the driver's own correction and matches what our decoder
            # had to do by remapping the ranges array.
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'angle_crop_min': 135.0,
            'angle_crop_max': 225.0,
            'range_min': 0.02,
            'range_max': 12.0,
        }],
    )

    # The scanner sits above base_link. 0.18 m is the VENDOR'S EXAMPLE value,
    # not a measurement of this rover - Rover 1's chassis has not been measured.
    # Fix it before anything consumes the transform for navigation.
    tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'base_laser'],
    )

    return LaunchDescription([port, lidar, tf])
