"""rover1.launch.py - everything Rover 1 publishes, plus the bridge the UI uses.

  d500 scanner       LDROBOT's own driver by default (native ~501 rays);
                     lidar_driver:=builtin swaps in our d500_node instead
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
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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

    # WHICH SCANNER DRIVER
    # 'vendor' (default) is LDROBOT's own ldlidar_ros2 on the LD19 profile -
    # the D500 kit's STL-19P speaks that protocol. It publishes the scanner's
    # NATIVE resolution: angle_increment 0.0125 rad, ~501 rays per revolution,
    # which is 5000 measurements/s / 10 Hz. Our own d500_node bins to 360 and
    # throws that resolution away, so the vendor driver is strictly better for
    # anything that will do mapping.
    #
    # 'builtin' selects d500_node instead. It is kept because it is ours: it
    # reports CRC rejects and frame counts the vendor driver does not expose,
    # which is what diagnosed the link when the scanner was suspected. Use it
    # when you need to see whether the SERIAL LINK is healthy, not the map.
    driver = DeclareLaunchArgument(
        'lidar_driver', default_value='vendor',
        description="'vendor' (ldlidar_ros2, native resolution) or 'builtin' "
                    "(d500_node, exposes CRC/frame counters)")
    use_vendor = PythonExpression(["'", LaunchConfiguration('lidar_driver'), "' == 'vendor'"])

    d500_vendor = Node(
        condition=IfCondition(use_vendor),
        package='ldlidar_ros2',
        executable='ldlidar_ros2_node',
        name='d500_vendor',
        output='screen',
        parameters=[{
            'product_name': 'LDLiDAR_LD19',
            'laser_scan_topic_name': 'scan',
            'point_cloud_2d_topic_name': 'pointcloud2d',
            'frame_id': 'base_laser',
            'port_name': LaunchConfiguration('port'),
            'serial_baudrate': 230400,
            'laser_scan_dir': True,
            'enable_angle_crop_func': False,
            'angle_crop_min': 135.0,
            'angle_crop_max': 225.0,
            'range_min': 0.02,
            'range_max': 12.0,
        }],
    )

    # 0.18 m is the VENDOR'S EXAMPLE height, not a measurement of this rover.
    # Fix it before anything navigates off this transform.
    d500_tf = Node(
        condition=IfCondition(use_vendor),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'base_laser'],
    )

    d500 = Node(
        condition=UnlessCondition(use_vendor),
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

    return LaunchDescription([port, frame_id, driver, d500_vendor, d500_tf,
                              d500, thermal, yahboom, rosbridge])
