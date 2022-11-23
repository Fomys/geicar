from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    can_rx_node = Node(
        package="can",
        executable="can_rx_node",
        emulate_tty=True
    )

    can_tx_node = Node(
        package="can",
        executable="can_tx_node",
        emulate_tty=True
    )

    obstacle_detection = Node(
        package="obs_detect",
        executable="obstacle_detection",
        emulate_tty=True
    )

    secu = Node(
        package="secu",
        executable="secu",
        emulate_tty=True
    )

    asservissement = Node(
        package="asservissement",
        executable="asservissement",
        emulate_tty=True
    )



    config_dir = os.path.join(get_package_share_directory('imu_filter_madgwick'), 'config')

    imu_filter_madgwick_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        parameters=[os.path.join(config_dir, 'imu_filter.yaml')],
        emulate_tty=True
    )


    system_check_node = Node(
        package="system_check",
        executable="system_check_node",
        emulate_tty=True
    )
    web_server = Node(
        package="web_interface",
        executable="web_interface",
        emulate_tty=True
    )
    joystick = Node(
        package="joystick_to_cmd",
        executable="joystick_to_cmd_node",
        emulate_tty=True
    )
    car_control = Node(
        package="car_control",
        executable="car_control_node",
        emulate_tty=True
    )

ld.add_action(secu)
    ld.add_action(obstacle_detection)
    ld.add_action(can_rx_node)
    ld.add_action(can_tx_node)
    ld.add_action(imu_filter_madgwick_node)
    ld.add_action(system_check_node)
    # ld.add_action(asservissement)
    ld.add_action(web_server)
    ld.add_action(joystick)
    ld.add_action(car_control)

    return ld
