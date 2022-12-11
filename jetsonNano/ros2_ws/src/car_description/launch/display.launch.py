import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='car_description').find('car_description')
    default_model_path = os.path.join(pkg_share, 'src/car_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    world_path=os.path.join(pkg_share, 'world/my_world.sdf')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
	condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
 	 package='joint_state_publisher_gui',
  	executable='joint_state_publisher_gui',
  	name='joint_state_publisher_gui',
  	condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
)

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    laser_scan_matcher = launch_ros.actions.Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        parameters=[{'publish_odom': '/odom', 'publish_tf': True}]
    )


    lidar_node = launch_ros.actions.Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        emulate_tty=True
    )

    camera_node = launch_ros.actions.Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        emulate_tty=True
    )

    system_check_ack_node = launch_ros.actions.Node(
        package="system_check_ack",
        executable="system_check_ack_node",
        emulate_tty=True
    )



    return launch.LaunchDescription([
	launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
	launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        joint_state_publisher_node,
	joint_state_publisher_gui_node,
        robot_state_publisher_node,
	rviz_node,
        laser_scan_matcher,
        lidar_node,
        camera_node,
        system_check_ack_node
    ])

