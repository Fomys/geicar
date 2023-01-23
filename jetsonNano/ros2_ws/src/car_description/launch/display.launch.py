from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable

from nav2_common.launch import RewrittenYaml
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='car_description').find('car_description')
    default_model_path = os.path.join(pkg_share, 'src/car_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    world_path = os.path.join(pkg_share, 'maps/etage1_secretariat.yaml')
#    world_path = os.path.join(pkg_share, 'maps/carre_blanc.yaml')
#    world_path = os.path.join(pkg_share, 'maps/premier_etage_belle_.yaml')
    bt = os.path.join(pkg_share, 'config/bt.xml')
    ekf = os.path.join(pkg_share, 'config/ekf.yaml')

    slam_localization_params = RewrittenYaml(
	source_file=os.path.join(pkg_share, 'config/slam_localization.yaml'),
	convert_types=True,
	param_rewrites={}
    )

    configured_params = RewrittenYaml(
        source_file=os.path.join(pkg_share, 'config/nav2_params.yaml'),
        convert_types=True,
        param_rewrites={}
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[configured_params,{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[configured_params,],
	condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
 	 package='joint_state_publisher_gui',
  	executable='joint_state_publisher_gui',
  	name='joint_state_publisher_gui',
        parameters=[configured_params,],
  	condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[configured_params,],
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    laser_scan_matcher = launch_ros.actions.Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        parameters=[configured_params,{'publish_tf': True}]
    )


    lidar_node = launch_ros.actions.Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[configured_params, {
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        emulate_tty=True
    )


    system_check_ack_node = launch_ros.actions.Node(
        package="system_check_ack",
        executable="system_check_ack_node",
        emulate_tty=True,
        parameters=[configured_params,],
    )
    amcl = launch_ros.actions.Node(
        package="nav2_amcl",
        executable="amcl",
        name='amcl',
        emulate_tty=True,
        parameters=[configured_params,],
    )
    mapserver = launch_ros.actions.Node(
        package="nav2_map_server",
        executable="map_server",
        name='map_server',
        emulate_tty=True,
        parameters=[configured_params,{'yaml_filename': world_path}],
	output = "screen"
    )

    planner_server_node = launch_ros.actions.Node(
        package="nav2_planner",
        executable="planner_server",
        emulate_tty=True,
        parameters=[configured_params,],
	name = "planner_server",
	output = "screen"
    )

    controller_node = launch_ros.actions.Node(
	package="nav2_controller",
	executable="controller_server",
	name="controller_server",
	parameters=[configured_params,],
	emulate_tty=True,
	output = "screen"
    )

    lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[configured_params,{'autostart': True}, {'node_names': ["map_server", "amcl", "planner_server","controller_server","behavior_server","bt_navigator","smoother_server"]}]
    )

    slam_localization_node = launch_ros.actions.Node(
	package='slam_toolbox',
	executable='localization_slam_toolbox_node',
	name='slam_toolbox',
	output='screen',
	parameters=[slam_localization_params,{"map_file_name": world_path}]
    )

    smoother_server = launch_ros.actions.Node(
	package='nav2_smoother',
        executable='smoother_server',
	name='smoother_server',
	output='screen',
	respawn_delay=2.0,
	parameters=[configured_params]
    )


    behaviors_server = launch_ros.actions.Node(
	package='nav2_behaviors',
	executable='behavior_server',
	name='behavior_server',
	output='screen',
	respawn_delay=2.0,
	parameters=[configured_params]
    )

    bt_navigator_server = launch_ros.actions.Node(
	 package='nav2_bt_navigator',
         executable='bt_navigator',
         name='bt_navigator',
         output='screen',
         respawn_delay=2.0,
         parameters=[configured_params,{"default_bt_xml_filename": bt}]
    )

    waypoint_follower_server = launch_ros.actions.Node(
	 package='nav2_waypoint_follower',
         executable='waypoint_follower',
         name='waypoint_follower',
         output='screen',
         respawn_delay=2.0,
         parameters=[configured_params]
    )

    velocity_smoother = launch_ros.actions.Node(
	package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        respawn_delay=2.0,
        parameters=[configured_params],
    )

    robot_localization = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf]
    )

    return launch.LaunchDescription([
	launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False',
                                            description='Flag to enable use_sim_time'),
	launch.actions.DeclareLaunchArgument(name='gui', default_value='False',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
	rviz_node,
        laser_scan_matcher,
        lidar_node,
        system_check_ack_node,
        mapserver,
        lifecycle_manager,
	planner_server_node,
	controller_node,
	amcl,
	#slam_localization_node,
	smoother_server,
	behaviors_server,
	bt_navigator_server,
	#waypoint_follower_server,
	#velocity_smoother
        robot_localization
    ])
