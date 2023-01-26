cmake_minimum_required(VERSION 3.8)
project(bt_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
        include
)

set(library_name ${PROJECT_NAME})

set(dependencies
        rclcpp
        rclcpp_action
        rclcpp_lifecycle
        geometry_msgs
        sensor_msgs
        nav2_msgs
        nav_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        std_srvs
        nav2_util
	    nav2_behavior_tree
        interfaces
        )

add_library(bt_plugin_new_bip_action_node SHARED plugins/new_bip_action_node.cpp)
add_library(bt_plugin_new_wait_package_node SHARED plugins/new_wait_package_node.cpp)


ament_target_dependencies(bt_plugin_new_bip_action_node
        ${dependencies}
        )

ament_target_dependencies(bt_plugin_new_wait_package_node
        ${dependencies}
        )


foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS
        bt_plugin_new_bip_action_node
        bt_plugin_new_wait_package_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(DIRECTORY include/
        DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
