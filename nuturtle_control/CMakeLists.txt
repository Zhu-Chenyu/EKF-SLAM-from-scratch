cmake_minimum_required(VERSION 3.20)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(catch_ros2 REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Include directory for package headers
include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Control.srv"
  "srv/InitialPose.srv"
)

# Get typesupport for custom services (circle uses Control, odometry uses InitialPose)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(turtle_control src/turtle_control.cpp)
target_link_libraries(turtle_control
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nuturtlebot_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  turtlelib::turtlelib
)

add_executable(circle src/circle.cpp)
target_link_libraries(circle
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nuturtlebot_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  turtlelib::turtlelib
  "${cpp_typesupport_target}"
)

add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nuturtlebot_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  turtlelib::turtlelib
  "${cpp_typesupport_target}"
)

add_executable(turtle_control_test test/turtle_control_test.cpp)
target_link_libraries(turtle_control_test
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nuturtlebot_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  turtlelib::turtlelib
  catch_ros2::catch_ros2_with_node_main
)

add_executable(turtle_odom_test test/turtle_odom_test.cpp)
target_link_libraries(turtle_odom_test
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${nuturtlebot_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  catch_ros2::catch_ros2_with_node_main
)

add_executable(turtle_circle_test test/turtle_circle_test.cpp)
target_link_libraries(turtle_circle_test
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  catch_ros2::catch_ros2_with_node_main
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(turtle_odom_test "${cpp_typesupport_target}")

install(DIRECTORY test launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  turtle_control
  circle
  odometry
  turtle_control_test
  turtle_odom_test
  turtle_circle_test
  DESTINATION lib/${PROJECT_NAME}
)

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
