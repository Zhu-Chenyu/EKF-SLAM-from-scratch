cmake_minimum_required(VERSION 3.20)
project(nuslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Armadillo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlelib REQUIRED)

include_directories(include)

# EKF as a library
add_library(ekf_lib src/ekf.cpp)
target_include_directories(ekf_lib PUBLIC include include/nuslam ${ARMADILLO_INCLUDE_DIRS})
target_link_libraries(ekf_lib PUBLIC
  ${ARMADILLO_LIBRARIES}
  turtlelib::turtlelib
)

# Circle fitting as a library
add_library(circle_fit_lib src/circle_fitting.cpp)
target_include_directories(circle_fit_lib PUBLIC include include/nuslam ${ARMADILLO_INCLUDE_DIRS})
target_link_libraries(circle_fit_lib PUBLIC
  ${ARMADILLO_LIBRARIES}
)

# SLAM node
add_executable(nuslam src/nuslam.cpp)
target_link_libraries(nuslam PUBLIC
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2_ros::tf2_ros
  ekf_lib
  circle_fit_lib
)

# Landmark node
add_executable(landmark src/landmark.cpp)
target_link_libraries(landmark PUBLIC
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  circle_fit_lib
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS nuslam landmark ekf_lib circle_fit_lib
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Catch2 REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_executable(circle_tests test/circle_tests.cpp)
  target_link_libraries(circle_tests circle_fit_lib Catch2::Catch2WithMain)
endif()

ament_package()
