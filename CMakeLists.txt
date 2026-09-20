cmake_minimum_required(VERSION 3.16)
project(wla_r680_navigation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(command_guard src/command_guard.cpp)
ament_target_dependencies(command_guard geometry_msgs rclcpp std_msgs)

add_executable(imu_conditioner src/imu_conditioner.cpp)
ament_target_dependencies(imu_conditioner rclcpp sensor_msgs)

add_executable(interface_monitor src/interface_monitor.cpp)
add_executable(depth_to_points src/depth_to_points.cpp)
ament_target_dependencies(interface_monitor nav_msgs rclcpp sensor_msgs std_msgs)
ament_target_dependencies(depth_to_points rclcpp sensor_msgs)

install(TARGETS command_guard imu_conditioner interface_monitor depth_to_points DESTINATION lib/${PROJECT_NAME})
ament_python_install_package(wla_r680_navigation_py)
install(PROGRAMS scripts/navigation_goal_bridge
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/mapping_mission scripts/prepare_mapping_run scripts/save_manual_map
  scripts/r680_nav.sh
  scripts/resolve_navigation_map scripts/r680_mapping.sh DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_storage test/test_storage.py)
endif()

ament_package()
