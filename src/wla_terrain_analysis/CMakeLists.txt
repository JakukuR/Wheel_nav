cmake_minimum_required(VERSION 3.8)
project(wla_terrain_analysis)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(terrain_core src/terrain_core.cpp)
target_include_directories(terrain_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
set_target_properties(terrain_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(terrain_analysis src/terrain_analysis_node.cpp)
target_link_libraries(terrain_analysis terrain_core)
ament_target_dependencies(terrain_analysis rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros)

add_executable(terrain_odometry src/terrain_odometry.cpp)
ament_target_dependencies(terrain_odometry rclcpp sensor_msgs nav_msgs)

# ROS-free core checks (synthetic D455 frames over ramps, steps, drops, boxes).
add_executable(test_terrain_core test/test_terrain_core.cpp)
target_link_libraries(test_terrain_core terrain_core)

install(TARGETS terrain_core EXPORT export_${PROJECT_NAME} ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS terrain_analysis terrain_odometry test_terrain_core DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_test(NAME terrain_core COMMAND test_terrain_core)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
