cmake_minimum_required(VERSION 3.16)
project(wla_diff_mpc)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
foreach(dep geometry_msgs nav_msgs nav2_core nav2_costmap_2d nav2_util pluginlib
            rclcpp rclcpp_lifecycle tf2 tf2_geometry_msgs tf2_ros)
  find_package(${dep} REQUIRED)
endforeach()

add_library(wla_diff_mpc SHARED src/qp.cpp src/solver.cpp src/reference.cpp src/controller.cpp)
target_compile_features(wla_diff_mpc PUBLIC cxx_std_17)
target_include_directories(wla_diff_mpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(wla_diff_mpc Eigen3::Eigen OsqpEigen::OsqpEigen)
ament_target_dependencies(wla_diff_mpc geometry_msgs nav_msgs nav2_core nav2_costmap_2d
  nav2_util pluginlib rclcpp rclcpp_lifecycle tf2 tf2_geometry_msgs tf2_ros)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS wla_diff_mpc LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  enable_testing()
  add_executable(terminal_alignment_regression test/terminal_alignment_regression.cpp)
  target_compile_features(terminal_alignment_regression PRIVATE cxx_std_17)
  target_include_directories(terminal_alignment_regression PRIVATE include)
  add_test(NAME terminal_alignment_regression COMMAND terminal_alignment_regression)
  add_executable(obstacle_braking_regression test/obstacle_braking_regression.cpp src/qp.cpp src/solver.cpp)
  target_compile_features(obstacle_braking_regression PRIVATE cxx_std_17)
  target_include_directories(obstacle_braking_regression PRIVATE include)
  target_link_libraries(obstacle_braking_regression Eigen3::Eigen OsqpEigen::OsqpEigen)
  add_test(NAME obstacle_braking_regression COMMAND obstacle_braking_regression)
  add_executable(qp_regression test/qp_regression.cpp src/qp.cpp src/solver.cpp)
  target_compile_features(qp_regression PRIVATE cxx_std_17)
  target_include_directories(qp_regression PRIVATE include)
  target_link_libraries(qp_regression Eigen3::Eigen OsqpEigen::OsqpEigen)
  add_test(NAME qp_regression COMMAND qp_regression)

  add_executable(reference_regression test/reference_regression.cpp src/reference.cpp)
  target_compile_features(reference_regression PRIVATE cxx_std_17)
  target_include_directories(reference_regression PRIVATE include)
  target_link_libraries(reference_regression Eigen3::Eigen)
  ament_target_dependencies(reference_regression nav_msgs tf2 tf2_geometry_msgs)
  add_test(NAME reference_regression COMMAND reference_regression)
endif()

ament_export_include_directories(include)
ament_export_libraries(wla_diff_mpc)
ament_package()
