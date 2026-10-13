cmake_minimum_required(VERSION 3.16)
project(wla_diff_mpc)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
foreach(dep geometry_msgs nav_msgs nav2_core nav2_costmap_2d nav2_util pluginlib
            rclcpp rclcpp_lifecycle tf2 tf2_geometry_msgs tf2_ros)
  find_package(${dep} REQUIRED)
endforeach()

# One implementation shared by the Nav2 plugin and the person-follow service.
add_library(wla_diff_mpc_core SHARED src/qp.cpp src/solver.cpp src/reference.cpp)
target_compile_features(wla_diff_mpc_core PUBLIC cxx_std_17)
target_include_directories(wla_diff_mpc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(wla_diff_mpc_core Eigen3::Eigen OsqpEigen::OsqpEigen)
ament_target_dependencies(wla_diff_mpc_core geometry_msgs nav_msgs tf2 tf2_geometry_msgs)
set_target_properties(wla_diff_mpc_core PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

add_library(wla_diff_mpc SHARED src/controller.cpp)
target_compile_features(wla_diff_mpc PUBLIC cxx_std_17)
target_include_directories(wla_diff_mpc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(wla_diff_mpc wla_diff_mpc_core)
ament_target_dependencies(wla_diff_mpc geometry_msgs nav_msgs nav2_core nav2_costmap_2d
  nav2_util pluginlib rclcpp rclcpp_lifecycle tf2 tf2_geometry_msgs tf2_ros)
set_target_properties(wla_diff_mpc PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS wla_diff_mpc_core wla_diff_mpc EXPORT export_wla_diff_mpc
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  enable_testing()
  foreach(name terminal_alignment obstacle_braking qp reference)
    add_executable(${name}_regression test/${name}_regression.cpp)
    target_compile_features(${name}_regression PRIVATE cxx_std_17)
    target_compile_options(${name}_regression PRIVATE -UNDEBUG)
    target_link_libraries(${name}_regression wla_diff_mpc_core)
    add_test(NAME ${name}_regression COMMAND ${name}_regression)
  endforeach()
endif()

ament_export_targets(export_wla_diff_mpc HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(wla_diff_mpc wla_diff_mpc_core)
ament_export_dependencies(Eigen3 OsqpEigen geometry_msgs nav_msgs nav2_core nav2_costmap_2d
  nav2_util pluginlib rclcpp rclcpp_lifecycle tf2 tf2_geometry_msgs tf2_ros)
ament_package()
