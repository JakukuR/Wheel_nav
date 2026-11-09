cmake_minimum_required(VERSION 3.16)
project(wla_follow_local)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)

# Shared options of the C++ follow core (PY_CPP_ARCHITECTURE.md, conventions).
# -ffp-contract=off keeps results bit-identical to the Python reference.
add_library(wla_follow_options INTERFACE)
target_compile_features(wla_follow_options INTERFACE cxx_std_20)
target_compile_options(wla_follow_options INTERFACE
  -Wall -Wextra -Wpedantic -ffp-contract=off
  "$<$<CONFIG:Debug>:-O0;-g3>"
  "$<$<NOT:$<CONFIG:Debug>>:-O2;-fno-omit-frame-pointer>"
  "$<$<CONFIG:RelWithDebInfo>:-g>")
option(WLA_FOLLOW_COVERAGE "Instrument the follow core for coverage" OFF)
if(WLA_FOLLOW_COVERAGE)
  target_compile_options(wla_follow_options INTERFACE --coverage)
  target_link_options(wla_follow_options INTERFACE --coverage)
endif()

add_library(wla_follow_core STATIC src/rolling_map.cc)
target_include_directories(wla_follow_core PUBLIC include PRIVATE src)
target_link_libraries(wla_follow_core PUBLIC wla_follow_options)
target_compile_options(wla_follow_core PRIVATE -Werror)
set_target_properties(wla_follow_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

pybind11_add_module(_wla_follow_core src/follow_core_module.cc)
target_link_libraries(_wla_follow_core PRIVATE wla_follow_core)

pybind11_add_module(_wla_sim_geometry src/sim_geometry.cpp)
target_include_directories(_wla_sim_geometry PRIVATE src)
target_compile_features(_wla_sim_geometry PRIVATE cxx_std_17)
target_compile_options(_wla_sim_geometry PRIVATE -ffp-contract=off -O2 -Wall -Wextra -Wpedantic)

install(TARGETS _wla_sim_geometry _wla_follow_core LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}")

if(BUILD_TESTING)
  # One executable and one ctest entry per C++ test file.
  file(GLOB WLA_FOLLOW_CPP_TESTS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test/cpp/*.cpp")
  foreach(test_source IN LISTS WLA_FOLLOW_CPP_TESTS)
    get_filename_component(test_name "${test_source}" NAME_WE)
    add_executable("${test_name}" "${test_source}")
    target_link_libraries("${test_name}" PRIVATE wla_follow_core)
    add_test(NAME "${test_name}" COMMAND "${test_name}")
  endforeach()
endif()
ament_package()
