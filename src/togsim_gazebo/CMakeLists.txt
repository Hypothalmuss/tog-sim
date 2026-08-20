cmake_minimum_required(VERSION 3.8)
project(togsim_gazebo)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

add_library(togsim_conveyor_system SHARED src/conveyor_system.cc)
target_link_libraries(togsim_conveyor_system
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  ignition-plugin${IGN_PLUGIN_VER}::register)

add_library(togsim_vacuum_gripper_system SHARED src/vacuum_gripper_system.cc)
target_link_libraries(togsim_vacuum_gripper_system
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  ignition-plugin${IGN_PLUGIN_VER}::register)

install(TARGETS togsim_conveyor_system togsim_vacuum_gripper_system
  LIBRARY DESTINATION lib)
install(DIRECTORY worlds models config launch DESTINATION share/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/product_spawner scripts/gen_tray_model DESTINATION lib/${PROJECT_NAME})

# make the plugins and models discoverable as soon as the workspace is sourced
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
ament_package()
