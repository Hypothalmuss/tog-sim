cmake_minimum_required(VERSION 3.8)
project(togsim_perception)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_datagen scripts/train_seg scripts/segmentation_node scripts/pick_pose_node scripts/tray_vacancy_node
  scripts/eval_pick_poses
  DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_perception test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()
ament_package()
