cmake_minimum_required(VERSION 3.8)
project(togsim_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PickCandidate.msg"
  "msg/PickCandidateArray.msg"
  "msg/TrayState.msg"
  "msg/VacuumState.msg"
  "msg/CycleEvent.msg"
  "msg/Kpi.msg"
  "msg/SystemState.msg"
  "msg/Alarm.msg"
  "msg/MotionSegment.msg"
  "srv/SetVacuum.srv"
  "srv/GetPickCandidates.srv"
  "srv/SetSpeedOverride.srv"
  "srv/SetMode.srv"
  "srv/SpawnProducts.srv"
  "srv/IgnService.srv"
  "srv/TeachProduct.srv"
  "srv/ReloadModel.srv"
  "action/ExecuteMotion.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)
ament_package()
