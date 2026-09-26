cmake_minimum_required(VERSION 3.8)
project(rebot_teach_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DrawingPath.msg"
  "srv/ListActionGroups.srv"
  "srv/PreviewActionGroup.srv"
  "srv/PreviewActionSequence.srv"
  "srv/ReplayActionGroup.srv"
  "srv/RenameActionGroup.srv"
  "srv/ListActionSequences.srv"
  "srv/CreateShapeAction.srv"
  "srv/DeleteActionGroup.srv"
  "srv/ConfigureShapeMarker.srv"
  "srv/ConfigureTrace.srv"
  "srv/ConfigureForbiddenZone.srv"
  "srv/CopyActionGroup.srv"
  "srv/CheckShapeReachability.srv"
  "srv/SaveActionSequence.srv"
  "srv/ReplayActionSequence.srv"
  "srv/SaveActionGroup.srv"
  "srv/SelectActionGroup.srv"
  "srv/StartActionGroup.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
