cmake_minimum_required(VERSION 3.22)
project(rebotarm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY description rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
