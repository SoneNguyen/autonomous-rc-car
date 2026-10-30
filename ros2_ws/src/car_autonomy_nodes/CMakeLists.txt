cmake_minimum_required(VERSION 3.16)
project(car_autonomy_nodes)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS car_autonomy_nodes/safe_drive_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME safe_drive_controller
)
ament_package()
