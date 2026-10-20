cmake_minimum_required(VERSION 3.16)
project(car_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

foreach(directory launch config)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${directory}")
    install(DIRECTORY ${directory} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_pytest_test(test_scenario test/test_scenario.py)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_contact_event.py")
    add_launch_test(test/test_contact_event.py TIMEOUT 300)
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_sensor_bridge.py")
    add_launch_test(test/test_sensor_bridge.py TIMEOUT 360)
  endif()
endif()

ament_package()
