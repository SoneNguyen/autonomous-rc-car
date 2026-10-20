cmake_minimum_required(VERSION 3.16)
project(car_autonomy_nodes)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  foreach(test_name
      test_safe_drive_controller
      test_lidar_corridor
      test_run_logger
      test_camera_obstacle_detector)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/${test_name}.py")
      ament_add_pytest_test(${test_name} "test/${test_name}.py")
    endif()
  endforeach()
  foreach(test_name test_command_timeout test_lidar_stop test_ai_fallback)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/${test_name}.launch.py")
      add_launch_test("test/${test_name}.launch.py" TIMEOUT 360)
    endif()
  endforeach()
endif()

ament_package()
