cmake_minimum_required(VERSION 3.16)
project(car_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

foreach(directory launch)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${directory}")
    install(DIRECTORY ${directory} DESTINATION share/${PROJECT_NAME})
  endif()
endforeach()

# Track new nested YAML/config files so an incremental colcon build installs them.
file(GLOB_RECURSE CAR_BRINGUP_CONFIG_FILES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/config/*")
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_rc_module_matrix_config.py")
    ament_add_pytest_test(test_rc_module_matrix_config test/test_rc_module_matrix_config.py)
  endif()
endif()

ament_package()
