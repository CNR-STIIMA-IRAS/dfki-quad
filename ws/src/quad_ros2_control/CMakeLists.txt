cmake_minimum_required(VERSION 3.10)
project(quad_ros2_control)

find_program(ROS_SYSTEM_PYTHON3 python3 PATHS /usr/bin /bin NO_DEFAULT_PATH)
if(ROS_SYSTEM_PYTHON3)
  set(Python3_EXECUTABLE "${ROS_SYSTEM_PYTHON3}" CACHE FILEPATH "Python interpreter used by ament_cmake" FORCE)
endif()

file(GLOB ROS_PYTHON_SITE_PACKAGES "/opt/ros/*/lib/python*/site-packages")
foreach(ROS_PYTHON_SITE_PACKAGE IN LISTS ROS_PYTHON_SITE_PACKAGES)
  if(EXISTS "${ROS_PYTHON_SITE_PACKAGE}/ament_package")
    if(DEFINED ENV{PYTHONPATH})
      set(ENV{PYTHONPATH} "${ROS_PYTHON_SITE_PACKAGE}:$ENV{PYTHONPATH}")
    else()
      set(ENV{PYTHONPATH} "${ROS_PYTHON_SITE_PACKAGE}")
    endif()
  endif()
endforeach()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/test_topic_based_forward_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES CHANGELOG.md README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
