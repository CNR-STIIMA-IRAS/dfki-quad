cmake_minimum_required(VERSION 3.20)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
project(quad_model_adaptation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gait_controller REQUIRED)

# Library (this will be used as the base class for plugins)
add_library(${PROJECT_NAME} SHARED
  src/quad_model_adaptation/kf_model_adaptation.cpp 
  src/quad_model_adaptation/least_squares_model_adaptation.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC c_std_17 cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  common::quad_model_pino
  gait_controller::gait_controller
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install library and export targets
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

endif()

# Export old-style CMake variables
ament_export_include_directories(
  include
)

# Export modern CMake targets
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(Eigen3 common gait_controller)

ament_package()
