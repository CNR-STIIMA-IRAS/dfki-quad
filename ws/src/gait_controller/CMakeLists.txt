cmake_minimum_required(VERSION 3.20)
project(gait_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common REQUIRED)
find_package(interfaces REQUIRED)

add_library(gait_controller SHARED
  src/gait_controller/adaptive_gait_sequencer.cpp 
  src/gait_controller/bio_gait_sequencer.cpp 
  src/gait_controller/gait.cpp 
  src/gait_controller/simple_gait_sequencer.cpp
  src/gait_controller/raibert_foot_step_planner.cpp
  src/gait_controller/gait_reference_trajectory_planner.cpp
)

target_link_libraries(gait_controller
  common::quad_model_pino interfaces::interfaces__rosidl_typesupport_cpp
)

target_include_directories(gait_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
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


#############
## Install ##
#############
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS
    gait_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(common interfaces rclcpp)
ament_package()
