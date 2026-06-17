cmake_minimum_required(VERSION 3.20)
project(quad_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common REQUIRED)
find_package(interfaces REQUIRED)
find_package(Matplot++ REQUIRED)


# ##############################################################################
# --- Gait Plotting --- #
# ##############################################################################

set(MATPLOT_SANITIZER_LINK_FLAGS "")
set(MATPLOT_SANITIZER_RUNTIME_LIBS "")
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Workspace matplot++ may be built with ASan/UBSan instrumentation.
    # Add runtime linkage to prevent unresolved __asan/__ubsan symbols.
    set(MATPLOT_SANITIZER_LINK_FLAGS -fsanitize=address -fsanitize=undefined)
    list(APPEND MATPLOT_SANITIZER_RUNTIME_LIBS asan ubsan)
endif ()

add_executable(gait_plotting src/plotting.cpp)

target_link_libraries(gait_plotting Eigen3::Eigen rclcpp::rclcpp common::quad_model_pino interfaces::interfaces__rosidl_typesupport_cpp)

target_link_libraries(gait_plotting Matplot++::matplot ${MATPLOT_SANITIZER_RUNTIME_LIBS})
target_link_options(gait_plotting PRIVATE ${MATPLOT_SANITIZER_LINK_FLAGS})

target_include_directories(
        gait_plotting
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# ##############################################################################
# --- Gait Sequence Plotting --- #
# ##############################################################################

add_executable(gait_sequence_plotting src/plot_gs.cpp)

target_link_libraries(gait_sequence_plotting Eigen3::Eigen rclcpp::rclcpp common::quad_model_pino interfaces::interfaces__rosidl_typesupport_cpp)
target_link_libraries(gait_sequence_plotting Matplot++::matplot ${MATPLOT_SANITIZER_RUNTIME_LIBS})
target_link_options(gait_sequence_plotting PRIVATE ${MATPLOT_SANITIZER_LINK_FLAGS})

target_include_directories(
        gait_sequence_plotting
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

        
# ##############################################################################
# --- CPU power consumption logging--- #
# ##############################################################################
add_executable(log_cpu_power src/log_cpu_power.cpp)

target_link_libraries(log_cpu_power rclcpp::rclcpp std_msgs::std_msgs__rosidl_typesupport_cpp)

target_include_directories(
        log_cpu_power
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


# ##############################################################################
# --- Trajectory Controller --- #
# ##############################################################################
add_executable(trajectory_replay src/trajectory_replay.cpp)

target_link_libraries(trajectory_replay interfaces::interfaces__rosidl_typesupport_cpp rclcpp::rclcpp ament_index_cpp::ament_index_cpp
        sensor_msgs::sensor_msgs__rosidl_typesupport_cpp geometry_msgs::geometry_msgs__rosidl_typesupport_cpp common::quad_model_symbolic)
target_link_libraries(trajectory_replay common::quad_model_symbolic)

target_include_directories(
        trajectory_replay
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
install(TARGETS trajectory_replay DESTINATION lib/${PROJECT_NAME})




# ##############################################################################
# --- Sys ID Leg Controller --- #
# ##############################################################################

add_executable(SysIDLegControllerNode src/SysIDLegControllerNode.cpp src/SysIDTrajectory.cpp)

target_link_libraries(SysIDLegControllerNode Eigen3::Eigen interfaces::interfaces__rosidl_typesupport_cpp rclcpp::rclcpp common::quad_model_symbolic)
target_link_libraries(SysIDLegControllerNode common::quad_model_symbolic)

target_include_directories(
        SysIDLegControllerNode
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
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

ament_package()
