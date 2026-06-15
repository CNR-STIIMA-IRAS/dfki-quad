cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()
project(controllers)

option(ON_PI "On Raspberry Pi" FALSE)
if (ON_PI)
    message(WARNING "Skipping package on Raspberry Pi.")
    return()
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)
IF (${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
else ()
    set(CMAKE_CXX_FLAGS_RELEASE "-Ofast")
    message(WARNING "detected non ARM architecture, applying -Ofast - this might be harmfull!")
endif ()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_export_targets REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP) # for speeding up eigens matrix multiplications
if (NOT WITHOUT_DRAKE)
    set(DRAKE_INSTALL_PREFIX "" CACHE PATH "Drake install prefix")
    foreach(_drake_prefix IN ITEMS
            "${DRAKE_INSTALL_PREFIX}"
            "$ENV{DRAKE_INSTALL_PREFIX}"
            "/home/feynman/drake/drake-build/install"
            "/home/feynman/drake/drakebuild/install"
            "/opt/drake")
        if(_drake_prefix AND EXISTS "${_drake_prefix}")
            list(PREPEND CMAKE_PREFIX_PATH "${_drake_prefix}")
        endif()
    endforeach()
    find_package(drake CONFIG REQUIRED)
    find_package(simulator REQUIRED)
endif ()
find_package(acados REQUIRED)
find_package(Matplot++ REQUIRED)
find_package(unitree_go REQUIRED)
find_package(fmt REQUIRED)
find_package(PkgConfig REQUIRED)

set(MATPLOT_SANITIZER_LINK_FLAGS "")
set(MATPLOT_SANITIZER_RUNTIME_LIBS "")
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    # Workspace matplot++ may be built with ASan/UBSan instrumentation.
    # Add runtime linkage to prevent unresolved __asan/__ubsan symbols.
    set(MATPLOT_SANITIZER_LINK_FLAGS -fsanitize=address -fsanitize=undefined)
    list(APPEND MATPLOT_SANITIZER_RUNTIME_LIBS asan ubsan)
endif ()

function(target_ros_dependencies target)
    foreach(dependency IN LISTS ARGN)
        if(dependency STREQUAL "Eigen3")
            target_link_libraries(${target} Eigen3::Eigen)
        elseif(dependency STREQUAL "interfaces")
            target_link_libraries(${target}
                    interfaces::interfaces__rosidl_typesupport_cpp)
        elseif(dependency STREQUAL "rclcpp")
            target_link_libraries(${target} rclcpp::rclcpp)
        elseif(dependency STREQUAL "ament_index_cpp")
            target_link_libraries(${target} ament_index_cpp::ament_index_cpp)
        elseif(dependency STREQUAL "common")
            target_link_libraries(${target} common::quad_model_pino)
        elseif(dependency STREQUAL "std_msgs")
            target_link_libraries(${target}
                    std_msgs::std_msgs__rosidl_typesupport_cpp)
        elseif(dependency STREQUAL "sensor_msgs")
            target_link_libraries(${target}
                    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)
        elseif(dependency STREQUAL "geometry_msgs")
            target_link_libraries(${target}
                    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp)
        elseif(dependency STREQUAL "drake")
            target_link_libraries(${target} drake::drake)
        elseif(dependency STREQUAL "simulator")
            target_link_libraries(${target} simulator::drake_visualizers)
        else()
            message(FATAL_ERROR "Unsupported dependency ${dependency}")
        endif()
    endforeach()
endfunction()


# ARCOPT
pkg_search_module(wbc-core REQUIRED IMPORTED_TARGET wbc-core)
pkg_search_module(wbc-robot_models-pinocchio REQUIRED IMPORTED_TARGET wbc-robot_models-pinocchio)
pkg_search_module(wbc-solvers-qpoases REQUIRED IMPORTED_TARGET wbc-solvers-qpoases)
pkg_search_module(wbc-scenes-acceleration_reduced_tsid REQUIRED IMPORTED_TARGET wbc-scenes-acceleration_reduced_tsid)
pkg_search_module(wbc-scenes-acceleration_tsid REQUIRED IMPORTED_TARGET wbc-scenes-acceleration_tsid)
pkg_search_module(wbc-controllers REQUIRED IMPORTED_TARGET wbc-controllers)
pkg_search_module(wbc-types REQUIRED IMPORTED_TARGET wbc-types)
pkg_search_module(wbc-solvers-eiquadprog QUIET IMPORTED_TARGET wbc-solvers-eiquadprog)
pkg_search_module(wbc-solvers-qpswift QUIET IMPORTED_TARGET wbc-solvers-qpswift)
pkg_search_module(wbc-solvers-proxqp QUIET IMPORTED_TARGET wbc-solvers-proxqp)
pkg_search_module(wbc-solvers-osqp QUIET IMPORTED_TARGET wbc-solvers-osqp)
pkg_search_module(wbc-solvers-hpipm QUIET IMPORTED_TARGET wbc-solvers-hpipm)
# pkg_search_module(base-types REQUIRED IMPORTED_TARGET base-types)


include_directories(include include/mit_controller src)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights comment the
    # line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo) comment the line
    # when this package is in a git repo and when a copyright and license is added
    # to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()


# ##############################################################################
# --- Trajectory Controller --- #
# ##############################################################################

if (NOT WITHOUT_DRAKE)
    add_executable(trajectory_replay src/trajectory_replay.cpp)

    target_ros_dependencies(trajectory_replay interfaces rclcpp ament_index_cpp
            sensor_msgs geometry_msgs common)
    target_link_libraries(trajectory_replay common::quad_model_symbolic)

    target_include_directories(
            trajectory_replay
            PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)
    install(TARGETS trajectory_replay DESTINATION lib/${PROJECT_NAME})

endif ()
# ##############################################################################
# --- Gait Plotting --- #
# ##############################################################################

add_executable(gait_plotting src/tools/plotting.cpp)

target_ros_dependencies(gait_plotting Eigen3 interfaces rclcpp common
)
target_link_libraries(gait_plotting Matplot++::matplot ${MATPLOT_SANITIZER_RUNTIME_LIBS})
target_link_options(gait_plotting PRIVATE ${MATPLOT_SANITIZER_LINK_FLAGS})

target_include_directories(
        gait_plotting
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# ##############################################################################
# --- Gait Sequence Plotting --- #
# ##############################################################################

add_executable(gait_sequence_plotting src/tools/plot_gs.cpp)

target_ros_dependencies(gait_sequence_plotting Eigen3 interfaces rclcpp common
)
target_link_libraries(gait_sequence_plotting Matplot++::matplot ${MATPLOT_SANITIZER_RUNTIME_LIBS})
target_link_options(gait_sequence_plotting PRIVATE ${MATPLOT_SANITIZER_LINK_FLAGS})

target_include_directories(
        gait_sequence_plotting
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

# ##############################################################################
# --- MIT CONTROLLER--- #
# ##############################################################################
if (NOT WITHOUT_DRAKE)
    add_executable(mitcontroller
            src/mit_controller/mit_controller.cpp
            src/mit_controller/mpc.cpp
            src/mit_controller/gait.cpp
            src/mit_controller/simple_gait_sequencer.cpp
            src/mit_controller/bio_gait_sequencer.cpp
            src/mit_controller/adaptive_gait_sequencer.cpp
            src/mit_controller/mpc_trajectory_planner.cpp
            src/mit_controller/raibert_foot_step_planner.cpp
            src/mit_controller/swing_leg_controller.cpp
            src/potato_sim/potato_sim.cpp
            src/potato_sim/potato_model.cpp
            src/SwingTrajectory.cpp
            src/model_adaptation/least_squares_model_adaptation.cpp
            src/model_adaptation/kf_model_adaptation.cpp
    )

    target_ros_dependencies(mitcontroller Eigen3 interfaces common drake simulator)
    target_link_libraries(mitcontroller acados Matplot++::matplot ${MATPLOT_SANITIZER_RUNTIME_LIBS})
    target_link_options(mitcontroller PRIVATE ${MATPLOT_SANITIZER_LINK_FLAGS})
    target_compile_options(mitcontroller PRIVATE -Wall -Wextra -Wpedantic -Werror)
    ament_export_targets(mitcontroller HAS_LIBRARY_TARGET)
    install(
            TARGETS mitcontroller
            EXPORT mitcontroller
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            RUNTIME DESTINATION bin
            INCLUDES
            DESTINATION include)
endif ()


# ##############################################################################
# --- MIT CONTROLLER NODE --- #
# ##############################################################################
add_executable(mitcontrollernode
        src/mit_controller_node.cpp
        src/mit_controller/mpc.cpp
        src/mit_controller/gait.cpp
        src/mit_controller/simple_gait_sequencer.cpp
        src/mit_controller/adaptive_gait_sequencer.cpp
        src/mit_controller/bio_gait_sequencer.cpp
        src/mit_controller/mpc_trajectory_planner.cpp
        src/mit_controller/raibert_foot_step_planner.cpp
        src/mit_controller/swing_leg_controller.cpp
        src/mit_controller/inverse_dynamics.cpp
        src/mit_controller/wbc_arc_opt.cpp
        src/model_adaptation/least_squares_model_adaptation.cpp
        src/model_adaptation/kf_model_adaptation.cpp
        src/SwingTrajectory.cpp
)

set_source_files_properties(
        src/mit_controller/wbc_arc_opt.cpp
        PROPERTIES
        COMPILE_FLAGS -fpermissive
)

set(WBC_SOLVER_TARGETS PkgConfig::wbc-solvers-qpoases)
foreach(optional_solver IN ITEMS eiquadprog qpswift proxqp osqp hpipm)
    if(TARGET PkgConfig::wbc-solvers-${optional_solver})
        list(APPEND WBC_SOLVER_TARGETS PkgConfig::wbc-solvers-${optional_solver})
        string(TOUPPER "${optional_solver}" optional_solver_upper)
        target_compile_definitions(
                mitcontrollernode PRIVATE WBC_HAS_${optional_solver_upper})
    endif()
endforeach()

target_link_libraries(mitcontrollernode
        PkgConfig::wbc-core
        PkgConfig::wbc-robot_models-pinocchio
        ${WBC_SOLVER_TARGETS}
        PkgConfig::wbc-scenes-acceleration_reduced_tsid
        PkgConfig::wbc-scenes-acceleration_tsid
        PkgConfig::wbc-controllers
        PkgConfig::wbc-types
)

if (ROBOT_NAME STREQUAL "ulab")
    target_compile_definitions(mitcontrollernode PUBLIC ROBOT_MODEL=ULAB)
    MESSAGE(WARNING "Building for ulab quad")
elseif (ROBOT_NAME STREQUAL "go2")
    target_compile_definitions(mitcontrollernode PUBLIC ROBOT_MODEL=GO2)
    MESSAGE(WARNING "Building for go2 quad")
else ()
    MESSAGE(FATAL_ERROR "Please specify robot model during build")
endif ()

target_compile_options(mitcontrollernode PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_ros_dependencies(mitcontrollernode Eigen3 common interfaces rclcpp)
target_link_libraries(mitcontrollernode acados fmt::fmt)
target_include_directories(
        mitcontrollernode
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


# ##############################################################################
# --- test Swing leg Controller --- #
# ##############################################################################

add_executable(testSwingTrajectoryControllerNode src/testSwingTrajectoryControllerNode.cpp
        src/SwingTrajectory.cpp)

target_ros_dependencies(testSwingTrajectoryControllerNode Eigen3 interfaces rclcpp ament_index_cpp
        sensor_msgs geometry_msgs common)

target_include_directories(
        testSwingTrajectoryControllerNode
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# ##############################################################################
# --- Sys ID Leg Controller --- #
# ##############################################################################

add_executable(SysIDLegControllerNode src/system_identification/SysIDLegControllerNode.cpp src/system_identification/SysIDTrajectory.cpp)

target_ros_dependencies(SysIDLegControllerNode Eigen3 interfaces rclcpp common)
target_link_libraries(SysIDLegControllerNode common::quad_model_symbolic)

target_include_directories(
        SysIDLegControllerNode
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


# ##############################################################################
# --- CPU power consumption logging--- #
# ##############################################################################
add_executable(log_cpu_power src/tools/log_cpu_power.cpp)

target_ros_dependencies(log_cpu_power
        rclcpp
        std_msgs
)

target_include_directories(
        log_cpu_power
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


# ##############################################################################
# Install nodes #
# ##############################################################################

##############################
# Python package and scripts #
##############################

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/joy_to_target.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/quad_stand_up.py DESTINATION lib/${PROJECT_NAME})

#########
# Share #
#########

if (NOT WITHOUT_DRAKE)
    install(TARGETS trajectory_replay gait_plotting gait_sequence_plotting log_cpu_power testSwingTrajectoryControllerNode SysIDLegControllerNode mitcontrollernode DESTINATION lib/${PROJECT_NAME})
else ()
    install(TARGETS gait_plotting gait_sequence_plotting log_cpu_power testSwingTrajectoryControllerNode SysIDLegControllerNode mitcontrollernode DESTINATION lib/${PROJECT_NAME})
endif ()

install(DIRECTORY launch config trajectories DESTINATION share/${PROJECT_NAME}/)

ament_package()
