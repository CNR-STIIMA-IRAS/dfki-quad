cmake_minimum_required(VERSION 3.10)
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()
project(controllers)

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


option(ON_PI "On Raspberry Pi" FALSE)
if(ON_PI)
    message(WARNING "Skipping package on Raspberry Pi.")
    return()
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)

option(USE_WBC ON)

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
find_package(quad_mpc REQUIRED)
find_package(quad_wbc REQUIRED)
find_package(quad_model_adaptation REQUIRED)
find_package(quad_swing_controller REQUIRED)

find_package(OpenMP) # for speeding up eigens matrix multiplications
if(NOT WITHOUT_DRAKE)
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
endif()
find_package(acados REQUIRED)
find_package(Matplot++ REQUIRED)
find_package(unitree_go REQUIRED)
find_package(gait_controller REQUIRED)
find_package(pluginlib REQUIRED)

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
        elseif(dependency STREQUAL "gait_controller")
            target_link_libraries(${target} gait_controller::gait_controller)
        else()
            message(FATAL_ERROR "Unsupported dependency ${dependency}")
        endif()
    endforeach()
endfunction()


include_directories(include include/mit_controller src)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights comment the
    # line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo) comment the line
    # when this package is in a git repo and when a copyright and license is added
    # to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()


# ##############################################################################
# --- MIT CONTROLLER NODE --- #
# ##############################################################################
add_executable(mitcontrollernode
    src/mit_controller_node.cpp
)

if(USE_WBC)
    target_compile_definitions(
        mitcontrollernode PRIVATE USE_WBC=1)
elseif(USE_WBC)
    target_compile_definitions(
        mitcontrollernode PRIVATE USE_WBC=0)
endif()

target_link_libraries(mitcontrollernode
    gait_controller::gait_controller
    quad_mpc::quad_mpc
    pluginlib::pluginlib
    quad_wbc::quad_wbc
    quad_model_adaptation::quad_model_adaptation
    quad_swing_controller::quad_swing_controller
)

target_compile_options(mitcontrollernode PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_ros_dependencies(mitcontrollernode Eigen3 common interfaces rclcpp)
target_link_libraries(mitcontrollernode acados)
target_include_directories(
    mitcontrollernode
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)



#########
# Share #
#########

install(TARGETS mitcontrollernode DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_package()
