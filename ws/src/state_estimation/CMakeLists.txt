cmake_minimum_required(VERSION 3.10)
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()
project(state_estimation)

find_program(ROS_SYSTEM_PYTHON3 python3 PATHS /usr/bin /bin NO_DEFAULT_PATH)
if (ROS_SYSTEM_PYTHON3)
    set(Python3_EXECUTABLE "${ROS_SYSTEM_PYTHON3}" CACHE FILEPATH "Python interpreter used by ament_cmake" FORCE)
endif ()

file(GLOB ROS_PYTHON_SITE_PACKAGES "/opt/ros/*/lib/python*/site-packages")
foreach (ROS_PYTHON_SITE_PACKAGE IN LISTS ROS_PYTHON_SITE_PACKAGES)
    if (EXISTS "${ROS_PYTHON_SITE_PACKAGE}/ament_package")
        if (DEFINED ENV{PYTHONPATH})
            set(ENV{PYTHONPATH} "${ROS_PYTHON_SITE_PACKAGE}:$ENV{PYTHONPATH}")
        else ()
            set(ENV{PYTHONPATH} "${ROS_PYTHON_SITE_PACKAGE}")
        endif ()
    endif ()
endforeach ()


set(CMAKE_BUILD_TYPE Release)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_STANDARD 17)
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

option(WITH_VICON "Use Vicon" ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(pinocchio REQUIRED)
find_package(common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(inekf)
if (WITH_VICON)
    find_package(vicon_receiver REQUIRED)
endif ()
# Add the executable for the C++ node
add_executable(state_estimation src/state_estimation_node.cpp src/contact_detection.cpp src/kalman_filter.cpp)

target_link_libraries(
    state_estimation
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs
    interfaces::interfaces
    inekf::inekf
    std_srvs::std_srvs
)
if (WITH_VICON)
    target_link_libraries(state_estimation ${vicon_receiver_TARGETS})
    target_compile_definitions(state_estimation PUBLIC WITH_VICON)
endif ()

target_link_libraries(state_estimation common::quad_model_pino)

target_include_directories(
        state_estimation
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

install(TARGETS state_estimation DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_package()
