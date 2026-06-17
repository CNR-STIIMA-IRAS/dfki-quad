cmake_minimum_required(VERSION 3.10)
project(drivers)

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


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
option(ON_PI "On Raspberry Pi" FALSE)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED IMPORTED_TARGET libserial)

include_directories(include/drivers)

function(target_ros_dependencies target)
    foreach(dependency IN LISTS ARGN)
        if(dependency STREQUAL "Eigen3")
            target_link_libraries(${target} Eigen3::Eigen)
        elseif(dependency STREQUAL "ament_index_cpp")
            target_link_libraries(${target} ament_index_cpp::ament_index_cpp)
        elseif(dependency STREQUAL "common")
            target_link_libraries(${target} common::quad_model_pino)
        elseif(dependency STREQUAL "geometry_msgs")
            target_link_libraries(${target}
                    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp)
        elseif(dependency STREQUAL "interfaces")
            target_link_libraries(${target}
                    interfaces::interfaces__rosidl_typesupport_cpp)
        elseif(dependency STREQUAL "rclcpp")
            target_link_libraries(${target} rclcpp::rclcpp)
        elseif(dependency STREQUAL "sensor_msgs")
            target_link_libraries(${target}
                    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp)
        elseif(dependency STREQUAL "std_msgs")
            target_link_libraries(${target}
                    std_msgs::std_msgs__rosidl_typesupport_cpp)
        elseif(dependency STREQUAL "std_srvs")
            target_link_libraries(${target}
                    std_srvs::std_srvs__rosidl_typesupport_cpp)
        elseif(dependency STREQUAL "unitree_api")
            target_link_libraries(${target}
                    unitree_api::unitree_api__rosidl_typesupport_cpp)
        elseif(dependency STREQUAL "unitree_go")
            target_link_libraries(${target}
                    unitree_go::unitree_go__rosidl_typesupport_cpp)
        elseif(TARGET ${dependency}::${dependency})
            target_link_libraries(${target} ${dependency}::${dependency})
        elseif(TARGET ${dependency})
            target_link_libraries(${target} ${dependency})
        else()
            message(FATAL_ERROR "Unsupported dependency ${dependency}")
        endif()
    endforeach()
endfunction()

add_executable(leg_driver src/leg_driver.cpp)
target_ros_dependencies(leg_driver rclcpp interfaces Eigen3 std_srvs ament_index_cpp)
target_link_libraries(leg_driver common::quad_model_pino)
install(TARGETS leg_driver DESTINATION lib/${PROJECT_NAME})

add_executable(contact_sensor_driver src/contact_sensor_driver.cpp)
target_ros_dependencies(contact_sensor_driver rclcpp interfaces)
target_link_libraries(contact_sensor_driver PkgConfig::LIBSERIAL common::quad_model_pino)
install(TARGETS contact_sensor_driver DESTINATION lib/${PROJECT_NAME})

# Install executables
#install(TARGETS test_abstract_motor_driver DESTINATION lib/${PROJECT_NAME})
# install(TARGETS mjbots_ros2_motor_driver DESTINATION lib/${PROJECT_NAME})

# Install launch and config directories install(DIRECTORY launch DESTINATION
# share/${PROJECT_NAME}/)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

ament_package()
