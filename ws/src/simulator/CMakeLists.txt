cmake_minimum_required(VERSION 3.10)
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()
project(simulator)

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


option(ON_PI "On Raspberry Pi" FALSE)
if (ON_PI)
    message(WARNING "Skipping package on Raspberry Pi.")
    return()
endif ()

if (WITHOUT_DRAKE)
    message(WARNING "Skipping packages because it depends on drake")
    return()
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(CMAKE_BUILD_TYPE Release)
# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(interfaces REQUIRED)
find_package(common REQUIRED)
find_package(sensor_msgs REQUIRED)
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
find_package(rosgraph_msgs CONFIG REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


include_directories(${EIGEN_INCLUDE_DIR})

add_executable(visualizer src/drake_visualizer.cpp)
target_compile_features(visualizer PUBLIC cxx_std_17)
add_executable(simulator src/drake_simulator.cpp src/pd_changeable_controller.cpp)
target_compile_features(simulator PUBLIC cxx_std_17)
add_executable(sim_disturber src/disturbance_node.cpp)
add_library(drake_visualizers STATIC src/drake_vizualisers/positions_vizualiser.cpp
        src/drake_vizualisers/forces_vizualiser.cpp
        src/drake_vizualisers/vectors_vizualiser.cpp
        src/drake_vizualisers/poses_vizualiser.cpp
)

target_link_libraries(visualizer drake_visualizers)

include_directories(include)

target_link_libraries(
        visualizer
        rclcpp::rclcpp
        ament_index_cpp::ament_index_cpp
        interfaces::interfaces__rosidl_typesupport_cpp
        sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
        common::quad_model_pino
        drake::drake
        Eigen3::Eigen)

target_link_libraries(
        drake_visualizers
        drake::drake
        Eigen3::Eigen)

target_link_libraries(
        simulator
        drake::drake
        rclcpp::rclcpp
        ament_index_cpp::ament_index_cpp
        interfaces::interfaces__rosidl_typesupport_cpp
        sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
        rosgraph_msgs::rosgraph_msgs__rosidl_typesupport_cpp
        common::quad_model_pino
        Eigen3::Eigen)

target_link_libraries(
        sim_disturber
        rclcpp::rclcpp
        interfaces::interfaces__rosidl_typesupport_cpp
        common::quad_model_pino
        Eigen3::Eigen)

target_include_directories(
        visualizer PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_include_directories(simulator
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/**>
        $<INSTALL_INTERFACE:include>)

target_include_directories(
        drake_visualizers PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_include_directories(
        sim_disturber PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_export_targets(drake_visualizers HAS_LIBRARY_TARGET)


install(TARGETS visualizer DESTINATION lib/${PROJECT_NAME})
install(TARGETS simulator DESTINATION lib/${PROJECT_NAME})
install(TARGETS sim_disturber DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/drake_vizualizers DESTINATION include)

install(
        TARGETS drake_visualizers
        EXPORT drake_visualizers
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES
        DESTINATION include)

install(DIRECTORY include launch DESTINATION share/${PROJECT_NAME})

ament_package()
