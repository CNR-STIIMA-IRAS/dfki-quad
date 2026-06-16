cmake_minimum_required(VERSION 3.8)
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()
project(common)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_STANDARD 17)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(interfaces REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pinocchio REQUIRED)

if (ON_PI)
    set(WITHOUT_DRAKE ON)
endif ()

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
endif ()
include_directories(include/common)
if (NOT WITHOUT_DRAKE)
    add_library(quad_model src/quad_model.cpp src/quad_state.cpp)
endif ()
add_library(quad_model_symbolic src/quad_state.cpp src/quad_model_symbolic.cpp)
add_library(quad_model_pino src/quad_model_pino.cpp src/quad_state.cpp)
if (ROBOT_NAME STREQUAL "ulab")
    target_compile_definitions(quad_model_symbolic PUBLIC ROBOT_MODEL=ULAB)
    MESSAGE(NOTICE "Building for ulab quad")
elseif (ROBOT_NAME STREQUAL "go2")
    target_compile_definitions(quad_model_symbolic PUBLIC ROBOT_MODEL=GO2)
    MESSAGE(NOTICE "Building for go2 quad")
else ()
    MESSAGE(WARNING "No default constructor!")
endif ()
if (NOT WITHOUT_DRAKE)
    target_link_libraries(quad_model PUBLIC
            drake::drake
            Eigen3::Eigen
            interfaces::interfaces__rosidl_generator_cpp
            rclcpp::rclcpp)
endif ()
target_link_libraries(quad_model_symbolic PUBLIC
        Eigen3::Eigen
        interfaces::interfaces__rosidl_generator_cpp
        rclcpp::rclcpp)
target_link_libraries(quad_model_pino PUBLIC
        Eigen3::Eigen
        ament_index_cpp::ament_index_cpp
        interfaces::interfaces__rosidl_generator_cpp
        pinocchio::pinocchio
        rclcpp::rclcpp)

target_include_directories(
        quad_model_symbolic
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_include_directories(
        quad_model_pino
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)


if (NOT WITHOUT_DRAKE)
    target_include_directories(
            quad_model_symbolic
            PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
endif ()
if (NOT WITHOUT_DRAKE)
    ament_export_targets(quad_model HAS_LIBRARY_TARGET)
endif ()
ament_export_targets(quad_model_symbolic HAS_LIBRARY_TARGET)
ament_export_targets(quad_model_pino HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 ament_index_cpp interfaces pinocchio rclcpp)

install(DIRECTORY include/common DESTINATION include)

if (NOT WITHOUT_DRAKE)
    install(
            TARGETS quad_model
            EXPORT quad_model
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            RUNTIME DESTINATION bin
            INCLUDES
            DESTINATION include)
endif ()

install(
        TARGETS quad_model_symbolic
        EXPORT quad_model_symbolic
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES
        DESTINATION include)

install(
        TARGETS quad_model_pino
        EXPORT quad_model_pino
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES
        DESTINATION include)

if (NOT WITHOUT_DRAKE)
    add_executable(model_test src/tests/model_test.cpp)

    target_link_libraries(model_test PRIVATE
            drake::drake
            Eigen3::Eigen
            interfaces::interfaces__rosidl_generator_cpp
            quad_model_pino
            quad_model_symbolic
            rclcpp::rclcpp)
    target_include_directories(
            model_test
            PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )
endif ()
##############################
# Python package and scripts #
##############################

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/plot_trajectory_bagfile.py DESTINATION lib/${PROJECT_NAME})

#########
# Share #
#########

install(DIRECTORY include model launch config DESTINATION share/${PROJECT_NAME})

ament_package()
