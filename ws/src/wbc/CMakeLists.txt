cmake_minimum_required (VERSION 3.16.3)
if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()
project(wbc)

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


# The nested upstream install tree is incompatible with ament's symlink
# install override. Install this package using standard CMake copy semantics.
set(AMENT_CMAKE_SYMLINK_INSTALL OFF CACHE BOOL "" FORCE)

find_package(ament_cmake REQUIRED)
find_package(qpOASES REQUIRED)

find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem serialization)
if(WBC_BUILD_TESTING)
    find_package(Boost REQUIRED COMPONENTS unit_test_framework)
endif()
if(NOT TARGET Boost::system)
    add_library(Boost::system INTERFACE IMPORTED)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(PROJECT_VERSION 0.2)
set(API_VERSION ${PROJECT_VERSION})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

option(ROBOT_MODEL_RBDL "Also build the RBDL-based robot model, by default only pinocchio is built" OFF)
option(ROBOT_MODEL_HYRODYN "Also build the HyRoDyn-based robot model, by default only pinocchio is built" OFF)
option(SOLVER_PROXQP "Build the ProxQP-based solver, by default qpoases is built" OFF)
option(SOLVER_EIQUADPROG "Build the Eiquadprog-based solver, by default only qpoases is built" OFF)
option(SOLVER_QPSWIFT "Build the QPSwift-based solver, by default only qpoases is built" OFF)
option(SOLVER_OSQP "Build the OSQP-based solver, by default only qpoases is built" OFF)
option(SOLVER_HPIPM "Build the HPIPM solver, by default only qpoases is built" OFF)
option(WBC_BUILD_TESTING "Build WBC unit tests" OFF)

add_subdirectory(src)


find_package(Doxygen)

if(DOXYGEN_FOUND)
    set(doxyfile_in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)
    set(doxyfile ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    configure_file(${doxyfile_in} ${doxyfile} @ONLY)
    message("Doxygen build started.")
    add_custom_target(doc
                      COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile_in}
                      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                      COMMENT "Generating API documentation with Doxygen"
                      VERBATIM)
else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)

ament_package()
