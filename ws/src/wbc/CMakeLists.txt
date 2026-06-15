cmake_minimum_required (VERSION 3.16.3)
project(wbc)

# The nested upstream install tree is incompatible with ament's symlink
# install override. Install this package using standard CMake copy semantics.
set(AMENT_CMAKE_SYMLINK_INSTALL OFF CACHE BOOL "" FORCE)

find_package(ament_cmake REQUIRED)
find_package(qpOASES CONFIG REQUIRED)

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
option(WBC_BUILD_TUTORIALS "Build WBC tutorials" OFF)

# qpOASES is installed by its workspace package as a non-PIC static archive.
# Build a private PIC copy so it can be linked into WBC's shared solver.
set(QPOASES_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../deps/qpOASES")
if(NOT EXISTS "${QPOASES_SOURCE_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "qpOASES source not found at ${QPOASES_SOURCE_DIR}")
endif()
file(GLOB QPOASES_SOURCES CONFIGURE_DEPENDS
     "${QPOASES_SOURCE_DIR}/src/*.cpp")
add_library(qpOASES_pic STATIC ${QPOASES_SOURCES})
set_target_properties(qpOASES_pic PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/qpoases_pic_lib")
target_compile_definitions(qpOASES_pic PRIVATE __NO_COPYRIGHT__ LINUX)
target_include_directories(qpOASES_pic PUBLIC "${QPOASES_SOURCE_DIR}/include")
install(TARGETS qpOASES_pic ARCHIVE DESTINATION lib)

# WBC consumes qpOASES through pkg-config, while the workspace dependency only
# exports CMake metadata. Generate adapters for this build and the install tree.
set(QPOASES_PKGCONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/pkgconfig")
file(MAKE_DIRECTORY "${QPOASES_PKGCONFIG_DIR}")
set(QPOASES_PC_LIBDIR "${CMAKE_CURRENT_BINARY_DIR}/qpoases_pic_lib")
configure_file(cmake/qpOASES.pc.in
    "${QPOASES_PKGCONFIG_DIR}/qpOASES.pc" @ONLY)
set(ENV{PKG_CONFIG_PATH} "${QPOASES_PKGCONFIG_DIR}:$ENV{PKG_CONFIG_PATH}")

set(QPOASES_INSTALL_PKGCONFIG_DIR
    "${CMAKE_CURRENT_BINARY_DIR}/install-pkgconfig")
file(MAKE_DIRECTORY "${QPOASES_INSTALL_PKGCONFIG_DIR}")
set(QPOASES_PC_LIBDIR "${CMAKE_INSTALL_PREFIX}/lib")
configure_file(cmake/qpOASES.pc.in
    "${QPOASES_INSTALL_PKGCONFIG_DIR}/qpOASES.pc" @ONLY)
install(FILES "${QPOASES_INSTALL_PKGCONFIG_DIR}/qpOASES.pc"
    DESTINATION lib/pkgconfig)

add_subdirectory(src)
add_dependencies(wbc-solvers-qpoases qpOASES_pic)

if(WBC_BUILD_TUTORIALS)
    add_subdirectory(tutorials)
endif()

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

ament_environment_hooks(env-hooks/pkg_config_path.dsv)
ament_package()
