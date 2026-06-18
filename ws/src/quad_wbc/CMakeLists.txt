cmake_minimum_required(VERSION 3.20)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
project(quad_wbc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(fmt REQUIRED)
find_package(common REQUIRED)

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

add_library(${PROJECT_NAME} SHARED
  src/quad_wbc/inverse_dynamics.cpp
  src/quad_wbc/wbc_arc_opt.cpp
)

# Library (this will be used as the base class for plugins)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC c_std_17 cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  common::quad_model_pino
  PkgConfig::wbc-core
  PkgConfig::wbc-robot_models-pinocchio
  PkgConfig::wbc-solvers-qpoases
  PkgConfig::wbc-scenes-acceleration_reduced_tsid
  PkgConfig::wbc-scenes-acceleration_tsid
  PkgConfig::wbc-controllers
  PkgConfig::wbc-types
)
target_link_libraries(${PROJECT_NAME} PRIVATE fmt::fmt-header-only)

foreach(optional_solver IN ITEMS eiquadprog qpswift proxqp osqp hpipm)
  if(TARGET PkgConfig::wbc-solvers-${optional_solver})
    target_link_libraries(${PROJECT_NAME} PUBLIC PkgConfig::wbc-solvers-${optional_solver})
    string(TOUPPER "${optional_solver}" optional_solver_upper)
    target_compile_definitions(${PROJECT_NAME} PUBLIC WBC_HAS_${optional_solver_upper})
  endif()
endforeach()

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install library and export targets
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # Keep package tests runnable in offline development containers.
  set(ament_cmake_xmllint_FOUND TRUE)
  # Existing sources do not follow the ROS uncrustify profile yet.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_inverse_dynamics test/test_inverse_dynamics.cpp)
  if(TARGET test_inverse_dynamics)
    target_link_libraries(test_inverse_dynamics ${PROJECT_NAME})
  endif()
endif()

# Export old-style CMake variables
ament_export_include_directories(
  include
)

# Export modern CMake targets
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(Eigen3 common)

ament_package(CONFIG_EXTRAS cmake/quad_wbc-extras.cmake)
