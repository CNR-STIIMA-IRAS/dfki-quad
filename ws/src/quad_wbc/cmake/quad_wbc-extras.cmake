find_package(PkgConfig REQUIRED)

function(quad_wbc_pkg_search module required_flag)
  if(NOT TARGET PkgConfig::${module})
    pkg_search_module(${module} ${required_flag} IMPORTED_TARGET ${module})
  endif()
endfunction()

quad_wbc_pkg_search(wbc-core REQUIRED)
quad_wbc_pkg_search(wbc-robot_models-pinocchio REQUIRED)
quad_wbc_pkg_search(wbc-solvers-qpoases REQUIRED)
quad_wbc_pkg_search(wbc-scenes-acceleration_reduced_tsid REQUIRED)
quad_wbc_pkg_search(wbc-scenes-acceleration_tsid REQUIRED)
quad_wbc_pkg_search(wbc-controllers REQUIRED)
quad_wbc_pkg_search(wbc-types REQUIRED)

foreach(optional_solver IN ITEMS eiquadprog qpswift proxqp osqp hpipm)
  quad_wbc_pkg_search(wbc-solvers-${optional_solver} QUIET)
endforeach()
