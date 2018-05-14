cmake_minimum_required(VERSION 2.8)

include(cmake/base.cmake)

set(PROJECT_NAME mc_cable_regrasp_controller)
set(PROJECT_DESCRIPTION "Cable grasp control points adjustment.")
set(PROJECT_URL "https://gite.lirmm.fr/me/mc_cable_regrasp_controller")

set(CMAKE_CXX_VISIBILITY_PRESET hidden)

setup_project()

#Enable C++11
if(NOT WIN32)
  add_definitions(-std=c++0x)
endif()

set(PKG_CONFIG_ADDITIONAL_VARIABLES "${PKG_CONFIG_ADDITIONAL_VARIABLES} libdir")
add_required_dependency(mc_rtc)

add_subdirectory(src)
setup_project_finalize()

