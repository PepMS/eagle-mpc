cmake_minimum_required(VERSION 3.8) # to enable debugging
SET(CMAKE_CONFIGURATION_TYPES ${CMAKE_BUILD_TYPE} CACHE STRING "" FORCE)
PROJECT(eagle_mpc VERSION 0.0
                  DESCRIPTION "Model predictive Control & Optimal control library for MultiCopter-based robots"
                  LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR -DBOOST_MPL_LIMIT_LIST_SIZE=30 -DPINOCCHIO_WITH_URDFDOM")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_INSTALL_PREFIX /usr/local)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(LIBRARY_DESTINATION "lib")
set(INCLUDE_DESTINATION "include")
set(SHARE_DESTINATION "share/eagle_mpc")

enable_testing()

option(WITH_BINDINGS "Compile bindings" ON)
option(WITH_EXAMPLES "Compile examples" OFF)
option(WITH_BENCHMARK "Compile benchmarks" OFF)

find_package(Eigen3 3.3 REQUIRED)
find_package(eigenpy REQUIRED)
find_package(pinocchio REQUIRED)
find_package(crocoddyl REQUIRED)
find_package(yaml-cpp REQUIRED)

set(Python_ADDITIONAL_VERSIONS 3.6 3.7 3.8)
find_package(Python3 REQUIRED Development Interpreter)
set(WHICH_PYTHON3 "python3${Python3_VERSION_MINOR}")
find_package(Boost COMPONENTS ${WHICH_PYTHON3} filesystem unit_test_framework REQUIRED)

SET(URDFDOM_LIB /usr/lib/x86_64-linux-gnu/liburdfdom_model.so)

message(STATUS "Eigen library:" ${EIGEN3_INCLUDE_DIRS})

add_subdirectory(src)
if (WITH_EXAMPLES)
  add_subdirectory(examples)
endif ()
if (WITH_BINDINGS)
  add_subdirectory(bindings) 
endif ()
if (WITH_BENCHMARK)
  add_subdirectory(benchmark)
endif ()