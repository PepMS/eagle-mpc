cmake_minimum_required(VERSION 3.8) # to enable debugging
SET(CMAKE_CONFIGURATION_TYPES ${CMAKE_BUILD_TYPE} CACHE STRING "" FORCE)
PROJECT(multicopter_mpc VERSION 0.0
                        DESCRIPTION "Model predictive Control & Optimal control library for MultiCopter-based robots"
                        LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR -DBOOST_MPL_LIMIT_LIST_SIZE=30 -DPINOCCHIO_WITH_URDFDOM")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/yaml_parser/cmake") 
set(CMAKE_INSTALL_PREFIX /usr/local)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(LIBRARY_DESTINATION "lib")
set(INCLUDE_DESTINATION "include")
set(SHARE_DESTINATION "share/multicopter_mpc")

enable_testing()

option(WITH_BINDINGS "Compile bindings" ON)
option(WITH_EXAMPLES "Compile examples" OFF)
option(WITH_BENCHMARK "Compile benchmarks" OFF)
option(WITH_UNITTEST "Compile unittest" OFF)

# Testing purposes
# set(MULTICOPTER_MPC_CONFIG_DIR ${PROJECT_BINARY_DIR}/other/multicopter_mpc)
# set(MULTICOPTER_MPC_ROOT_DIR ${CMAKE_SOURCE_DIR})
# set(MULTICOPTER_MPC_MULTIROTOR_DIR ${CMAKE_INSTALL_PREFIX}/share/multicopter_mpc/multicopter)

# configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/other/path.h "${MULTICOPTER_MPC_CONFIG_DIR}/path.h")
# configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config/other/path.py.in ${MULTICOPTER_MPC_CONFIG_DIR}/path.py)
# message("CONFIG ${MULTICOPTER_MPC_CONFIG_DIR}/config.h")
# message("CONFIG DIRECTORY ${PROJECT_BINARY_DIR}")
# include_directories("${PROJECT_BINARY_DIR}/other")

find_package(Eigen3 3.3 REQUIRED)
find_package(eigenpy REQUIRED)
find_package(pinocchio REQUIRED)
find_package(crocoddyl REQUIRED)

set(Python_ADDITIONAL_VERSIONS 3.6 3.7 3.8)
find_package(Python3 REQUIRED Development Interpreter)
set(WHICH_PYTHON3 "python3${Python3_VERSION_MINOR}")
find_package(Boost COMPONENTS ${WHICH_PYTHON3} filesystem unit_test_framework REQUIRED)

SET(YAML_PARSER_LIB /usr/local/lib/libyaml_parser.so)
SET(YAML_PARSER_INCLUDE_DIRS /usr/local/include/yaml_parser/)
SET(URDFDOM_LIB /usr/lib/x86_64-linux-gnu/liburdfdom_model.so)

message(STATUS "Eigen library:" ${EIGEN3_INCLUDE_DIRS})


add_subdirectory(src) # MultiCopter-MPC Library
if (WITH_EXAMPLES)
  add_subdirectory(examples)
endif ()
if (WITH_BINDINGS)
  add_subdirectory(bindings) 
endif ()
if (WITH_BENCHMARK)
  add_subdirectory(benchmark)
endif ()
if (WITH_UNITTEST)
  add_subdirectory(unittest)
endif ()