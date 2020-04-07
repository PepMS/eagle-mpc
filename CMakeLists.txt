cmake_minimum_required(VERSION 3.1)

PROJECT(multicopter_mpc VERSION 0.0
                        DESCRIPTION "Model predictive Control & Optimal control library for MultiCopter-based robots"
                        LANGUAGES CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR -DBOOST_MPL_LIMIT_LIST_SIZE=30 -DPINOCCHIO_WITH_URDFDOM")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/yaml_parser/cmake") 
set(CMAKE_INSTALL_PREFIX /usr/local)
set(CMAKE_BUILD_TYPE RELEASE)

find_package(eigenpy REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(YamlCpp REQUIRED)
find_package(Boost COMPONENTS python)
find_package(PythonInterp)
find_package(PythonLibs REQUIRED)

SET(YAML_PARSER_LIB /usr/local/lib/libyaml_parser.so)
SET(YAML_PARSER_INCLUDE_DIRS /usr/local/include/yaml_parser/)
SET(CROCODDYL_LIB /usr/local/lib/libcrocoddyl.so)
SET(CROCODDYL_INCLUDE_DIRS /usr/local/include/crocoddyl)
SET(PINOCCHIO_LIB /usr/local/lib/libpinocchio.so)
SET(PINOCCHIO_INCLUDE_DIRS /usr/local/include/pinocchio)
SET(URDFDOM_LIB /usr/lib/x86_64-linux-gnu/liburdfdom_model.so)

message(STATUS "Eigen library:" ${EIGEN3_INCLUDE_DIRS})

add_subdirectory(src) # MultiCopter-MPC Library
add_subdirectory(examples)
add_subdirectory(bindings) 
