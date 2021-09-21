get_filename_component(eagle_mpc_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)

find_dependency(Python3 REQUIRED Development Interpreter)
set(WHICH_PYTHON3 "python3${Python3_VERSION_MINOR}")
find_dependency(Boost REQUIRED COMPONENTS filesystem unit_test_framework ${WHICH_PYTHON3})
find_dependency(Eigen3 3.3 REQUIRED)
find_dependency(eigenpy REQUIRED)
find_dependency(pinocchio REQUIRED)
find_dependency(crocoddyl REQUIRED)
find_dependency(yaml-cpp REQUIRED)

if(NOT TARGET eagle_mpc::eagle_mpc)
    include("${eagle_mpc_CMAKE_DIR}/eagle_mpc-targets.cmake")
endif()

set(eagle_mpc_INCLUDE_DIRS "/usr/local/include")
set(EAGLE_MPC_INCLUDE_DIRS "/usr/local/include")

set(eagle_mpc_LIBRARIES ${_PACKAGE_CONFIG_LIBRARIES})
set(EAGLE_MPC_LIBRARIES ${_PACKAGE_CONFIG_LIBRARIES})