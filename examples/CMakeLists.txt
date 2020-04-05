set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/bin/)

# PROBLEM_CROCODDYL
set(SOURCES_PROBLEM_CROCODDYL
problem_crocoddyl.cpp)

set(INCLUDE_PROBLEM_CROCODDYL
../include
${EIGEN3_INCLUDE_DIRS}
${CROCODDYL_INCLUDE_DIRS}
)

set(LIBS_PROBLEM_CROCODDYL
multicopter_mpc
-lboost_system
${CROCODDYL_LIB}
${URDFDOM_LIB}
${YAMLCPP_LIBRARY}
)

add_executable(problem_crocoddyl ${SOURCES_PROBLEM_CROCODDYL})
target_include_directories(problem_crocoddyl PUBLIC ${INCLUDE_PROBLEM_CROCODDYL})
target_link_libraries(problem_crocoddyl PUBLIC ${LIBS_PROBLEM_CROCODDYL})

# PROBLEM_MULTICOPTER_MPC
set(SOURCES_PROBLEM_MULTICOPTER_MPC
problem_multicopter_mpc.cpp)

set(INCLUDE_PROBLEM_MULTICOPTER_MPC
../include
${EIGEN3_INCLUDE_DIRS}
${CROCODDYL_INCLUDE_DIRS}
)

set(LIBS_PROBLEM_MULTICOPTER_MPC
multicopter_mpc
-lboost_system
${CROCODDYL_LIB}
${URDFDOM_LIB}
${YAMLCPP_LIBRARY}
)

add_executable(problem_multicopter_mpc ${SOURCES_PROBLEM_MULTICOPTER_MPC})
target_include_directories(problem_multicopter_mpc PUBLIC ${INCLUDE_PROBLEM_CRCODDYL})
target_link_libraries(problem_multicopter_mpc PUBLIC ${LIBS_PROBLEM_MULTICOPTER_MPC})
