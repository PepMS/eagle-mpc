set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/bin/)

# PROBLEM_CROCODDYL
set(SOURCES_PROBLEM_CROCODDYL
problem-crocoddyl.cpp)

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
${PINOCCHIO_LIB}
)

add_executable(problem-crocoddyl ${SOURCES_PROBLEM_CROCODDYL})
target_include_directories(problem-crocoddyl PUBLIC ${INCLUDE_PROBLEM_CROCODDYL})
target_link_libraries(problem-crocoddyl PUBLIC ${LIBS_PROBLEM_CROCODDYL})

# PROBLEM_MULTICOPTER_MPC
set(SOURCES_PROBLEM_MULTICOPTER_MPC
problem-multicopter-mpc.cpp)

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
${PINOCCHIO_LIB}
)

add_executable(problem-multicopter-mpc ${SOURCES_PROBLEM_MULTICOPTER_MPC})
target_include_directories(problem-multicopter-mpc PUBLIC ${INCLUDE_PROBLEM_MULTICOPTER_MPC})
target_link_libraries(problem-multicopter-mpc PUBLIC ${LIBS_PROBLEM_MULTICOPTER_MPC})

# PROBLEM_MULTICOPTER_MPC_TEST
set(SOURCES_PROBLEM_MULTICOPTER_TEST
problem-multicopter-test.cpp)

set(INCLUDE_PROBLEM_MULTICOPTER_TEST
../include
${EIGEN3_INCLUDE_DIRS}
${CROCODDYL_INCLUDE_DIRS}
)

set(LIBS_PROBLEM_MULTICOPTER_TEST
multicopter_mpc
-lboost_system
${CROCODDYL_LIB}
${URDFDOM_LIB}
${YAMLCPP_LIBRARY}
${PINOCCHIO_LIB}

)

add_executable(problem-multicopter-test ${SOURCES_PROBLEM_MULTICOPTER_TEST})
target_include_directories(problem-multicopter-test PUBLIC ${INCLUDE_PROBLEM_MULTICOPTER_TEST})
target_link_libraries(problem-multicopter-test PUBLIC ${LIBS_PROBLEM_MULTICOPTER_TEST})

# PROBLEM_MULTICOPTER_MPC_TEST
set(SOURCES_MPC_MAIN
problem-main.cpp)

set(INCLUDE_MPC_MAIN
../include
${EIGEN3_INCLUDE_DIRS}
${CROCODDYL_INCLUDE_DIRS}
)

set(LIBS_MPC_MAIN
multicopter_mpc
-lboost_system
${CROCODDYL_LIB}
${URDFDOM_LIB}
${YAMLCPP_LIBRARY}
${PINOCCHIO_LIB}

)

add_executable(problem-main ${SOURCES_MPC_MAIN})
target_include_directories(problem-main PUBLIC ${INCLUDE_MPC_MAIN})
target_link_libraries(problem-main PUBLIC ${LIBS_MPC_MAIN})
