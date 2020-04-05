set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/examples/bin/)

# PROBLEM_CROCODDYL
set(SOURCES_PROBLEM_CROCODDYL
problem_crocoddyl.cpp)

set(INCLUDE_PROBLEM_CRCODDYL
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
target_include_directories(problem_crocoddyl PUBLIC ${INCLUDE_PROBLEM_CRCODDYL})
target_link_libraries(problem_crocoddyl PUBLIC ${LIBS_PROBLEM_CROCODDYL})
