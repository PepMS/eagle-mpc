set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/benchmark)

set(INCLUDE_BENCHMARK
../include
${EIGEN3_INCLUDE_DIRS}
${CROCODDYL_INCLUDE_DIRS}
)

set(LIBS_BENCHMARK
multicopter_mpc
-lboost_system
${CROCODDYL_LIB}
${URDFDOM_LIB}
${YAMLCPP_LIBRARY}
${PINOCCHIO_LIB}
)

file(GLOB APP_SOURCES ./*.cpp)
message("BENCHMARK BUILD")
foreach(sourcefile ${APP_SOURCES})
  get_filename_component(execname ${sourcefile} NAME_WE)
  message("-- Benchmark: " ${execname})
  add_executable(${execname} ${sourcefile})
  target_include_directories(${execname} PUBLIC ${INCLUDE_BENCHMARK})
  target_link_libraries(${execname} PUBLIC ${LIBS_BENCHMARK})
endforeach(sourcefile ${APP_SOURCES})
message("BENCHMARK BUILD")
