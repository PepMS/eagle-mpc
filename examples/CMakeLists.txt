set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin/examples)

set(INCLUDE_EXAMPLES
../include
${EIGEN3_INCLUDE_DIRS}
${CROCODDYL_INCLUDE_DIRS}
)

set(LIBS_EXAMPLES
multicopter_mpc
${Boost_SYSTEM_LIBRARY}
${CROCODDYL_LIB}
${URDFDOM_LIB}
${YAMLCPP_LIBRARY}
${PINOCCHIO_LIB}
)

file(GLOB APP_SOURCES ./*.cpp)
message("EXAMPLES BUILD")
foreach(sourcefile ${APP_SOURCES})
  get_filename_component(execname ${sourcefile} NAME_WE)
  message("-- Example: " ${execname})
  add_executable(${execname} ${sourcefile})
  target_include_directories(${execname} PUBLIC ${INCLUDE_EXAMPLES})
  target_link_libraries(${execname} PUBLIC ${LIBS_EXAMPLES})
endforeach(sourcefile ${APP_SOURCES})
