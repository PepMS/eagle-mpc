set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

file(GLOB_RECURSE MULTICOPTER_MPC_SOURCES *.cpp)
file(GLOB_RECURSE MULTICOPTER_MPC_HEADERS ../include/multicopter_mpc/*.hpp)

set(MULTICOPTER_MPC_INCLUDE_DIRS
    ../include
    ${EIGEN3_INCLUDE_DIRS}
    ${CROCODDYL_INCLUDE_DIRS}
)

set(MULTICOPTER_MPC_LIBS
${YAMLCPP_LIBRARY}
${YAML_PARSER_LIB}
${CROCODDYL_LIB}
${PINOCCHIO_LIB}
)

add_library(multicopter_mpc SHARED ${MULTICOPTER_MPC_SOURCES})
target_include_directories(multicopter_mpc PUBLIC ${MULTICOPTER_MPC_INCLUDE_DIRS})
target_link_libraries(multicopter_mpc PUBLIC ${MULTICOPTER_MPC_LIBS})

# Install
install(TARGETS multicopter_mpc
        LIBRARY DESTINATION lib)
install(DIRECTORY ../include/multicopter_mpc DESTINATION include)
install(DIRECTORY ../config/mission DESTINATION share/multicopter_mpc)
install(DIRECTORY ../config/multirotor DESTINATION share/multicopter_mpc)