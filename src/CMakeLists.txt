set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

file(GLOB_RECURSE MULTICOPTER_MPC_SOURCES *.cpp)
file(GLOB_RECURSE MULTICOPTER_MPC_HEADERS ${multicopter_mpc_SOURCE_DIR}/include/multicopter_mpc/*.hpp)

set(MULTICOPTER_MPC_INCLUDE_DIRS
    ${PINOCCHIO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CROCODDYL_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
)

set(MULTICOPTER_MPC_LIBS
${YAMLCPP_LIBRARY}
${YAML_PARSER_LIB}
${CROCODDYL_LIBRARIES}
${PINOCCHIO_LIBRARIES}
)

add_library(multicopter_mpc SHARED ${MULTICOPTER_MPC_SOURCES})
target_include_directories(multicopter_mpc PUBLIC
                       $<BUILD_INTERFACE:${multicopter_mpc_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:${INCLUDE_DESTINATION}> 
                       ${MULTICOPTER_MPC_INCLUDE_DIRS})
target_link_libraries(multicopter_mpc PUBLIC ${MULTICOPTER_MPC_LIBS})

# Install
install(TARGETS multicopter_mpc
        EXPORT multicopter_mpc DESTINATION ${LIBRARY_DESTINATION})
install(FILES ${multicopter_mpc_SOURCE_DIR}/cmake/multicopter_mpc-config.cmake DESTINATION ${LIBRARY_DESTINATION}/cmake/multicopter_mpc)
install(EXPORT multicopter_mpc DESTINATION ${LIBRARY_DESTINATION}/cmake/multicopter_mpc)
install(EXPORT multicopter_mpc CONFIGURATIONS Release DESTINATION ${LIBRARY_DESTINATION}/cmake/multicopter_mpc/Release)
install(DIRECTORY ../include/multicopter_mpc DESTINATION ${INCLUDE_DESTINATION})
install(FILES ${multicopter_mpc_SOURCE_DIR}/config/other/path.h DESTINATION ${INCLUDE_DESTINATION}/multicopter_mpc)
install(DIRECTORY ../config/mission DESTINATION ${SHARE_DESTINATION})
install(DIRECTORY ../config/multirotor DESTINATION ${SHARE_DESTINATION})
install(DIRECTORY ../config/ocp DESTINATION ${SHARE_DESTINATION})