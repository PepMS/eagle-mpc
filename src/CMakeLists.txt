set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

file(GLOB_RECURSE EAGLE_MPC_SOURCES *.cpp)
file(GLOB_RECURSE EAGLE_MPC_HEADERS ${eagle_mpc_SOURCE_DIR}/include/eagle_mpc/*.hpp)

set(EAGLE_MPC_INCLUDE_DIRS
    ${PINOCCHIO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CROCODDYL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

set(EAGLE_MPC_LIBS
    ${YAML_CPP_LIBRARIES}
    ${CROCODDYL_LIBRARIES}
    ${PINOCCHIO_LIBRARIES}
)

add_library(eagle_mpc SHARED ${EAGLE_MPC_SOURCES})
target_include_directories(eagle_mpc PUBLIC
                       $<BUILD_INTERFACE:${eagle_mpc_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:${INCLUDE_DESTINATION}> 
                       ${EAGLE_MPC_INCLUDE_DIRS})
target_link_libraries(eagle_mpc PUBLIC ${EAGLE_MPC_LIBS})

# Install
install(TARGETS eagle_mpc
        EXPORT eagle_mpc DESTINATION ${LIBRARY_DESTINATION})
install(FILES ${eagle_mpc_SOURCE_DIR}/cmake/eagle_mpc-config.cmake DESTINATION ${LIBRARY_DESTINATION}/cmake/eagle_mpc)
install(EXPORT eagle_mpc DESTINATION ${LIBRARY_DESTINATION}/cmake/eagle_mpc)
install(EXPORT eagle_mpc CONFIGURATIONS Release DESTINATION ${LIBRARY_DESTINATION}/cmake/eagle_mpc/Release)
install(DIRECTORY ../include/eagle_mpc DESTINATION ${INCLUDE_DESTINATION})