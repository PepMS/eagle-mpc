set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

file(GLOB_RECURSE EAGLE_MPC_SOURCES *.cpp)

set(EAGLE_MPC_INCLUDE_DIRS    
    ${PINOCCHIO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CROCODDYL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)

set(EAGLE_MPC_LIBS
    yaml-cpp
    crocoddyl::crocoddyl
    pinocchio::pinocchio
)

configure_file(${eagle_mpc_SOURCE_DIR}/config/path.hpp.in ${CMAKE_BINARY_DIR}/include/eagle_mpc/path.hpp)

add_library(eagle_mpc SHARED ${EAGLE_MPC_SOURCES})
target_include_directories(eagle_mpc
    PUBLIC
        "$<BUILD_INTERFACE:${eagle_mpc_SOURCE_DIR}/include;${EAGLE_MPC_INCLUDE_DIRS};${CMAKE_BINARY_DIR}/include>" # quotes to indicate several directories
        $<INSTALL_INTERFACE:${INCLUDE_DESTINATION}>
)

target_link_libraries(eagle_mpc PUBLIC ${EAGLE_MPC_LIBS})

target_compile_features(eagle_mpc PUBLIC cxx_std_14)

# Install
install(DIRECTORY ../include/eagle_mpc 
        DESTINATION ${INCLUDE_DESTINATION})
install(FILES ${CMAKE_BINARY_DIR}/include/eagle_mpc/path.hpp
        DESTINATION ${INCLUDE_DESTINATION}/eagle_mpc)
include(GNUInstallDirs)
install(TARGETS eagle_mpc
        EXPORT eagle_mpc-targets 
        LIBRARY DESTINATION ${LIBRARY_DESTINATION}
        )
install(EXPORT eagle_mpc-targets
        FILE eagle_mpc-targets.cmake
        NAMESPACE eagle_mpc::
        DESTINATION ${LIBRARY_DESTINATION}/cmake/eagle_mpc
        )
install(FILES ${eagle_mpc_SOURCE_DIR}/cmake/eagle_mpc-config.cmake 
        DESTINATION ${LIBRARY_DESTINATION}/cmake/eagle_mpc)