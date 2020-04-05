set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

set(MULTICOPTER_MPC_SOURCES
    multicopter-base-params.cpp
    waypoint.cpp
    mission.cpp
)

set(MULTICOPTER_MPC_HEADERS
    ../include/multicopter-base-params.hpp
    ../include/waypoint.hpp
    ../include/mission.hpp
)

set(MULTICOPTER_MPC_INCLUDE_DIRS
    ../include
    ${EIGEN3_INCLUDE_DIRS}
)

set(MULTICOPTER_MPC_LIBS
${YAML_PARSER_LIB}
)

add_library(multicopter_mpc SHARED ${MULTICOPTER_MPC_SOURCES})
target_include_directories(multicopter_mpc PUBLIC ${MULTICOPTER_MPC_INCLUDE_DIRS})
target_link_libraries(multicopter_mpc PUBLIC ${MULTICOPTER_MPC_LIBS})

# Install
install(TARGETS multicopter_mpc
        LIBRARY DESTINATION lib)
install(DIRECTORY ../include/multicopter_mpc DESTINATION include)
