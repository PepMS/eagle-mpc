message(STATUS "Python3_LIBRARIES = ${Python3_LIBRARIES}")
message(STATUS "Python3_EXECUTABLE = ${Python3_EXECUTABLE}")
message(STATUS "Python3_INCLUDE_DIRS = ${Python3_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")

set(${PROJECT_NAME}_PYTHON_BINDINGS_SOURCES python/multicopter_mpc/multicopter-mpc.cpp)
set(${PROJECT_NAME}_PYTHON_BINDINGS_FILES __init__.py)

add_library(${PROJECT_NAME}_pywrap SHARED ${${PROJECT_NAME}_PYTHON_BINDINGS_SOURCES})

target_include_directories(${PROJECT_NAME}_pywrap PUBLIC  ${Boost_INCLUDE_DIRS} 
                                                          ${Python3_INCLUDE_DIRS}
                                                          ${EIGEN3_INCLUDE_DIRS}
                                                          ${CMAKE_SOURCE_DIR}/bindings
)

target_link_libraries(${PROJECT_NAME}_pywrap ${Boost_LIBRARIES} 
                                             ${Python3_LIBRARIES}
                                             ${YAML_PARSER_LIB}
                                             multicopter_mpc
                                             ${eigenpy_LIBRARIES})

target_compile_options(${PROJECT_NAME}_pywrap PRIVATE "-Wno-conversion")

install(TARGETS ${PROJECT_NAME}_pywrap
        LIBRARY DESTINATION lib/python3/dist-packages/multicopter_mpc)
install(DIRECTORY python/multicopter_mpc/utils DESTINATION lib/python3/dist-packages/multicopter_mpc FILES_MATCHING PATTERN "*.py")
install(FILES python/multicopter_mpc/__init__.py DESTINATION lib/python3/dist-packages/multicopter_mpc)
# install(FILES ${MULTICOPTER_MPC_CONFIG_DIR}/path.py DESTINATION lib/python3/dist-packages/multicopter_mpc/utils)