message(STATUS "Python3_LIBRARIES = ${Python3_LIBRARIES}")
message(STATUS "Python3_EXECUTABLE = ${Python3_EXECUTABLE}")
message(STATUS "Python3_INCLUDE_DIRS = ${Python3_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES = ${Boost_LIBRARIES}")

set(${PROJECT_NAME}_PYTHON_BINDINGS_SOURCES python/eagle_mpc/eagle-mpc.cpp)
set(${PROJECT_NAME}_PYTHON_BINDINGS_FILES __init__.py)

add_library(${PROJECT_NAME}_pywrap SHARED ${${PROJECT_NAME}_PYTHON_BINDINGS_SOURCES})

target_include_directories(${PROJECT_NAME}_pywrap PUBLIC  ${Boost_INCLUDE_DIRS} 
                                                          ${Python3_INCLUDE_DIRS}
                                                          ${EIGEN3_INCLUDE_DIRS}
                                                          ${CMAKE_SOURCE_DIR}/bindings
)

target_link_libraries(${PROJECT_NAME}_pywrap ${Boost_LIBRARIES} 
                                             ${Python3_LIBRARIES}
                                             eagle_mpc
                                             ${eigenpy_LIBRARIES})

target_compile_options(${PROJECT_NAME}_pywrap PRIVATE "-Wno-conversion")

install(TARGETS ${PROJECT_NAME}_pywrap
        LIBRARY DESTINATION lib/python3/dist-packages/eagle_mpc)
install(DIRECTORY python/eagle_mpc/utils DESTINATION lib/python3/dist-packages/eagle_mpc FILES_MATCHING PATTERN "*.py")
install(FILES python/eagle_mpc/__init__.py DESTINATION lib/python3/dist-packages/eagle_mpc)