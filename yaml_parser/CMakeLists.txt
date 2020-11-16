cmake_minimum_required(VERSION 3.1)

PROJECT(yaml_parser VERSION 0.0
                    DESCRIPTION "General purpose YAML parser"
                    LANGUAGES CXX)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D..")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_INSTALL_PREFIX /usr/local)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

find_package(YamlCpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
set(Python_ADDITIONAL_VERSIONS 3.6 3.7 3.8)
find_package(Python3 REQUIRED Development Interpreter)
set(WHICH_PYTHON3 "python3${Python3_VERSION_MINOR}")
find_package(Boost COMPONENTS ${WHICH_PYTHON3} filesystem unit_test_framework REQUIRED)
message(STATUS "Eigen library:" ${EIGEN3_INCLUDE_DIRS})

set(SOURCES
src/converter_utils.cpp
src/parser_yaml.cpp
)

set(INCLUDE_DIRS
include
${EIGEN3_INCLUDE_DIRS}
# /usr/local/include/
)

set(HEADERS
include/yaml_parser/converter_utils.h
include/yaml_parser/converter.h
include/yaml_parser/params_server.hpp
include/yaml_parser/parser_yaml.h
)

set(LIBS 
${YAMLCPP_LIBRARY}
)

# Build
add_library(yaml_parser SHARED ${SOURCES})
target_include_directories(yaml_parser PUBLIC ${INCLUDE_DIRS})
target_link_libraries(yaml_parser PUBLIC ${LIBS})

# Install
install(TARGETS yaml_parser
        LIBRARY DESTINATION lib)
install(DIRECTORY include/yaml_parser DESTINATION include)
# install(FILES "cmake/FindYamlCpp.cmake" DESTINATION "lib/cmake/YamlCpp")
# install(FILES "cmake/FindYamlParser.cmake" DESTINATION "lib/cmake/${PROJECT_NAME}")

add_subdirectory(bindings) 
