ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK) 

add_subdirectory(python)

set(INCLUDE_UNITTEST
../include
)

set(LIBS_UNITTEST
multicopter_mpc
${Boost_FILESYSTEM_LIBRARY}
${Boost_SYSTEM_LIBRARY}
${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
)

file(GLOB APP_SOURCES ./*.cpp)
message("UNITTEST BUILD")
foreach(sourcefile ${APP_SOURCES})
  get_filename_component(testname ${sourcefile} NAME_WE)
  message("-- Unittest: " ${testname})
  add_executable(${testname} ${sourcefile})
  target_include_directories(${testname} PUBLIC ${INCLUDE_UNITTEST})
  target_link_libraries(${testname} PUBLIC ${LIBS_UNITTEST})
  set_target_properties(${testname} PROPERTIES 
                        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build/unittest)
  add_test(NAME ${testname} WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/build/unittest 
           COMMAND ${CMAKE_SOURCE_DIR}/build/unittest/${testname})
endforeach(sourcefile ${APP_SOURCES})