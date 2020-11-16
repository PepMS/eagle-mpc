set(${PROJECT_NAME}_PYTHON_TESTS
multicopter_base_params  
waypoint
)

FOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
  add_test(NAME "py-${TEST}" 
           COMMAND ${Python3_EXECUTABLE} "${PROJECT_SOURCE_DIR}/unittest/python/test_${TEST}.py")  
ENDFOREACH(TEST ${${PROJECT_NAME}_PYTHON_TESTS})
