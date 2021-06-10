get_filename_component(SELF_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(${SELF_DIR}/${CMAKE_BUILD_TYPE}/eagle_mpc.cmake)
set(eagle_mpc_INCLUDE_DIRS "/usr/local/include")