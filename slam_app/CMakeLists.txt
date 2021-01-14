add_subdirectory(lcm_mapping)

if(NOT BUILD_QNX)
add_subdirectory(ros_mapping)
add_subdirectory(python_wrapper)
endif()