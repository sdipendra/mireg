cmake_minimum_required(VERSION 2.8.12.2)

project(mireg_program)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(THRUST_DEVICE_SYSTEM THRUST_DEVICE_SYSTEM_OMP)	#later edit not sure whether required

include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(include)
add_subdirectory(res)
add_subdirectory(src)
