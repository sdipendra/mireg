find_package(CUDA REQUIRED)
list(APPEND CUDA_NVCC_FLAGS "-arch=sm_30;-std=c++11")
find_package(OpenCV REQUIRED)

set(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11 -O2 -g -pg")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")

#cuda_add_executable(mireg mireg.cu)
add_executable(mireg_serial mireg.cpp)
add_executable(display display.cpp)
add_executable(transformer transformer.cpp)

add_subdirectory(visualizer)
add_subdirectory(helper)

target_link_libraries(mireg_serial project_classes pcl_visualizer pcl_helper transformation registration ${OpenCV_LIBS})
target_link_libraries(display project_classes pcl_visualizer ${OpenCV_LIBS})
target_link_libraries(transformer project_classes)
#target_link_libraries(mireg project_classes pcl_visualizer pcl_helper transformation registration ${OpenCV_LIBS})

