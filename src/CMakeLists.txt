add_compile_options(-std=c++11)
find_package(CUDA REQUIRED)
list(APPEND CUDA_NVCC_FLAGS "-arch=sm_30;-std=c++11")
find_package(OpenCV REQUIRED)

cuda_add_executable(mireg mireg.cu)
add_executable(mireg_serial mireg.cpp)
add_executable(display display.cpp)

add_subdirectory(visualizer)
add_subdirectory(helper)

target_link_libraries(mireg_serial pcl_visualizer pcl_helper transformation registration ${OpenCV_LIBS})
target_link_libraries(display pcl_visualizer ${OpenCV_LIBS})
target_link_libraries(mireg pcl_visualizer pcl_helper transformation registration ${OpenCV_LIBS})

