add_compile_options(-std=c++11)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library (pcl_visualizer pcl_visualizer.cpp)
target_link_libraries (pcl_visualizer ${PCL_LIBRARIES})
