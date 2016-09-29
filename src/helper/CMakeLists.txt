add_compile_options(-std=c++11)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(project_classes project_classes.cpp)
add_library(pcl_helper pcl_helper.cpp)
add_library(transformation transformation.cpp)
add_library(characteristic_map characteristic_map.cpp)
add_library(mutual_information mutual_information.cpp)
add_library(registration registration.cpp)

target_link_libraries (pcl_helper ${PCL_LIBRARIES})
target_link_libraries (characteristic_map gsl gslcblas project_classes)
target_link_libraries (registration pcl_visualizer transformation characteristic_map mutual_information project_classes)

