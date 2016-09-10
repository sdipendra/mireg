# Mireg
This project is mainly focused on registering 3D Laser Scans using Mutual Information and GPU parallelization of computations

1. Current program uses 3D Laser Scan point cloud data and corresponding Camera data for registration of 2 Laser scans.
2. The key assumption of the current method is that a dominant ground plane is present in the laser scans, which is the case for laser scanners mounted on top of cars and other vehicles.
3. Sample scans are provided with the code to test the algorithm and code which have been extracted and obtained from fords dataset.(http://robots.engin.umich.edu/uploads/SoftwareData/Ford/dataset-1.tar.gz)


To build the project you will need to have following libraries and tools installed on your system:
1. CMake
2. Nvidia Cuda
3. Boost 1.61.0 or higher
4. flann
5. gsl-2.1 or higher
6. opencv-3.1.0 or higher
7. pcl 1.8.0 or higher
8. VTK 7.0.0 or higher

To build the project:

    cd TO_PROJECT_DIRECTORY
    mkdir build
    cd build
    cmake ..
    make
    cd bin
    ./mireg file_name1 file_name2 {all, variance, reflectivity, grayscale, normal}

this will output the euler transformation to be carried out on the first(reading) scan to align with the second(reference) scan. Transformation matrix can also be obtained by tweaking the code slightly as I am converting transformation matrix only to euler transformations.

This is very crude information. I will improve it on request and on my own as I get time. The code also needs to be organised well.

Current code is only a serial implementation. My first target will be to organise the code properly so others can understand it. After that I will work on improving the serial implementation by using more effecient routines.

Once, all that is done I will design the parallel version of the program and work on it's implementation.

## Most importantly please feel free to contribute, offer suggestions and crticize the current work.
