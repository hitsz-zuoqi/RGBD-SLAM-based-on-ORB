cmake_minimum_required(VERSION 2.8)
project(lecture2)


SET( CMAKE_BUILD_TYPE "Release"  )
SET( CMAKE_CXX_FLAGS "-std=c++11 -Wall -O3 " )
# set(CMAKE_BUILD_TYPE "Debug")
# set(CMAKE_CXX_FLAGS "-O0 -Wall -g -ggdb ")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
List(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(G2O REQUIRED)
find_package(CSparse REQUIRED)
find_package(OpenCV 3.4 REQUIRED)
find_package(PCL REQUIRED common io visualization filters)


add_definitions(${PCL_DEFINITIONS})


include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}/pcl-1.9
    "/usr/local/include/vtk-8.2"
    ${G2O_INCLUDE_DIR}
    ${CSPARSE_INCLUDE_DIR}
)

add_subdirectory(src)
add_subdirectory(test)