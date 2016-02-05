cmake_minimum_required(VERSION 2.8)

project(mobilefusion)

SET(MY_DIR ${mobilefusion_SOURCE_DIR})

# guard against in-source build
IF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    MESSAGE(FATAL_ERROR "In-source builds not allowed.")
ENDIF()

# set output directory
SET(LIBRARY_OUTPUT_PATH ${MY_DIR}/lib)

option(BUILD_TEST "Build unit test of modules" OFF)

# cmake modules
list(APPEND CMAKE_MODULE_PATH ${MY_DIR})

# dependencies
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(freenect2 REQUIRED)
find_package(CPUTSDF REQUIRED)
find_package(X11 REQUIRED)

SET(INCLUDE_DIRS
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${freenect2_INCLUDE_DIR}
    ${CPUTSDF_INCLUDE_DIR}
    ${X11_INCLUDE_DIR}
    ${MY_DIR}/include
)

SET(LIBRARIES
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${freenect2_LIBRARIES}
    ${CPUTSDF_LIBRARIES}
    ${X11_LIBRARIES}
)

SET(SOURCES
    src/CloudCompareRenderer.cpp
    src/CloudNormalProvider.cpp
    src/CloudProvider.cpp
    src/CloudRegister.cpp
    src/CloudRenderer.cpp
    src/CloudRecorder.cpp
    src/FusionManager.cpp
    src/Kinect.cpp
    src/KinectPngReader.cpp
    src/KinectRecorder.cpp
    src/KinectRenderer.cpp
    src/Octree.cpp
    src/OctreeNode.cpp
    src/TsdfVolumeOctree.cpp
    src/TSDFVolumeWrapper.cpp
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${INCLUDE_DIRS})

# added for gdb debugging
SET(CMAKE_CXX_FLAGS_DEBUG "-O0")
SET(CMAKE_C_FLAGS_DEBUG "-O0")

# executables
add_executable(main main.cpp ${SOURCES})
target_link_libraries(main ${LIBRARIES})

if(BUILD_TEST)
    add_executable(icp_test icp_test.cpp ${SOURCES})
    target_link_libraries(icp_test ${LIBRARIES})
endif(BUILD_TEST)
