cmake_minimum_required(VERSION 2.8)

project(mobilefusion)

SET(MY_DIR ${mobilefusion_SOURCE_DIR})

# guard against in-source build
IF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    MESSAGE(FATAL_ERROR "In-source builds not allowed.")
ENDIF()

# set output directory
SET(LIBRARY_OUTPUT_PATH ${MY_DIR}/lib)

# cmake modules
list(APPEND CMAKE_MODULE_PATH ${MY_DIR})

# dependencies
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(freenect2 REQUIRED)
find_package(CPUTSDF REQUIRED)

SET(INCLUDE_DIRS
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${CPUTSDF_INCLUDE_DIR}
    ${freenect2_INCLUDE_DIR}
    ${MY_DIR}/include
)

SET(LIBRARIES
    ${OpenCV_LIBRARIES}
    ${PCL_LIBRARIES}
    ${CPUTSDF_LIBRARIES}
    ${freenect2_LIBRARIES}
)

SET(SOURCES
    src/CloudCompareRenderer.cpp
    src/CloudNormalProvider.cpp
    src/CloudProvider.cpp
    src/CloudRegister.cpp
    src/CloudRenderer.cpp
    src/CpuTsdf.cpp
    src/FusionManager.cpp
    src/Kinect.cpp
    src/KinectRecorder.cpp
    src/KinectRenderer.cpp
    src/Octree.cpp
    src/TsdfVolumeOctree.cpp
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${INCLUDE_DIRS})

# executables
add_executable(main main.cpp ${SOURCES})
target_link_libraries(main ${LIBRARIES})
message(STATUS ${OpenCV_INCLUDE_DIRS})
message(STATUS ${OpenCV_LIBRARIES})
