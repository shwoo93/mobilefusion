cmake_minimum_required(VERSION 2.8)

project(mobilefusion)

SET(MY_DIR ${mobilefusion_SOURCE_DIR})

# guard against in-source build
IF(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    MESSAGE(FATAL_ERROR "In-source builds not allowed.")
ENDIF()

# C++ 11
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
    MESSAGE(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
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
    src/Kinect.cpp
    src/CameraInterface.cpp
<<<<<<< HEAD
    src/util3d.cpp
    src/util3d_registration.cpp
=======
>>>>>>> d90abbe570863f5858036b9a744f61a42d295191
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${INCLUDE_DIRS})

# executables
add_executable(main main.cpp ${SOURCES})
target_link_libraries(main ${LIBRARIES})
message(STATUS ${OpenCV_INCLUDE_DIRS})
message(STATUS ${OpenCV_LIBRARIES})
