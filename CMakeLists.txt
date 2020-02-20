cmake_minimum_required(VERSION 3.1)
project(usrl_vo)

#IF(NOT CMAKE_BUILD_TYPE)
#    SET(CMAKE_BUILD_TYPE Release)
#ENDIF()

#MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -O3 -fopenmp -pthread")

#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# OpenCV
find_package(OpenCV 3.1 REQUIRED)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 REQUIRED)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
endif()
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
#find_package(Eigen3 3.1.0 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#if(Eigen3_FOUND)
#    message("./h dir ${EIGEN3_INCLUDE_DIR}")
#endif()
include_directories("/usr/include/eigen3")

# Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# G2O
#find_package(G2O REQUIRED)
#include_directories(${G2O_INCLUDE_DIRS})
#LIST(APPEND LINK_G2O_LIBS 
#    ${G2O_CORE_LIBRARY}
#    ${G2O_STUFF_LIBRARY}
#    ${G2O_SOLVER_CSPARSE}
#    ${G2O_TYPES_SBA}
#)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
    src/VisualOdometry.cpp
    src/FrontEnd.cpp
    src/Converter.cpp
#    src/Camera.cpp
    src/Frame.cpp
    src/Feature.cpp
    src/MapPoint.cpp
    src/Map.cpp
    src/BackEnd.cpp
    src/Viewer.cpp
)

set(THIRD_PARTY_LIBS
    ${OpenCV_LIBS}
#    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${Sophus_LIBRARIES}
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
#    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_core.so 
#    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_stuff.so
#    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_types_data.so
#    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_solver_csparse.so
#    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o_csparse_extension.so
)

target_link_libraries(${PROJECT_NAME}
    ${THIRD_PARTY_LIBS}
    ${LINK_G2O_LIBS}
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/examples/stereo)

add_executable(stereo_kitti
    examples/stereo/stereo_kitti.cpp
)

target_link_libraries(stereo_kitti ${PROJECT_NAME})


