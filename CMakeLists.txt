cmake_minimum_required(VERSION 2.8)
set(CMAKE_VERBOSE_MAKEFILE 1) # to see all make process

project(cuda_emicp_softassign)

# Force gcc and g++ to use version 5.4 because nvcc cannot handle any versions higher than 5.4.
set(CMAKE_C_COMPILER "/usr/bin/gcc-5")

SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
SET_PROPERTY(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache) 

# PCL
find_package(PCL 1.8 REQUIRED)
include_directories ( ${PCL_INCLUDE_DIRS} )
link_directories ( ${PCL_LIBRARY_DIRS} )
add_definitions ( ${PCL_DEFINITIONS} )

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})
include(FindFlann)

# OpenMP
FIND_PACKAGE(OpenMP REQUIRED)
if (OPENMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -fPIC")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fPIC")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Boost
find_package(Boost 1.54 REQUIRED)

# CUDA
find_package(CUDA 8.0 REQUIRED)
include_directories ( ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc/ )

message("CUDA_VERSION: " ${CUDA_VERSION} )
message("CUDA_CUDA_LIBRARY: " ${CUDA_CUDA_LIBRARY} )
message("CUDA_CUDART_LIBRARY: " ${CUDA_CUDART_LIBRARY} )
message("CUDA_NVCC_FLAGS: " ${CUDA_NVCC_FLAGS} "-Wno-deprecated-gpu-targets -fPIC")
message("CUDA_INCLUDE_DIRS: " ${CUDA_INCLUDE_DIRS} )
message("CUDA_CUBLAS_LIBRARIES: " ${CUDA_CUBLAS_LIBRARIES} )
message("CUDA_TOOLKIT_ROOT_DIR: " ${CUDA_TOOLKIT_ROOT_DIR} )

find_library(ATLAS atlas)

include(FindLAPACK)
add_definitions ( ${LAPACK_LINKER_FLAGS} )

cuda_add_executable ( ${PROJECT_NAME} src/3dregistration.h src/main.cpp src/findRTfromS.cpp src/cloud2data.cpp src/emicp.cu)
target_link_libraries ( ${PROJECT_NAME} ${PCL_LIBRARIES} ${LAPACK_LIBRARIES} -llapack)

CUDA_ADD_CUBLAS_TO_TARGET ( ${PROJECT_NAME} )
