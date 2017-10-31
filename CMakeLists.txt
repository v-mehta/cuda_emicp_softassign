cmake_minimum_required(VERSION 3.5)

project(emicp)
add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# PCL and VTK fixes
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(VTK REQUIRED)

get_directory_property(dir_defs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS)
set(vtk_flags)
foreach(it ${dir_defs})
  if(it MATCHES "vtk*")
  list(APPEND vtk_flags ${it})
  endif()
endforeach()

foreach(d ${vtk_flags})
  remove_definitions(-D${d})
endforeach()

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/" ${CMAKE_MODULE_PATH})
include(FindFlann)

# OpenMP
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -fPIC")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fPIC")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Boost
find_package(Boost 1.54 REQUIRED)

# Atlas
find_library(ATLAS atlas)

# Lapack
include(FindLAPACK)
add_definitions ( ${LAPACK_LINKER_FLAGS} )

# CUDA
find_package(CUDA REQUIRED)

include_directories(
  include/emicp
  ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc/
  ${CUDA_INCLUDE_DIRS}
)

set(CUDA_ARCH "-arch=sm_35" CACHE STRING "Value of the NVCC -arch option.")
set(CUDA_NVCC_FLAGS "-arch=sm_35" CACHE STRING "nvcc flags" FORCE)
set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
message("CUDA_TOOLKIT_ROOT_DIR: " ${CUDA_TOOLKIT_ROOT_DIR} )

set(CUDASRCS src/emicp.cu)
CUDA_COMPILE(cuda_objs ${CUDASRCS})

cuda_add_library(${PROJECT_NAME}
	src/findRTfromS.cpp
	src/cloud2data.cpp
  src/emicp.cu
  ${cuda_objs}
)

target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES} ${CUDA_CUBLAS_LIBRARIES})

cuda_add_executable(example src/main.cpp src/emicp.cu)
target_link_libraries(example
  ${PROJECT_NAME}
  ${CUDA_LIBRARIES}
  ${CUDA_nppi_LIBRARY}
  ${PCL_LIBRARIES}
  ${LAPACK_LIBRARIES}
  ${CUDA_CUBLAS_LIBRARIES}
)

add_executable(test_icp
  src/test_icp.cpp
)

target_link_libraries(test_icp
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
)

set_target_properties(test_icp PROPERTIES COMPILE_OPTIONS "-std=c++11")
