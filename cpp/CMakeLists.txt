project(NSF)
cmake_minimum_required(VERSION 3.18.2)

SET(CMAKE_BUILD_TYPE Release)
SET(MATLAB_ROOT ${MATLAB_ROOT} CACHE FILEPATH "directory")
SET(ZED_CUDA_VERSION 11.5)

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

add_definitions(/DMATLAB_MEX_FILE)
add_definitions(/DMX_COMPAT_32)

find_package(Matlab REQUIRED)

IF(MATLAB_FOUND)
    message(STATUS "MATLAB Found, MATLAB MEX will be compiled.")
    add_subdirectory(processTextMex)
    add_subdirectory(robotMex)
    add_subdirectory(objLoaderMex)

ELSE(MATLAB_FOUND)
    message( FATAL_ERROR "MATLAB not found...nothing will be built." )
ENDIF(MATLAB_FOUND)

#add_executable(test processTextMex/process_txtCPP.cpp)