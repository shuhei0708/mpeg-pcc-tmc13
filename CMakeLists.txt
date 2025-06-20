cmake_minimum_required(VERSION 3.0)
project (TMC3 CXX)

include( CheckCXXCompilerFlag )
CHECK_CXX_COMPILER_FLAG( "-std=c++11" COMPILER_SUPPORTS_CXX11 )
CHECK_CXX_COMPILER_FLAG( "-std=c++0x" COMPILER_SUPPORTS_CXX0X )

if (COMPILER_SUPPORTS_CXX11)
 set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
elseif (COMPILER_SUPPORTS_CXX0X)
 set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x" )
else ()
 MESSAGE(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

# Toplevel library is tmc3
add_subdirectory(tmc3)

# Add the sample_api executable
add_executable(sample_api sample_api.cpp)
target_link_libraries(sample_api PRIVATE tmc3_lib)
