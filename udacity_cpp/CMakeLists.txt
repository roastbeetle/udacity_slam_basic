cmake_minimum_required( VERSION 2.8 )
project ( udacity )

set(CMAKE_BUILD_TYPE Release)
set( CMAKE_CXX_COMPILER "g++" )
set(CMAKE_CXX_STANDARD 14)

include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )