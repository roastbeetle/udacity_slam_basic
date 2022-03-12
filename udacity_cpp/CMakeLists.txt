cmake_minimum_required( VERSION 2.8 )
project ( udacity )

set(CMAKE_BUILD_TYPE Release)
set( CMAKE_CXX_COMPILER "g++" )
set(CMAKE_CXX_STANDARD 14)
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )


find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
)

include_directories( include )
add_subdirectory( src )