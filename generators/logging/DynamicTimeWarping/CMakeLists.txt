cmake_minimum_required(VERSION 2.8.9)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "-m64")
SET(CMAKE_C_FLAGS "-m64")
SET(CMAKE_EXE_LINKER_FLAGS "-m64")
SET(CMAKE_MODULE_LINKER_FLAGS "-m64")
SET(CMAKE_SHARED_LINKER_FLAGS "-m64")
project (comparePaths)

#Depends on OMPL and MRPT
# find_package(OMPL)
find_package(MRPT) 
include_directories( /usr/local/include/ )
include_directories( ${MRPT_INCLUDE_DIRS} )
include_directories( src/include )
file(GLOB SOURCES "src/*.cpp")
add_library(comparePaths SHARED ${SOURCES})
target_link_libraries(comparePaths /usr/local/lib/libompl.so ${MRPT_LIBRARIES})

#Use "sudo make install" to apply
install(TARGETS comparePaths DESTINATION /usr/local/lib)
