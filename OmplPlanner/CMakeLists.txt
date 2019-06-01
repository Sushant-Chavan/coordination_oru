cmake_minimum_required(VERSION 2.8.9)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "-m64")
SET(CMAKE_C_FLAGS "-m64")
SET(CMAKE_EXE_LINKER_FLAGS "-m64")
SET(CMAKE_MODULE_LINKER_FLAGS "-m64")
SET(CMAKE_SHARED_LINKER_FLAGS "-m64")
project (omplMotionPlanner)

#Depends on OMPL and MRPT
# find_package(OMPL)
find_package(MRPT) 
find_package(smpl REQUIRED)
find_package(smpl_ompl_interface REQUIRED)
message( STATUS ${SMPL_INCLUDE_DIRS})
include_directories( /usr/local/include/ )
include_directories( ${MRPT_INCLUDE_DIRS} )
include_directories( src/include )
file(GLOB SOURCES "src/*.cpp")
add_library(omplMotionPlanner SHARED ${SOURCES})
target_link_libraries(omplMotionPlanner /usr/local/lib/libompl.so /usr/local/lib/libsmpl.so /usr/local/lib/libsmpl_ompl_interface.so ${MRPT_LIBRARIES})

#Use "sudo make install" to apply
install(TARGETS omplMotionPlanner DESTINATION /usr/local/lib)
