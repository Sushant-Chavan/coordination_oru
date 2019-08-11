cmake_minimum_required(VERSION 3.5.1)

# Set Project name
project (Generate_Graphml)

# Enable C++ 11
set (CMAKE_CXX_STANDARD 11)

find_package(Boost COMPONENTS system filesystem serialization REQUIRED)
# find_package(OMPL)
find_package(MRPT) 

# Define a variable PROJECT_LINK_LIBS to contain all library dependencies
set ( PROJECT_LINK_LIBS ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_SERIALIZATION_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} )

# Define the paths to the libraries
link_directories(/usr/local/lib/)

# Define the paths to the include directories
include_directories(include/
 /usr/include/eigen3/
 /usr/local/include/
 )

 include_directories( ${MRPT_INCLUDE_DIRS} )

# Define the name of the executable to be build using the source files
add_executable(GenerateGraphml
 src/generate_graphml.cpp
 )

# Link the executable to the defined libraries
target_link_libraries(GenerateGraphml ${PROJECT_LINK_LIBS} /usr/local/lib/libompl.so ${MRPT_LIBRARIES})
