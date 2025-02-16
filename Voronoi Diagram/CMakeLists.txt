cmake_minimum_required(VERSION 3.10)

# Set the project name
project(VoronoiDiagram)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable
add_executable(testAVL src/AVLTree/testAVL.cpp)

# Include directories
include_directories(src/AVLTree)

# Link libraries (if any)
# target_link_libraries(VoronoiDiagram <library_name>)