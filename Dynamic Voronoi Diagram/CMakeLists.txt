cmake_minimum_required(VERSION 3.10)

# Set the project name
project(DynamicVoronoiDiagram)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable
add_executable(DynamicVoronoiDiagram main.cpp)

# Include directories
include_directories(include)

# Link libraries (if any)
# target_link_libraries(DynamicVoronoiDiagram <library_name>)