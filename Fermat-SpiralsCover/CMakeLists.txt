cmake_minimum_required(VERSION 3.10)

# Set the project name
project(FermatSpiralsCover VERSION 1.0)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Add the executable
add_executable(FermatSpiralsCover main.cpp sc_define.cpp sc_plan.cpp)

# Include directories
# include_directories(${PROJECT_SOURCE_DIR}/include)

# Link libraries (if any)
# target_link_libraries(FermatSpiralsCover <library_name>)
target_link_libraries(FermatSpiralsCover ${OpenCV_LIBS} Eigen3::Eigen)