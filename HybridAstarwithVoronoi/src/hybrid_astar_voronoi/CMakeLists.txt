cmake_minimum_required(VERSION 3.5)
project(hybrid_astar_voronoi)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../ros2support.cmake)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

FIND_PACKAGES(
  ament_cmake
  rclcpp
  ament_index_cpp
  geometry_msgs
  nav_msgs
  tf2
  trajectory_msgs
  Eigen3
  OpenCV
)

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/include/hybrid_astar_voronoi
)

file(GLOB HAV_SOURCES "src/hybrid_astar_voronoi/*.cpp")
BUILD_EXEC(
  TARGET hybrid_astar_voronoi
  SOURCES ${HAV_SOURCES} testHav.cpp
  DEPENDENCIES rclcpp ament_index_cpp geometry_msgs nav_msgs tf2 trajectory_msgs Eigen3 OpenCV
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
