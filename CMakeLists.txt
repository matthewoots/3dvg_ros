cmake_minimum_required(VERSION 3.0.2)
project(visibility_graph)

# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

## Find catkin macros and libraries
find_package(catkin 
  COMPONENTS
    roscpp
    common_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

catkin_package(
  CATKIN_DEPENDS  
    roscpp 
    common_msgs

  DEPENDS
    Eigen3
)

include_directories(
    include
    VisiLibity1/src
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node 
    src/main.cpp
    src/visibility.cpp
    VisiLibity1/src/visilibity.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)