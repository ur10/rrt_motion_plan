cmake_minimum_required(VERSION 3.0.2)
project(rrt_motion_plan)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp 
  INCLUDE_DIRS include
)

##add_library(rrt ${PROJECT_SOURCE_DIR}/src/rrt.cpp)
add_executable(rrt ${PROJECT_SOURCE_DIR}/src/rrt.cpp ${PROJECT_SOURCE_DIR}/src/main.cpp  ${HEADER_FILES})
target_link_libraries(rrt ${LIBS} ${catkin_LIBRARIES})