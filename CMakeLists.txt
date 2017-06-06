cmake_minimum_required(VERSION 3.1.0)
project(jason_test)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  tf)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 3 REQUIRED)

include_directories(
  # include
	${catkin_INCLUDE_DIRS} 
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs cv_bridge
  DEPENDS EIGEN3 PCL OpenCV
  # INCLUDE_DIRS include
)
if(CMAKE_COMPILER_IS_GNUCXX)
  add_definitions(-std=gnu++0x)
endif()

add_executable(rot_test src/rot_test.cpp)
target_link_libraries(rot_test ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(plane_seg src/plane_seg.cpp)
target_link_libraries(plane_seg ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
