cmake_minimum_required(VERSION 2.8.3)
project(projector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  image_transport
  pcl_ros
  roscpp
  tf2_ros
)

find_package(OpenCV REQUIRED core imgproc highgui)

catkin_python_setup()
catkin_package()

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(image_screen
  src/image_screen/image_screen.cpp
  )

add_executable(cloud_gateway
  src/cloud_gateway/cloud_node.cpp
  src/cloud_gateway/cloud_gateway.cpp
  )



## Specify libraries to link a library or executable target against
target_link_libraries(image_screen
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(cloud_gateway
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS  image_screen cloud_gateway
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/projector/nodes/game.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} )



