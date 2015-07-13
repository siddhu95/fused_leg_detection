cmake_minimum_required(VERSION 2.8.4)
PROJECT(dlib)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  message_filters
  image_geometry
  visualization_msgs
  tf
)

find_package(OpenCV)
find_package(Boost REQUIRED)
include(include/dlib/cmake)
#include(/home/miningrox/Documents/dlib-18.16/dlib/cmake)

catkin_package()

include_directories(include /home/miningrox/Documents/dlib-18.16 ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
ADD_EXECUTABLE(video_depth_split src/examples/video_depth_split.cpp)
TARGET_LINK_LIBRARIES(video_depth_split dlib ${OpenCV_LIBS} ${catkin_LIBRARIES})

ADD_EXECUTABLE(RGBD_hog src/RGBD_hog.cpp)
TARGET_LINK_LIBRARIES(RGBD_hog dlib ${OpenCV_LIBS} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

ADD_EXECUTABLE(image_on_3d src/image_on_3d.cpp)
TARGET_LINK_LIBRARIES(image_on_3d dlib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})






