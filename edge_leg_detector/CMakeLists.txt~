cmake_minimum_required(VERSION 2.8.3)
project(edge_leg_detector)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  visualization_msgs
  message_filters
  tf
)

find_package(Boost REQUIRED COMPONENTS thread)

###################################
## catkin specific configuration ##
###################################
    
catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs visualization_msgs tf message_filters
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(detector src/edge_leg_detector/detector.cpp)
target_link_libraries(detector
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(marker src/edge_leg_detector/marker.cpp)
target_link_libraries(marker
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(laserCombine src/edge_leg_detector/laserCombine.cpp)
target_link_libraries(laserCombine
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
#add_dependencies(marker edge_leg_detector_generate_messages_cpp)

#############
## Install ##
#############

install(TARGETS detector 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(TARGETS laserCombine
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
