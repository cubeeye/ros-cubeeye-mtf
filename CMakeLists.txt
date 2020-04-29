cmake_minimum_required(VERSION 2.8.3)
project(cubeeye_mtf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

 generate_dynamic_reconfigure_options(
   cfg/mtf_depth.cfg
 )

#catkin_package(
#  INCLUDE_DIRS include
#   CATKIN_DEPENDS roscpp sensor_msgs rospy std_msgs dynamic_reconfigure
#  DEPENDS system_lib
#)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
)

add_executable(${PROJECT_NAME}_node src/DepthMTFNode.cpp src/DepthMTFReader.cpp)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${CMAKE_CURRENT_SOURCE_DIR}/lib/libMTF_API.so
)

