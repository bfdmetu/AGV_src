cmake_minimum_required(VERSION 2.8.3)
project(data_collection)

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

#FIND_PACKAGE(PCL 1.5 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  geometry_msgs
  tf
#  lamp_msgs
#  message_generation
  visualization_msgs
#  pcl_ros
#  pcl_conversions
)

add_message_files(
  FILES
#  lamp_msgs.msg
)

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
#CATKIN_DEPENDS message_runtime
)


include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

add_executable(data_collection src/data_collection.cpp)
target_link_libraries(data_collection ${catkin_LIBRARIES})

add_executable(GPS_data_collect src/GPS_data_collect.cpp)
target_link_libraries(GPS_data_collect ${catkin_LIBRARIES})


install(TARGETS data_collection
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
