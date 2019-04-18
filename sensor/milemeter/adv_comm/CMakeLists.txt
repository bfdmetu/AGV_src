cmake_minimum_required(VERSION 2.8.3)
project(adv_comm)

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  geometry_msgs
  tf
#  lamp_msgs
#  message_generation
  visualization_msgs
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
CATKIN_DEPENDS message_runtime
)


include_directories(
  ${RPLIDAR_SDK_PATH}/include
  ${RPLIDAR_SDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
)



add_executable(rplidartoADV src/rplidar_client.cpp)
target_link_libraries(rplidartoADV ${catkin_LIBRARIES})

add_executable(Sutpc_ros_odom src/Sutpc_ros_odom.cpp)
target_link_libraries(Sutpc_ros_odom ${catkin_LIBRARIES})


install(TARGETS rplidartoADV
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
