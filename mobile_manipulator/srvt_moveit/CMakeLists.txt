cmake_minimum_required(VERSION 2.8.3)
project(srvt_moveit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sir_robot_ros_interface
  message_generation
)

add_service_files(
    FILES 
    TaskService.srv
    ImageService.srv
)

 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

foreach(dir launch srv src)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
