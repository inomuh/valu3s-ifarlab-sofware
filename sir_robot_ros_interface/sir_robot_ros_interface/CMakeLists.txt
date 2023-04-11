cmake_minimum_required(VERSION 2.8.3)
project(sir_robot_ros_interface)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  agv_msgs
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  Pose.msg
  Poses.msg
)

add_service_files(
  FILES
  ManipulatorPose.srv
  ManipulatorPose_ino.srv
  ManipulatorPose_ino_2.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
     INCLUDE_DIRS include
     CATKIN_DEPENDS rospy roscpp std_msgs agv_msgs message_runtime
)


include_directories(include Eigen ${catkin_INCLUDE_DIRS})


add_library(sir_robot_ros_interface
  src/SIRLogger.cpp
  src/SIRRobot/KawasakiRS005LRobot.cpp
  src/SIRRobot/SIRConnection.cpp
  src/SIRRobot/SIRLinConnection.cpp
  src/SIRRobot/SIRPacketPackager.cpp
  src/SIRRobot/SIRPacketParser.cpp
  src/SIRRobot/SIRPacketTest.cpp
  src/SIRRobot/SIRPoseDB.cpp
  src/SIRRobot/SIRPoseQueueDB.cpp
  src/SIRRobot/SIRRobot.cpp
  src/SIRRobot/SIRRobotCommands.cpp
  src/SIRRobot/SIRRobotPacketPackager.cpp
  src/SIRRobot/SIRRobotPacketParser.cpp
  src/SIRRobot/SIRWinConnection.cpp
)

add_dependencies(
  ${PROJECT_NAME} 
  DEPS ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

install(
  DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


add_executable(KawasakiRobotTestForLinux tests/KawasakiRobotTestForLinux.cpp)
target_link_libraries(KawasakiRobotTestForLinux ${catkin_LIBRARIES} sir_robot_ros_interface)

add_dependencies(KawasakiRobotTestForLinux ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_dependencies(sir_robot_ros_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(KawasakiRobotTestForLinux_Valu3s tests/KawasakiRobotTestForLinux_Valu3s.cpp)

target_link_libraries(KawasakiRobotTestForLinux_Valu3s ${catkin_LIBRARIES} sir_robot_ros_interface)

add_dependencies(KawasakiRobotTestForLinux_Valu3s ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


# add_executable(KawasakiRobotTestForLinux_Valu3s_ino tests/KawasakiRobotTestForLinux_Valu3s_ino.cpp)
# 
# target_link_libraries(KawasakiRobotTestForLinux_Valu3s_ino ${catkin_LIBRARIES} sir_robot_ros_interface)
# 
# add_dependencies(KawasakiRobotTestForLinux_Valu3s_ino ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


add_executable(KawasakiRobotTestForLinux_Valu3s_ino_2 tests/KawasakiRobotTestForLinux_Valu3s_ino_2.cpp)
#MEOZKAN - the following line was added 20.03.2023
add_executable(KawasakiRobotTestForLinux_Valu3s_ino_2_safetyStop tests/KawasakiRobotTestForLinux_Valu3s_ino_2_safetyStop.cpp)
add_executable(KawasakiRobot_safetyStop_test tests/KawasakiRobot_safetyStop_test.cpp)

target_link_libraries(KawasakiRobotTestForLinux_Valu3s_ino_2 ${catkin_LIBRARIES} sir_robot_ros_interface)
#MEOZKAN - the following line was added 20.03.2023
target_link_libraries(KawasakiRobotTestForLinux_Valu3s_ino_2_safetyStop ${catkin_LIBRARIES} sir_robot_ros_interface)
target_link_libraries(KawasakiRobot_safetyStop_test ${catkin_LIBRARIES} sir_robot_ros_interface)


add_dependencies(KawasakiRobotTestForLinux_Valu3s_ino_2 ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
#MEOZKAN - the following line was added 20.03.2023
add_dependencies(KawasakiRobotTestForLinux_Valu3s_ino_2_safetyStop ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(KawasakiRobot_safetyStop_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(KawasakiRobotJointPublisher tests/KawasakiRobotJointPublisher.cpp)
add_executable(kawasaki_joints_real_to_sim tests/kawasaki_joints_real_to_sim.cpp)
add_executable(kawasaki_kinematics_tf tests/kawasaki_kinematics_tf.cpp)

target_link_libraries(KawasakiRobotJointPublisher ${catkin_LIBRARIES} sir_robot_ros_interface)
target_link_libraries(kawasaki_joints_real_to_sim ${catkin_LIBRARIES} sir_robot_ros_interface)
target_link_libraries(kawasaki_kinematics_tf ${catkin_LIBRARIES} sir_robot_ros_interface)

add_dependencies(KawasakiRobotJointPublisher ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(kawasaki_joints_real_to_sim ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
