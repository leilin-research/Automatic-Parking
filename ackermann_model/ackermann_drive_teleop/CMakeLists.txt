cmake_minimum_required(VERSION 2.8.3)
project(ackermann_drive_teleop)

find_package(catkin REQUIRED COMPONENTS roslaunch
  rospy
  ackermann_msgs
)

catkin_package(
  INCLUDE_DIRS scripts
  LIBRARIES
  CATKIN_DEPENDS
    rospy
    ackermann_msgs
  DEPENDS
)

roslaunch_add_file_check(launch)

catkin_install_python(PROGRAMS scripts/ackermann_drive_keyop.py scripts/ackermann_drive_joyop.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
