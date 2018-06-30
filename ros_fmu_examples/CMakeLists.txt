cmake_minimum_required(VERSION 2.8.3)
project(ros_fmu_examples)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_fmu_examples
#  CATKIN_DEPENDS ros_fmu roscpp
#  DEPENDS system_lib
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY share/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
