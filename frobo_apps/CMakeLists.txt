cmake_minimum_required(VERSION 2.8.3)

project(frobo_apps)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY nodes/
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
USE_SOURCE_PERMISSIONS
)

