cmake_minimum_required(VERSION 2.8.3)
project(frobo_nav)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp nav_msgs actionlib move_base_msgs geometry_msgs visualization_msgs tf rospy)

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/CalibrateAngular.cfg cfg/CalibrateLinear.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure nav_msgs move_base_msgs geometry_msgs visualization_msgs)

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY maps
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY nodes/
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	USE_SOURCE_PERMISSIONS
)
