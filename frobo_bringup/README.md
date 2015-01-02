# Frobo Bringup Package

Based on  ROS by Example Vol 1 Package by Patrick Goebel, see https://github.com/pirobot/rbx1

## Simulate Frobo

     roslaunch frobo_bringup fake_frobo.launch
     rosrun rviz rviz -d `rospack find frobo_nav`/sim.rviz

## Use real Frobo
### Start Frobo

     scripts/robotStart.sh

### Start laser scan
Using Kinect:

     scripts/kinectLaserStart.sh
     
Using XV11 lidar:

     scripts/laserStart.sh

### Start optional components     
IMU:

     scripts/imuStart.sh
     
Collision detection Sonars:

     scripts/sonarStart.sh

### Navigate     
Navigate around on a map using RVIZ Nav goals:

     scripts/mapStart.sh
     
Patrol between defined checkpoints:

     scripts/patrol.sh
     
Patrol based on speech recognition:

     scripts/speechNav.sh

Stall detection during navigation using move_base:

     scripts/stallDetection.sh
     
## Create Map

     roscore
     rosparam set use_sim_time true
     roslaunch hector_mapping mapping_default.launch
     rosbag play --clock laser_map_360.bag
     rosrun map_server map_saver -f hector_laser_map_360
     
## Other scripts
Talk using Festival:

     scripts/talk.sh

Speech recognition and talk back:

     scripts/talkback.sh

Start Kinect and view image:

     scripts/kinectStart.sh
     rosrun image_view image_view image:=/camera/rgb/image_raw

Connect directly to ROS Arduino motor controller 
(for commands, see https://github.com/KristofRobot/ros_arduino_bridge/blob/master/ros_arduino_firmware/src/libraries/ROSArduinoBridge_motor_only/commands.h) :

     scripts/connectArduinoMotor.sh

Sync time:

     scripts/ntp.sh
