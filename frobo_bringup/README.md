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
     
## Create Bag file for mapping

     rosbag record -O map_data /scan /tf /odom

## Create Map from Bag file using hector_mapping (recommended)

     roscore
     rosparam set use_sim_time true
     roslaunch hector_mapping mapping_default.launch
     rosrun hector_mapping hector_mapping _base_frame:=base_footprint _map_resolution:=0.1
     rosbag play --clock map_data.bag
     rosrun map_server map_saver -f map
     
## Create Map from Bag file using gmapping 

     roscore
     rosparam set use_sim_time true
     rosrun gmapping slam_gmapping scan:=scan _delta:=0.1 _maxUrange:=4.99 _xmin:=-5.0 _ymin:=-5.0 _xmax:=5.0 _ymax:=5.0 _particles:=30 _srr:=0 _srt:=0 _str:=0 _stt:=0.1 _minimumScore:=10000
     rosbag play --clock map_data.bag
     rosrun map_server map_saver -f map
     
## Follow

### Object Follower

Color blob tracking:

     roslaunch frobo_bringup kinect_rgb.launch
     roslaunch frobo_vision camshift.launch
     
Object follower:

     roslaunch frobo_apps object_follower.launch

### Person Follower

#### Python

     roslaunch frobo_bringup kinect_pointcloud.launch
     roslaunch frobo_apps follower2.launch

#### C++ (best performance)

     roslaunch frobo_follower follower.launch
     
OR

     scripts/follower.sh
     
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
