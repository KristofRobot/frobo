# Frobo Navigation Package

Based on  ROS by Example Vol 1 Package by Patrick Goebel, see https://github.com/pirobot/rbx1

## Usage

Stall Detection:

  roslaunch frobo_nav frobo_stall_detect.launch
  
Patrol:

  roslaunch frobo_nav frobo_nav_test.launch map:=hector_laser_map_360.yaml
  
Calibration:

  nav_calibrate_linear.py
  nav_calibrate_angular_cw.py
  nav_calibrate_angular_ccw.py
  nav_umbmark_cw.py
  nav_umbmark_ccw.py




