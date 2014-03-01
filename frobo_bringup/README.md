# Frobo Bringup Package

Based on  ROS by Example Vol 1 Package by Patrick Goebel, see https://github.com/pirobot/rbx1

Simulate:

     roslaunch frobo_bringup fake_frobo.launch
     rosrun rviz rviz -d `rospack find frobo_nav`/sim.rviz


Use Kinect as fake laserscanner:

     roslaunch frobo_bringup fake_laser.launch
