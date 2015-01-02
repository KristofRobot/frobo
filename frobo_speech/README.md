# Frobo Speech Package

Based on  ROS by Example Vol 1 Speech Package by Patrick Goebel, see https://github.com/pirobot/rbx1

# Usage

## Speak

     roscore
     rosrun sound_play soundplay_node.py
     rosrun sound_play say.py "I am Frobo"

## Talkback

     roslaunch frobo_speech talkback.launch
     
## Speech-based move-base

     roslaunch frobo_speech frobo_speech_nav.launch map:=hector_laser_map_360.yaml
     
