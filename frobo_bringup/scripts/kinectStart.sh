echo "Enabling Kinect..."
echo '1-1.3' > /sys/bus/usb/drivers/usb/bind
echo "Done"
roslaunch frobo_bringup kinect.launch
