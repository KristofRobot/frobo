#Set port
stty -F /dev/lidar cs8 115200 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts

#Start motor
echo 'MotorOn' > /dev/lidar
#Start sending raw data
echo 'ShowRaw' > /dev/lidar

#Start laser driver
rosrun xv_11_laser_driver neato_laser_publisher _port:=/dev/lidar _firmware_version:=2 _frame_id:=/lidar_link

#Stop motor when above process ends
echo 'MotorOff' > /dev/lidar
