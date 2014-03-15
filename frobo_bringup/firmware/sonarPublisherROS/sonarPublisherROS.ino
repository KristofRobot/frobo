/* 
 * Based on rosserial Ultrasound Example 
 *
 * Publishes sonar readings as LaserScan msgs
 *
 * Run: rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=BAUD_RATE
 * 
 */

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>

#include <NewPing.h>
#include <FastRunningMedian.h>

const long BAUD_RATE=115200; //change baud rate here

const byte MAX_DISTANCE=60; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm. No reason to wait longer for ping to return than this.

NewPing sonarFront[3] = { // Sensor object array.
  NewPing(11, 11, MAX_DISTANCE), //left
  NewPing(6, 6, MAX_DISTANCE),   //middle
  NewPing(5, 5, MAX_DISTANCE)   //right
};

const int pubSpeed = 500;  //publish every x milliseconds
unsigned long pubTimer;     

const int pingSpeed = 50; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second; retriggering of same sonar is best kept above 30ms
unsigned long pingTimer;     // Holds the next ping time.
byte pingPointer=0;            //holds position of next sonar to use; will overflow by design
const byte FRONT_PING_NUM=3;    //number of sonars
const byte PING_MEDIAN_NUM=3;  //holds number of pings for median filter
const byte NUM_SONAR_POINTS=20;   //number of points to fill sonar range

FastRunningMedian<unsigned int,5, 0> frontDistance[FRONT_PING_NUM];

const unsigned int MAX_ECHO_TIME = min(MAX_DISTANCE, MAX_SENSOR_DISTANCE) * US_ROUNDTRIP_CM + (US_ROUNDTRIP_CM / 2); // Calculate the maximum distance in uS.
float ranges[NUM_SONAR_POINTS];

ros::NodeHandle nh;
sensor_msgs::LaserScan laser_msg;
ros::Publisher pub_ranges[FRONT_PING_NUM] = {
   ros::Publisher("~sonar_fl", &laser_msg),
   ros::Publisher("~sonar_fc", &laser_msg),
   ros::Publisher("~sonar_fr", &laser_msg)
};


char *frameids[FRONT_PING_NUM] = {
  "/base_fl_sonar_link", //left
  "/base_fc_sonar_link", //center
  "/base_fr_sonar_link"  //right
};


void setup()
{
  //set baud rate
  nh.getHardware()->setBaud(BAUD_RATE);
  nh.initNode();
  
  for (int i=0; i<FRONT_PING_NUM; i++){
    nh.advertise(pub_ranges[i]);
  }
    //width of 30 degrees ~= 0.6 rads
    laser_msg.angle_min = -0.3;
    laser_msg.angle_max = 0.3;
    laser_msg.angle_increment = (laser_msg.angle_max - laser_msg.angle_min)/NUM_SONAR_POINTS;  //width / number of samples
    laser_msg.time_increment = 0.0;
    laser_msg.range_min = 0.0;
    laser_msg.range_max = MAX_DISTANCE+1/100.0; //should be > MAX_DISTANCE (not >=!) 
    laser_msg.ranges_length = NUM_SONAR_POINTS;
 
 
  //get first distances
  for (int i=0; i<FRONT_PING_NUM*PING_MEDIAN_NUM; i++){
      getNextDistance();
      delay(50); //need to wait at least 30ms between pings; with three sonars; 10ms
  }
  
}

void loop()
{
  //if pingSpeed time has passed, trigger new sonar ping
  if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
      //recalibrate pingTimer - to avoid problems
      pingTimer = millis()+pingSpeed;
      getNextDistance();
  }
  
  if (millis() >= pubTimer) {
    pubTimer = millis()+pubSpeed;
    for (int i=0; i<FRONT_PING_NUM; i++){
      float dist = getMedianPing(i)/100.0;   //return ping distance in meters
      for (int j=0; j<NUM_SONAR_POINTS;j++) ranges[j] = dist;
      laser_msg.ranges = ranges;
      laser_msg.header.stamp = nh.now();
      laser_msg.header.frame_id = frameids[i];
      pub_ranges[i].publish(&laser_msg);
      //nh.spinOnce();
    }
   nh.spinOnce();
  }
}

/* 
* Get next distance by pinging using the next sonar in row
* and adding the value to its running median
*/
void getNextDistance(){
  byte sonarPos = pingPointer%FRONT_PING_NUM;
  
  int dist = sonarFront[sonarPos].ping_cm();
  frontDistance[sonarPos].addValue(dist);
  
  pingPointer++; //increment, and let it overflow when it reaches max (255)
}

/* Return minimal front distance detected */
int getMedianPing(byte sonarPos){
   return frontDistance[sonarPos].getMedian();
}
