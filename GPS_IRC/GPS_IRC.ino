#include <ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <TinyGPS++.h>

// Define the ROS node and topic
ros::NodeHandle nh;
sensor_msgs::NavSatFix gps_msg;


// Create an instance of the TinyGPS++ library
TinyGPSPlus gps;

// Define the serial port for the GPS module
#include <SoftwareSerial.h>

// Define the serial port for the GPS module
SoftwareSerial GPS_SERIAL(2, 3);  // RX, TX

void setup() {
  // Initialize ROS
  nh.initNode();
  nh.advertise("/gps_data", &gps_msg);
  
  // Initialize the serial communication with the GPS module
  GPS_SERIAL.begin(9600);
}

void loop() {
  // Read GPS data
  while (GPS_SERIAL.available()) {
    gps.encode(GPS_SERIAL.read());
  }

  // Check if we have valid GPS data
  if (gps.location.isValid()) {
    gps_msg.header.stamp = nh.now();
    gps_msg.latitude = gps.location.lat();
    gps_msg.longitude = gps.location.lng();
    gps_msg.altitude = gps.altitude.meters();
    
    const char* topic_name = "/gps_data";
    // Publish GPS data on the ROS topic
    nh.advertise(topic_name, &gps_msg);
    ros::Publisher<sensor_msgs::NavSatFix> gps_pub(/gps_data, &gps_msg);
  }

  // You can add additional logic and delays here as needed
}
