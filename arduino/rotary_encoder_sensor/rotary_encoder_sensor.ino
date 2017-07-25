// Include libraries
#include "Metro.h"     //Include Metro library
#include "MLX90316.h"  // Include MLX90316 library
#define USE_USBCON
#include <ros.h>
#include <std_msgs/Float32.h>

// ROS variables
//NodeHandle_: # subscribers, #  publishers, input buffer(bytes), output buffer)(bytes)
ros::NodeHandle_<ArduinoHardware, 0, 1, 200, 200> nh;

std_msgs::Float32 angle_msg;

ros::Publisher pub_steering_angle("steering_angle", &angle_msg);

long publisher_timer = 0;
int period = 10; // ms

// Connection with sensor
int pinMOSI = 6;  // pin DAT
int pinSCK = 5;   // pin CLK
int pinSS = 4;    // pin CS

int value;
double offset = 207.4;
double angle = 0; 

Metro mlxMetro = Metro(5);
MLX90316 mlx_1  = MLX90316();

void setup(){
  delay(50);
  // Init ROS publishers
  nh.initNode();
  nh.advertise(pub_steering_angle);
  delay(50);
  
  mlx_1.attach(pinSS, pinSCK, pinMOSI);
  
  delay(50);
}

void loop() {
  if (millis() > publisher_timer) {
    if (mlxMetro.check() == 1) {
      value = mlx_1.readAngle();
      angle = (value / 10) - offset;
  
      // Publish ROS messages
      angle_msg.data = angle;
      pub_steering_angle.publish(&angle_msg);
    }

    publisher_timer = millis() + period;
  }

  nh.spinOnce();
}
