#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "co2.h"
#include "drive.h"

ros::NodeHandle nh;

//CO2 concentration in ppm
double CO2Val;

// Variables to store desired vehicle speed and turning rate
double v_d;     // [m/s]
double omega_d; // [rad/s]

//Sharp sensor pins
const byte FIR = A0;

//Get desired translational and angular velocities from ROS
void messageCb( const geometry_msgs::Twist& msg){
  omega_d = msg.angular.z * 2;
  v_d = msg.linear.x / 2;
}

//Setup subscriber
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

//Setup
void setup()
{
  // Open the serial port at 9600 bps
  Serial.begin(9600);
    
  //Initialize program
  nh.initNode();
  nh.subscribe(sub);
  motorInit();
  CO2Init();
}

//Main
void loop()
{
  //Serial.print("Sharp reading ");
  //Serial.print(analogRead(FIR));
  //Serial.print("\n");
  nh.spinOnce();

  //Dont allow forward motion if an object is in front of the robot
  if (analogRead(FIR) > 300)
    {
      driveVehicle(0, 0);
    }
  else
    {
      driveVehicle(omega_d, v_d);
    }
    
  //Get CO2 concentration and display to the NeoPixel array
  CO2Val = getCO2();
  displayCO2(CO2Val);
    
    
}
