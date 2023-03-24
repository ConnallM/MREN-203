#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "co2.h"
#include "drive.h"

ros::NodeHandle nh;

double CO2Val;

// Variables to store desired vehicle speed and turning rate
double v_d;     // [m/s]
double omega_d; // [rad/s]

/* PIN CONNECTIONS */

// Sharp sensor pins
const byte FIR = A0;

/* HELPER FUNCTIONS */

void messageCb( const geometry_msgs::Twist& msg){
  omega_d = msg.angular.z * 4;
  v_d = msg.linear.x;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

/* SETUP FUNCTION */

void setup()
{
    // Open the serial port at 9600 bps
    Serial.begin(9600);

    nh.initNode();
    nh.subscribe(sub);
    motorInit();
    CO2Init();
}

/* MAIN PROGRAM LOOP */

void loop()
{
    //Serial.print("Sharp reading ");
    //Serial.print(analogRead(FIR));
    //Serial.print("\n");
    nh.spinOnce();

    if (analogRead(FIR) > 300)
    {
      driveVehicle(0, 0);
    }
    else
    {
      // Get left and right wheel desired speeds
      driveVehicle(omega_d, v_d);
    }
    
    CO2Val = getCO2();
    displayCO2(CO2Val);
    
    
}