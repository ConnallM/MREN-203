#include <ros.h>
#include <std_msgs/Float32.h>
#include "co2.h"
#include "drive.h"

//Setup ROS node and publisher
std_msgs::Float32 CO2_msg;
ros::Publisher pub_CO2("CO2", &CO2_msg);
ros::NodeHandle nh;

//CO2 concentration in ppm
double CO2Val;

// Variables to store desired vehicle speed and turning rate
double v_d = 0.25;     // [m/s]
double omega_d = 1; // [rad/s]

// Sharp sensor pins
const byte FIR = A0;
const byte LIR = A1;
const byte RIR = A2;

int minVal;
int val;

int readIR(const byte IR){
    minVal = analogRead(IR);
    for (int i = 0; i < 10; i++){
      val = analogRead(IR);
      if (val < minVal){
        minVal = val;
      }
      delay(10);
    }
    return minVal;
}

//Setup
void setup()
{
    //Open the serial port at 9600 bps
    Serial.begin(9600);

    //Initialize program
    nh.initNode();
    nh.advertise(pub_CO2);
    motorInit();
    CO2Init();
}

//Main loop
void loop()
{
    nh.spinOnce();

    //Object detected
    if (readIR(FIR) > 250){
      if (readIR(LIR) > readIR(RIR)){
        //Turn right
        while (readIR(FIR) > 150){
            driveVehicle(omega_d, -v_d);
        }
      }
      else{
        //Turn left
        while (readIR(FIR) > 150){
            driveVehicle(-omega_d, -v_d);
        }
      }
    }
    if (readIR(LIR) > 250 or readIR(RIR) > 250){
      driveVehicle(log(readIR(LIR)) - log(readIR(RIR)), v_d);
    }
    else{
      driveVehicle(0, v_d);
    }
      
    
    //Get the CO2 concentration, display to NeoPixel array, and publish the data to ROS
    CO2Val = getCO2();
    displayCO2(CO2Val);
    CO2_msg.data =CO2Val;
    pub_CO2.publish(&CO2_msg);
    
    
}
