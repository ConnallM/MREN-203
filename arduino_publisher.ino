#include <ros.h>
#include <std_msgs/Float32.h>
#include "co2.h"
#include "drive.h"

 ros::Publisher pub_CO2("CO2", &CO2_msg);

ros::NodeHandle nh;

double CO2Val;

// Variables to store desired vehicle speed and turning rate
double v_d;     // [m/s]
double omega_d; // [rad/s]

/* PIN CONNECTIONS */

// Sharp sensor pins
const byte FIR = A0;
const byte LIR = A1;
const byte RIR = A2;

/* HELPER FUNCTIONS */

/* SETUP FUNCTION */

void setup()
{
    // Open the serial port at 9600 bps
    Serial.begin(9600);

    nh.initNode();
    nh.advertise(pub_temp);
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

    //Object detected
    if (analogRead(FIR) > 500)
    {
      if (analogRead(LIR) > analogRead(RIR)){
        //Turn left
        while (analogRead(FIR) < 1000){
            driveVehicle(omega_d, 0)
        }
      }
      else{
        //Turn right
        while (analogRead(FIR) < 1000){
            driveVehicle(-omega_d, 0)
        }
      }
    }
    else
    {
      driveVehicle(omega_d, v_d);
    }
    
    CO2Val = getCO2();
    displayCO2(CO2Val);
    CO2_msg.data =CO2Val;
    pub_CO2.publish(&CO2_msg);
    
    
}
