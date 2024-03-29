#include "drive.h"

// Left wheel PWM control
int EB = 6; // Wheel PWM pin (must be a PWM pin)
int I3 = 7; // Wheel direction digital pin 1
int I4 = 8; // Wheel direction digital pin 2

// Right wheel PWM control
int EA = 5; // Wheel PWM pin (must be a PWM pin)
int I1 = 4; // Wheel direction digital pin 1
int I2 = 3; // Wheel direction digital pin 2

// Left wheel encoder digital pins
const byte SIGNAL_AL = 12; // green wire
const byte SIGNAL_BL = 11; // yellow wire

// Right wheel encoder digital pins
const byte SIGNAL_AR = 10;  // green wire
const byte SIGNAL_BR = 9; // yellow wire

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Vehicle track [m]
const double ELL = 0.2775;

// Sampling interval for measurements in milliseconds
const int T = 100;

// Controller gains (use the same values for both wheels)
const double KP = 200.0; // Proportional gain
const double KI = 100.0; // Integral gain

// Motor PWM command variables [0-255]
short u_L = 0;
short u_R = 0;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks_L = 0;
volatile long encoder_ticks_R = 0;

// Variables to store estimated angular rates of wheels [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;

// Variables to store estimated wheel speeds [m/s]
double v_L = 0.0;
double v_R = 0.0;

// Variables to store vehicle speed and turning rate
double v = 0.0;     // [m/s]
double omega = 0.0; // [rad/s]

// Variable to store desired wheel speeds [m/s]
double v_Ld = 0.0;
double v_Rd = 0.0;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// Variables to store errors for controller
double e_L = 0.0;
double e_R = 0.0;
double e_Lint = 0.0;
double e_Rint = 0.0;

void motorInit(){
  // Configure digital pins for output
    pinMode(EA, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);

    // Configure digital pins for output
    pinMode(EB, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    // Set the pin modes for the encoders
    pinMode(SIGNAL_AL, INPUT);
    pinMode(SIGNAL_BL, INPUT);
    pinMode(SIGNAL_AR, INPUT);
    pinMode(SIGNAL_BR, INPUT);

    // Send 0 PWM commands
    analogWrite(EA, 0);
    analogWrite(EB, 0);

    // Send brake signals to motor driver
    digitalWrite(I1, LOW);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, LOW);

    // Every time the pin goes high, this is a pulse; enable the interrupts
    attachInterrupt(digitalPinToInterrupt(SIGNAL_AL), decodeEncoderTicks_L,
                    RISING);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_AR), decodeEncoderTicks_R,
                    RISING);
}

// This function is called when SIGNAL_AL (left encoder) goes HIGH
void decodeEncoderTicks_L()
{
    if (digitalRead(SIGNAL_BL) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticks_L--;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticks_L++;
    }
}

// This function is called when SIGNAL_AR (right encoder) goes HIGH
void decodeEncoderTicks_R()
{
    if (digitalRead(SIGNAL_BR) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticks_R++;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticks_R--;
    }
}

// Compute the wheel rate from elapsed time and encoder ticks [rad/s]
double compute_wheel_rate(long encoder_ticks, double delta_t)
{
    double omega;
    omega = 2.0 * PI * ((double)encoder_ticks / (double)TPR) * 1000.0 / delta_t;
    return omega;
}

// Compute wheel speed [m/s]
double compute_wheel_speed(double omega_wheel)
{
    double v_wheel;
    v_wheel = omega_wheel * RHO;
    return v_wheel;
}

// Compute vehicle speed [m/s]
double compute_vehicle_speed(double v_L, double v_R)
{
    double v;
    v = 0.5 * (v_L + v_R);
    return v;
}

// Compute vehicle turning rate [rad/s]
double compute_vehicle_rate(double v_L, double v_R)
{
    double omega;
    omega = 1.0 / ELL * (v_R - v_L);
    return omega;
}

// Compute v_L from v and omega
double compute_L_wheel_speed(double v, double omega)
{
    double v_wheel = 0.0;
    v_wheel = v - ELL / 2.0 * omega;
    return v_wheel;
}

// Compute v_R from v and omega
double compute_R_wheel_speed(double v, double omega)
{
    double v_wheel = 0.0;
    v_wheel = v + ELL / 2.0 * omega;
    return v_wheel;
}

// Wheel speed PI controller function
short PI_controller(double e_now, double e_int, double k_P, double k_I)
{
    short u;
    u = (short)(k_P * e_now + k_I * e_int);

    // Saturation (i.e., maximum input) detection
    if (u > 255)
    {
        u = 255;
    }
    else if (u < -255)
    {
        u = -255;
    }
    return u;
}

// This function applies PWM inputs (u_L and u_R) to the right and left wheels
void driveVehicle(double omega_d, double v_d)
{
    v_Ld = compute_L_wheel_speed(v_d, omega_d);
    v_Rd = compute_R_wheel_speed(v_d, omega_d);
    
    t_now = millis();

    // Perform control update every T milliseconds
    if (t_now - t_last >= T)
    {

        // Set the desired vehicle speed and turning rate
        v_d = 0.5;     // [m/s]
        omega_d = 0.0; // [rad/s]

        // Estimate the rotational speed of each wheel [rad/s]
        omega_L = compute_wheel_rate(encoder_ticks_L, (double)(t_now - t_last));
        omega_R = compute_wheel_rate(encoder_ticks_R, (double)(t_now - t_last));

        // Compute the speed of each wheel [m/s]
        v_L = compute_wheel_speed(omega_L);
        v_R = compute_wheel_speed(omega_R);

        /*
        Serial.print("Left wheel speed ");
        Serial.print(v_L);
        Serial.print("\n");
        Serial.print("Right wheel speed ");
        Serial.print(v_R);
        Serial.print("\n");
        */

        // Compute the speed of the vehicle [m/s]
        v = compute_vehicle_speed(v_L, v_R);

        // Compute the turning rate of the vehicle [rad/s]
        omega = compute_vehicle_rate(v_L, v_R);

        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        encoder_ticks_L = 0;
        encoder_ticks_R = 0;

        // Compute errors
        e_L = v_Ld - v_L;
        e_R = v_Rd - v_R;
        
        Serial.print("Left and right errors");
        Serial.print("\n");
        Serial.print(e_L);
        Serial.print("\t");
        Serial.print(e_R);
        Serial.print("\n");
        

        // Integrate errors with anti-windup
        if (abs(u_L) < 255)
        {
            e_Lint += e_L;
        }
        if (abs(u_R) < 255)
        {
            e_Rint += e_R;
        }

        // Compute control signals using PI controller
        u_L = PI_controller(e_L, e_Lint, KP, KI);
        u_R = PI_controller(e_R, e_Rint, KP, KI);
    
      // LEFT WHEEL
      if (u_L < 0) // If the controller calculated a negative input...
      {
          digitalWrite(I3, HIGH); // Drive backward (left wheels)
          digitalWrite(I4, LOW);  // Drive backward (left wheels)
  
          analogWrite(EB, -u_L); // Write left motors command
      }
      else // the controller calculated a positive input
      {
          digitalWrite(I3, LOW);  // Drive forward (left wheels)
          digitalWrite(I4, HIGH); // Drive forward (left wheels)
  
          analogWrite(EB, u_L); // Write left motors command
      }
  
      // RIGHT WHEEL
      if (u_R < 0) // If the controller calculated a negative input...
      {
          digitalWrite(I1, LOW);  // Drive backward (right wheels)
          digitalWrite(I2, HIGH); // Drive backward (right wheels)
  
          analogWrite(EA, -u_R); // Write right motors command
      }
      else // the controller calculated a positive input
      {
          digitalWrite(I1, HIGH); // Drive forward (right wheels)
          digitalWrite(I2, LOW);  // Drive forward (right wheels)
  
          analogWrite(EA, u_R); // Write right motors command
      }
    }
}