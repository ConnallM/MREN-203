/*
 * Connall Milberry
 * PI Controller for LynxMotion rover
 */

// Motor driver direction pin
const byte EA = 5;   // Right wheel PWM pin
const byte I1 = 4;   // Right wheel direction digital pin 1
const byte I2 = 3;   // Right wheel direction digital pin 2

const byte EB = 6;   //Left wheel PWM pin
const byte I3 = 7;  //Left wheel direction digital pin 1
const byte I4 = 8;   //Left wheel direction digital pin 2

// Motor PWM command variables [0-255]
int uL = 0;
int uR = 0;

// Left wheel encoder digital pins
const byte SIGNAL_A_L = 12;
const byte SIGNAL_B_L = 11;

// Right wheel encoder digital pins
const byte SIGNAL_A_R = 10;
const byte SIGNAL_B_R = 9;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;
const double ELL = 0.2775;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks_L = 0;
volatile long encoder_ticks_R = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;

// Left and right wheel speed variables
double V_L = 0;
double V_R = 0;

// Left and right wheel desired speeds
double dVL;
double dVR;

// Overall desired vehicle translational speed, positive forward, negative backwards
double V = 0.8;

// Overall desired vehicle turning rate, positive cw, negative ccw
double omega = 0.2;

// Sampling interval for measurements in milliseconds
const int T = 100;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

// Integral error
long e_intL = 0;
long e_intR = 0;

// Compute vehicle speed [m/s]
double getSpeed(double V_L, double V_R)
{
  return 0.5*(V_L+V_R);
}

// Compute vehicle turning rate
double getRotation (double V_L, double V_R)
{
  return 1 / ELL * (V_R - V_L);
}

// Calculate the desired left wheel speed
double getLeftWheelSpeed (double omega, double V)
{
  return V - omega*ELL/2;
}

// Calculate the desired right wheel speed
double getRightWheelSpeed (double omega, double V)
{
  return omega*ELL/2 + V;
}

short PI_controller (double e_now, double k_p, double e_int, double k_i)
{
  short u;
  u = short(k_p * e_now);
  if (u > 255)
  {
    u = 255;
  }
  else if  (u < -255)
  {
    u = -255;
  }
  return u;
}

// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicks_L()
{
    if (digitalRead(SIGNAL_B_L) == HIGH)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticks_L++;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticks_L--;
    }
}

void decodeEncoderTicks_R()
{
    if (digitalRead(SIGNAL_B_R == HIGH))
    {
        encoder_ticks_R++;
    }
    else
    {
        encoder_ticks_R--;
    }
}

void setup()
{
    // Open the serial port at 9600 bps
    Serial.begin(9600);

    // Set the pin modes for the motor driver
    pinMode(EA, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    // Set the pin modes for the encoders
    pinMode(SIGNAL_A_L, INPUT);
    pinMode(SIGNAL_A_R, INPUT);
    pinMode(SIGNAL_B_L, INPUT);
    pinMode(SIGNAL_B_R, INPUT);

    // Every time the pin goes high, this is a pulse
    attachInterrupt(digitalPinToInterrupt(SIGNAL_A_L), decodeEncoderTicks_L, RISING);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_A_R), decodeEncoderTicks_R, RISING);

    // Get desired left and right wheel speeds
    dVL = getLeftWheelSpeed(omega, V);
    dVR = getRightWheelSpeed(omega, V);

    // Print a message
    //Serial.print("Program initialized.");
    //Serial.print("\n");
}

void loop()
{
    // Get the elapsed time [ms]
    t_now = millis();

    if (t_now - t_last >= T)
    {
        // Estimate the rotational speed [rad/s]
        omega_L = 2.0 * PI * ((double)encoder_ticks_L / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        omega_R = 2.0 * PI * ((double)encoder_ticks_R / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        V_L = omega_L * RHO;
        V_R = omega_R * RHO;

        // Print some stuff to the serial monitor
        /*
        Serial.print("Left encoder ticks: ");
        Serial.print(encoder_ticks_L);
        Serial.print("\n");
        Serial.print("Right encoder ticks: ");
        Serial.print(encoder_ticks_R);
        Serial.print("\n\n");
        Serial.print("Estimated left wheel speed: ");
        Serial.print(omega_L);
        Serial.print(" rad/s");
        Serial.print("\n");
        Serial.print("Estimated right wheel speed: ");
        Serial.print(omega_R);
        Serial.print(" rad/s");
        Serial.print("\n\n");
        */

        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        encoder_ticks_L = 0;
        encoder_ticks_R = 0;
    }

    // Set the wheel motor PWM command [0-255]
    uL = PI_controller (dVL - V_L, 200, e_intL, 100);
    uR = PI_controller (dVR - V_R, 200, e_intR, 100);
    e_intL += dVL - V_L;
    e_intR += dVR - V_R;
    
    Serial.print("Wheel speeds\n");
    Serial.print(V_L);
    Serial.print("\n");
    Serial.print(V_R);
    Serial.print("\n");
    

    
    Serial.print("PWM signal\n");
    Serial.print(uL);
    Serial.print("\n");
    Serial.print(uR);
    Serial.print("\n");
    

    
    

    // Select a direction
    if (uL > 0)
    {
      digitalWrite(I3, LOW);
      digitalWrite(I4, HIGH);
    }
    else
    {
      digitalWrite(I3, HIGH);
      digitalWrite(I4, LOW);
    }

    if (uR > 0)
    {
      digitalWrite(I1, HIGH);
      digitalWrite(I2, LOW);
    }
    else
    {
      digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    }
    
    

    // PWM command to the motor driver
    analogWrite(EA, abs(uR));
    analogWrite(EB, abs(uL));
}
