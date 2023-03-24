#ifndef DRIVE_H
#define DRIVE_H
#include <Arduino.h>

void motorInit();
void decodeEncoderTicks_L();
void decodeEncoderTicks_R();
double compute_wheel_rate(long encoder_ticks, double delta_t);
double compute_wheel_speed(double omega_wheel);
double compute_vehicle_speed(double v_L, double v_R);
double compute_vehicle_rate(double v_L, double v_R);
double compute_L_wheel_speed(double v, double omega);
double compute_R_wheel_speed(double v, double omega);
short PI_controller(double e_now, double e_int, double k_P, double k_I);
void driveVehicle(double omega_d, double v_d);
#endif

