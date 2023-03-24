#ifndef CO2_H
#define CO2_H
#include <Arduino.h>
#include <Adafruit_SCD30.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_NeoPixel.h>

void CO2Init();
void displayCO2(double CO2);
double getCO2();
#endif