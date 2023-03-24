#include "CO2.h"

Adafruit_SCD30 scd;
Adafruit_SGP30 sgp;

int pin = A5;      // Data pin for the NeoPixel array
int numPixels = 8; // Define the number of NeoPixels on the board
int pixelFormat = NEO_RGBW + NEO_KHZ800;
int pixelNum;

Adafruit_NeoPixel *pixels;

uint32_t getAbsoluteHumidity(float temperature, float humidity);

void CO2Init(){
  if (!scd.begin())
    {
        Serial.println("Sensor not found :(");
        while (1)
        {
            delay(10); // This will stay here forever if a sensor isn't found
        }
    }

    // Then create a new NeoPixel object dynamically with these values:
    pixels = new Adafruit_NeoPixel(numPixels, pin, pixelFormat);

    // Initialize NeoPixel strip object
    pixels->begin();


    scd.setMeasurementInterval(2);
    // Read the measurement interval [s]
    Serial.print("Measurement Interval: ");
    Serial.print(scd.getMeasurementInterval());
    Serial.println(" seconds");


    // Print a message
    Serial.print("Program initialized.");
    Serial.print("\n");
}

void displayCO2(double CO2){
  pixels->clear(); // Set all pixel colors to 'off'
   pixelNum = floor((CO2 - 419)/100);
   if (pixelNum > 8){
     pixelNum = 8;
    }

    for (int i = 0; i < pixelNum; i++){
      if (i == 0){
        pixels->setPixelColor(i, pixels->Color(0, 0, 100, 0));
      }
      else if (i == 1){
        pixels->setPixelColor(i, pixels->Color(50, 0, 50, 0));
      }
      else if (i == 2){
        pixels->setPixelColor(i, pixels->Color(100, 0, 0, 0));
      }
      else if (i == 3){
        pixels->setPixelColor(i, pixels->Color(100, 50, 0, 0));
      }
      else if (i == 4){
        pixels->setPixelColor(i, pixels->Color(50, 50, 0, 0));
      }else if (i == 5){
        pixels->setPixelColor(i, pixels->Color(40, 60, 0, 0));
      }
      else if (i == 6){
        pixels->setPixelColor(i, pixels->Color(10, 90, 0, 0));
      }
      else{
        pixels->setPixelColor(i, pixels->Color(0, 100, 0, 0));
      }
    }
    pixels->show(); // Send the updated pixel colors to the hardware.
}

double getCO2(){
  if (scd.dataReady())
    {
        // The library also has a handy check if data can't be read
        if (!scd.read())
        {
            Serial.println("Error reading sensor data");
            return;
        }

        /*
        Serial.print("CO2: ");
        Serial.print(scd.CO2, 3);
        Serial.println(" ppm");
        Serial.print("TVOC ");
        Serial.print(sgp.TVOC);
        Serial.print(" ppb\t");
        Serial.print("eCO2 ");
        Serial.print(sgp.eCO2);
        Serial.println(" ppm");
        */


        return (scd.CO2 + sgp.eCO2);
    }
}