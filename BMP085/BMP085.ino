// BMP085 Pressure sensor - !!NOTE!! 3.3 V sensor, account for that in hardware

#include <Wire.h>                        // I2C Library
#include <Adafruit_BMP085.h>             // BMP085 Library from Adafruit

Adafruit_BMP085 bmp;                    // Create BMP Class
  
void setup() {
  Serial.begin(9600);                    // Begin serial communication
  Serial.println("Type to Start...");    // Wait for serial communication
  while(!Serial.available());
  Serial.println("Millis, Temperature, Pressure, Altitude");
  bmp.begin();                           // Initialize BMP085
}
  
void loop() {
    Serial.print(millis());              // Print millis for logging purposes
    Serial.print(",");
    
    Serial.print(bmp.readTemperature());  // Print Temperature
    Serial.print(", ");
    
    Serial.print(bmp.readPressure());     // Print pressure
    Serial.print(", ");
    
    Serial.println(bmp.readAltitude(101325));  // Print altitude - Unverified Accuracy
      
    delay(500);                           // Pause
}
