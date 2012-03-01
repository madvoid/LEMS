// Dallas DS18B20 OneWire temperature sensor
// Code is modified example from DallasTemperature library

#include <OneWire.h>                                // OneWire and DallasTemperature libraries
#include <DallasTemperature.h>

OneWire oneWire(2);                                 // Initialize OneWire comm on pin 2
DallasTemperature dstemp(&oneWire);                 // Initialize DStemp sensor on OneWire bus 
DeviceAddress dsaddress;                            // Make device address

void setup(){
  Serial.begin(1200);                               // Serial comm start
  Serial.println("DS18B Temperature Tester");        
  Serial.println("Type to Start...");               // Wait for serial communication
  while(!Serial.available());

  Serial.print("Locating devices...");              // 
  dstemp.begin();                                   // Initialize DStemp sensor
  Serial.println("Found");

  Serial.print("Parasite power is: ");              // Check for parasite power
  if (dstemp.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  dstemp.getAddress(dsaddress, 0);                  // Set/Get Address?
  
  dstemp.setResolution(dsaddress, 12);              // Set resolution

  Serial.print("Device 0 Address: ");               // Print address
  printAddress(dsaddress);
  Serial.println();
  
  Serial.print("Device 0 Resolution: ");            // Print resolution
  Serial.print(dstemp.getResolution(dsaddress), DEC); 
  Serial.println();    
}

void loop(){ 
  dstemp.requestTemperatures();                     // Send the command to get temperatures
  float temp = dstemp.getTempC(dsaddress);          // Calculate temperature in celsius
  Serial.println(temp,4);                           // Print temperature
}


void printAddress(DeviceAddress deviceAddress)     // Print Hex address function
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}




