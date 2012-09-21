// SHT75 Temperature and Humidity Sensor
// Uses SHT1x library from Jonathan Oxer

#include <SHT1x.h>
#include <OneWire.h>                                // OneWire and DallasTemperature libraries
#include <DallasTemperature.h>

OneWire oneWire(38);                                 // Initialize OneWire comm on pin 2
DallasTemperature dstemp(&oneWire);                 // Initialize DStemp sensor on OneWire bus 
DeviceAddress dsaddress; 

#define dataPin  18
#define clockPin 19
SHT1x sht1x(dataPin, clockPin);

float tempC;                              // Initialize Variables
float tempF;
float humidity;
unsigned long startTime;
unsigned long endTime;

void setup()
{
  Serial.begin(115200); // Open serial connection to report values to host
  dstemp.begin();     
  dstemp.getAddress(dsaddress, 0);                  // Set/Get Address?
  dstemp.setResolution(dsaddress, 12);              // Set resolution                              // Initialize DStemp sensor
  Serial.println("Type To Start...");
  while(!Serial.available());
  Serial.println("TempC,TempF,RelHum (%) :: Elapsed Time :: DStemp");
}

void loop()
{
  startTime = millis();
  tempC = sht1x.readTemperatureC();         // Take Readings
  tempF = sht1x.readTemperatureF();
  humidity = sht1x.readHumidity();
  endTime = millis();

  Serial.print(tempC);
  Serial.print(",");
  Serial.print(tempF);
  Serial.print(",");
  Serial.print(humidity);
  Serial.print(" :: ");
  Serial.print(endTime-startTime);
  Serial.print(" :: ");

  dstemp.requestTemperatures();                     // Send the command to get temperatures
  float temp = dstemp.getTempC(dsaddress);          // Calculate temperature in celsius
  Serial.println(temp,4);   

  delay(1000);
}