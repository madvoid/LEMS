// HIH4030 Humidity sensor with DHT22 humidity/temperature sensor

#include "DHT.h"					// DHT22 sensor library
DHT dht(51, DHT22);					// DHT22 Pin is digital 51 The function initializes the sensor

const int hih_pin = A0;                                 // HIH4030 pin
int hih_raw = 0;                                        // Raw HIH4030 reading
float hih_hum = 0.0;                                    // Calculated humidity
float hih_tchum = 0.0;                                  // Temperature corrected humidity

float dht_hum = 0.0;                                    // DHT22 humidity
float dht_temp = 0.0;                                   // DHT22 temperature


void setup(){
  Serial.begin(1200);                                   // Serial begin communication    
  Serial.println("Type to Start...");                   // Wait for serial commands
  while(!Serial.available());
  Serial.println("DHT22 Temp, DH22 Hum, HIH4030 Hum, HIH4030 Temp. Corrected Hum");
}


void loop(){
  hih_raw = analogRead(hih_pin);                        // Analog read HIH4030
  dht_hum = dht.readHumidity();                         // DHT get measurements
  dht_temp = dht.readTemperature();

  hih_hum = 0.15751*(float)hih_raw-25.8065;             // Calculate Humidity
  hih_tchum = hih_hum/(1.0546-0.00216*dht_temp);

  Serial.print(dht_temp);                               // Print output
  Serial.print(",");
  Serial.print(dht_hum);
  Serial.print(",");
  Serial.print(hih_hum);
  Serial.print(",");
  Serial.println(hih_tchum);

  delay(5000);                                          // Can't really poll any faster than 5000 ms
}



