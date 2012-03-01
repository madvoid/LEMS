README.txt
Arduino code for various sensors for the LEMS


Includes code for following sensors:
LI200 Pyranometer with MCP3201 ADC
BMP085 Pressure Sensor
DS18B20 Temperature Sensor
HIH4030 Humidity Sensor
DHT22 Temperature/Humidity Sensor
TN9 Infrared Temperature Sensor
Decagon 5TM Soil Moisture/Temperature Sensor
All of the above combined

The aforementioned sensors will need the following libraries
Adafruit_BMP085 Library -- https://github.com/adafruit/Adafruit-BMP085-Library
DHT Library -- https://github.com/adafruit/DHT-sensor-library
OneWire Library -- http://www.pjrc.com/teensy/td_libs_OneWire.html
DallasTemperature Library -- http://milesburton.com/Dallas_Temperature_Control_Library


The combined sensor code also uses Adafruit's datalogger shield
The datalogger shield needs the following libraries:
SD Library -- https://github.com/adafruit/SD
RTC Library -- https://github.com/adafruit/RTClib


Many thanks to those who supplied libraries, especially Adafruit!

