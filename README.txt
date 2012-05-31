README.txt
Arduino code for various sensors for the LEMS
While updates may be rare, this code is always a work in progress.


Includes code for following sensors:
LI200 Pyranometer with ADS7841 ADC (Though the circuit is more important)
BMP085 Pressure Sensor
DS18B20 Temperature Sensor
HIH4030 Humidity Sensor
DHT22 Temperature/Humidity Sensor
TN9 Infrared Temperature Sensor
Decagon 5TM Soil Moisture/Temperature Sensor
ADS7841 12-Bit ADC
All of the above combined


The aforementioned sensors will need the following libraries
Adafruit_BMP085 Library -- https://github.com/adafruit/Adafruit-BMP085-Library
DHT Library -- https://github.com/adafruit/DHT-sensor-library
OneWire Library -- http://www.pjrc.com/teensy/td_libs_OneWire.html
DallasTemperature Library -- http://milesburton.com/Dallas_Temperature_Control_Library
Streaming Library -- http://arduiniana.org/libraries/streaming/	  (Not absolutely necessary but sporadically used in the code)

The combined sensor code also uses Adafruit's datalogger shield
The datalogger shield needs the following libraries:
SD Library -- https://github.com/adafruit/SD
RTC Library -- https://github.com/adafruit/RTClib


Many thanks to those who supplied libraries, especially Adafruit!

