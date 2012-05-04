#include <SPI.h>                      // SPI Library
#include <Streaming.h>

// Following codes correspond to the codes that the ADS7841 expects to see for it's control byte
// Please refer to data sheet
const byte li_code =  0b10010100;      // ADC channel 0 - Used by Li200
const byte hih_code = 0b11010100;     // ADC channel 1 - Used by HIH4030
const byte ch2_code = 0b10100100;     // ADC channel 2 - Currently unused
const byte ch3_code = 0b11100100;     // ADC channel 3 - Currently unused

void setup(){
  Serial.begin(9600);                 // Serial Begin
  Serial.println("Type to start...");
  while(!Serial.available()); 

  SPI.begin();                      // Start SPI
  digitalWrite(SS,HIGH);
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  Serial << "Sunlight_Bits" << " Humidity_Bits" << endl;
}


void loop(){
  unsigned int li_bit;            // Sunlight bit count
  unsigned int hih_bit;           // Humidity bit count

  li_bit = ads7841(li_code);
  //hih_bit = ads7841(hih_code);  // Uncomment to display humidity value
  Serial << li_bit << endl;
  delay(100);                    
}


unsigned int ads7841(const byte control){    // Function to read ADS7841
  int bitnum;                                // Return value
  digitalWrite(SS,LOW);                      // Activate ADS7841
  SPI.transfer(control);                     // Transfer control byte
  byte msb = SPI.transfer(0);                // Read MSB & LSB
  byte lsb = SPI.transfer(0);
  digitalWrite(SS,HIGH);                     // Deactivate ADS7841
  msb = msb & 0x7F;                          // Isolate readings and form final reading
  lsb = lsb >> 3;
  bitnum = (word(msb) << 5) | lsb;
  return bitnum;                             // Return
}

