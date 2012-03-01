// LI200 Pyranometer
// Using INA122 op-amp & MCP3201 12 Bit ADC
#include <SPI.h>                      // SPI Library

unsigned int li_bit = 0;            // Sunlight bit count
float li_val = 0.0;                 // Sunlight value W/m^2

void setup(){
  Serial.begin(9600);               // Begin serial moniter
  
  SPI.begin();                      // Start SPI 
  SPI.setBitOrder(MSBFIRST);        // Set SPI protocols
  SPI.setDataMode(SPI_MODE0);
  
  Serial.println("Type to Start");  // Wait for serial signal
  while(!Serial.available());  
}

void loop(){
  li_bit = mcp3201();                 // Find licor digital value
  Serial.println(li_bit);           // Print
  delay(1000);                      // Delay
}

unsigned int mcp3201(void){           // Function to interact MCP3201 ADC
  word lival = 0;                   // Initialize return
  digitalWrite(SS, LOW);            // Start communication
  byte limsb = SPI.transfer(0);     // Communicate
  byte lilsb = SPI.transfer(0);
  digitalWrite(SS, HIGH);           // End communication
  lival = ((lival | limsb) << 8) | lilsb;      // Concatenate bytes into word
  lival = lival >> 1;               // Remove extraneous data
  lival = lival << 4;
  lival = lival >> 4;
  return(lival);                   // Return number of bits
}

