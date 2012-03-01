// LI200 Pyranometer
// Using INA122 op-amp & MCP3201 12 Bit ADC
#include <SPI.h>                      // SPI Library

unsigned int li_bit = 0;            // Sunlight bit count
float li_val = 0.0;                 // Sunlight value W/m^2
const int slavepin = 53;

void setup(){
  Serial.begin(9600);               // Begin serial moniter
  
  SPI.begin();                      // Start SPI 
  SPI.setBitOrder(MSBFIRST);        // Set SPI protocols
  SPI.setDataMode(SPI_MODE0);
  
  Serial.println("Type to Start");  // Wait for serial signal
  while(!Serial.available());  
}

void loop(){
  li_bit = mcp3201(slavepin);                 // Find licor digital value
  Serial.println(li_bit);           // Print
  delay(1000);                      // Delay
}

unsigned int mcp3201(const int pinnum){           // Function to interact MCP3201 ADC -- Pass Slave Select pin in
  word bitnum = 0;                   // Initialize return
  digitalWrite(pinnum, LOW);            // Start communication
  byte msb = SPI.transfer(0);     // Communicate
  byte lsb = SPI.transfer(0);
  digitalWrite(pinnum, HIGH);           // End communication
  bitnum = ((bitnum | msb) << 8) | lsb;      // Concatenate bytes into word
  bitnum = bitnum >> 1;               // Remove extraneous data
  bitnum = bitnum << 4;
  bitnum = bitnum >> 4;
  return(bitnum);                   // Return number of bits
}

