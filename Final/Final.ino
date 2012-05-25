// Sensor Station Final Code - Revision 3
// Nipun Gunawardena
/* The lower 5TM sensor data wire will be connected to digital pin 17 (Serial2 RX)
 and the upper 5TM sensor data wire will be connected to digital pin 15 (Serial3 RX)
 This will not be defined because it will automatically be set in the setup loop.
 All sensors can be run from 5 V Arduino pin.  This code is only compatible with the Arduino Mega.
 Remember to run two jumper wires from analog pins 4 and 5 to digital pins 20 and 21 respectively.
 On the data logging shield, jumper wires will be required from digital pins 8 and 9 to the 
 red LED and green LED respectively.
 */

#include <SD.h>						// SD card library
#include <Wire.h>					// I2C library
#include "RTClib.h"					// Real Time Clock (RTC) library
#include <Adafruit_BMP085.h>				// BMP085 Library
#include <OneWire.h>				// OneWire Library 
#include <DallasTemperature.h>		// Dallas Temperature Library; Requires <OneWire.h> to run
#include <SPI.h>
RTC_DS1307 RTC;						// Initialize RTC
File logfile;						// Initialize class called logfile
Adafruit_BMP085 bmp;					// Initialize class called bmp

// Pin Variables...
const int chipSelect = 10;			        // Pin needed for data logger
const int ftm_powerL = 24;          	                // Lower 5TM sensor power pin (White Wire)
                                                        // Lower 5TM corresponds to Serial2
const int ftm_powerU = 26;          	                // Upper 5TM sensor power pin (White Wire)
                                                        // Upper 5TM corresponds to Serial3 (Red Wire)
const int tn9_data = 2;				        // TN9 data pin (Green Wire)
const int tn9_clk = 3;    				// TN9 clock pin (White Wire)
const int tn9_action = 4;          			// TN9 action pin (Edgemost Black Wire)
const int green_led = 9;		        	// Datalogger green LED
const int red_led = 8;				        // Datalogger red LED
OneWire oneWire(38);					// Initialize OneWire Device on pin 23
// MISO = 50;                                           // SPI Pins as a reminder
// MOSI = 51;
// SCK = 52;
// SS = 53;
// SDA = 20;                                            // I2C Pins as a reminder
// SCL = 21;

// 5TM Variables...
double ftm_moisL = 0.0;  				// Lower 5TM moisture
double ftm_moisU = 0.0;     				// Upper 5TM moisture
double ftm_tempL = 0.0;     				// Lower 5TM temp
double ftm_tempU = 0.0;     				// Upper 5TM temp
int ftm_i = 0;					        // 5TM index counter

// TN9 Variables...
const int tn9_len = 5;					// Length of tn9 values array
volatile byte tn9_pos = 0;				// TN9 array position count
volatile byte tn9_rawval[5] = {
  0,0,0,0,0};              // TN9 array
byte tn9_n = 0;						// TN9 interrupt bit count
byte tn9_cbit = 0;					// TN9 current bit read
boolean tn9_irflag = false;				// TN9 flag to indicate IR reading made
boolean tn9_ambflag = false;				// TN9 flag to indicate ambient temp reading made
float tn9_ir = 0.0;					// TN9 IR temperature
float tn9_amb = 0.0;					// TN9 ambient temperature

// BMP085 Variables...
float bmp_temp = 0.0;				// BMP085 Temperature
float bmp_pres = 0.0;				// BMP085 Pressure
// If properly setup, there is a pressure command as well
// Dallas Temp Variables...
DallasTemperature dstemp(&oneWire);	// Initializes OneWire Device and Dallas Temperature Sensor
DeviceAddress dsaddress;			// Sets Address;
float ds_temp = 0.0;				// Dallas sensor temperature

// HIH4030 Variables...
unsigned int hih_raw = 0;					// HIH4030 Bit Value
float hih_hum = 0.0;                                              // HIH4030 Humidity
float hih_tchum = 0.0;                                          // HIH4030 Temperature Corrected Humidity

// Li200 Variables
unsigned int li_val = 0;				        // Word to hold 12 bit sunlight values
// !! CHANGES FOR EACH DIFFERENT LI200. SEE CERTIFICATE!!

// ADS7841 Control Codes
const byte li_code =  0b10010100;      // ADC channel 0 - Used by Li200
const byte hih_code = 0b11010100;     // ADC channel 1 - Used by HIH4030
const byte ch2_code = 0b10100100;     // ADC channel 2 - Currently unused
const byte ch3_code = 0b11100100;     // ADC channel 3 - Currently unused

// Miscellaneous Variables
unsigned long time_old = 0;
unsigned long time_dif = 0;




// SETUP //////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(1200);
  Serial2.begin(1200);		        // Lower 5TM
  Serial3.begin(1200);    		// Upper 5TM

  pinMode(chipSelect, OUTPUT);		// Pin initialization...
  pinMode(ftm_powerL, OUTPUT);
  pinMode(ftm_powerU, OUTPUT);
  pinMode(tn9_clk, INPUT);					
  pinMode(tn9_data, INPUT);
  pinMode(tn9_action, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  digitalWrite(tn9_clk, HIGH);
  digitalWrite(tn9_data, HIGH);
  digitalWrite(tn9_action, HIGH);
  digitalWrite(ftm_powerL, LOW);
  digitalWrite(ftm_powerU, LOW);
  digitalWrite(green_led, LOW);
  digitalWrite(red_led, LOW);

  Serial.println("Type any character to start");		// Wait for serial input to start
  //while (!Serial.available());
  delay(2000);
  Serial.print("Initializing SD card...");		        // Initialize Data Logger
  if (!SD.begin(chipSelect)) {  				// Card check...
    digitalWrite(red_led, HIGH);  		
    error("Card failed, or not present");
  }
  Serial.println("Card initialized.");
  char filename[] = "LOGGER00.CSV";			        // Create filename
  for (uint8_t i = 0; i < 100; i++) {      			// Indexes file every time program is restarted
    filename[6] = i/10 + '0';				        // !! Will stop at 99 !!
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      logfile = SD.open(filename, FILE_WRITE); 
      break;  
    }
  }
  if (! logfile) {						// File check
    digitalWrite(red_led, HIGH);
    error("Couldnt create file");               
  }
  Serial.print("Logging to: ");
  Serial.println(filename);
  Wire.begin();  						// Initialize RTC communication
  if (!RTC.begin()) {					        // RTC check
    digitalWrite(red_led, HIGH);
    error("RTC failed"); 
  }
  bmp.begin();								// Begin BMP085
  dstemp.begin();							// Begin dallas temp sensor
  dstemp.getAddress(dsaddress, 0);			// Get dallas temp sensor address
  dstemp.setResolution(dsaddress, 12);		// Set dallas temp sensor resolution
  SPI.begin();
//  SPI.setBitOrder(MSBFIRST);
//  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  digitalWrite(SS,HIGH);
  logfile.println("Millis,Month,Day,Year,Hour,Minute,Second,Dallas Amb,Rel Hum,Temp Corrected Rel Hum,Pressure,BMP Amb,IR,TN9 Amb,Soil Lower Temp,Soil Lower Mois,Soil Upper Temp,Soil Upper Mois,Sunlight");

  digitalWrite(green_led, HIGH);				// About to enter main loop confirmation
  delay(1000);
  digitalWrite(green_led, LOW);
  Serial.println("Starting");
  attachInterrupt(1,tn9Data,FALLING);  		                // Interrupt
  digitalWrite(tn9_action,LOW);				        // Make sensor start sending data
} //setup()





// LOOP ///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  DateTime now;
  if(tn9_pos == tn9_len && tn9_rawval[0] == 0x4C){		// If sensor has sent IR packet...
    tn9_ir = tn9Temp(tn9_rawval);				// Calculate temperature
    tn9_irflag = true;				        	// Indicate IR reading
    digitalWrite(tn9_action,LOW);				// Make sensor start sending data
  }

  if(tn9_pos == tn9_len && tn9_rawval[0] == 0x66){		// If sensor has sent ambient packet...
    tn9_amb = tn9Temp(tn9_rawval);				// Calculate temperature
    tn9_ambflag = true;					        // Indicate Ambient reading
    digitalWrite(tn9_action,LOW);				// Make sensor start sending data    
  }

  if(tn9_pos == tn9_len && tn9_rawval[0] == 0x53){		// If sensor has sent junk packet...
    digitalWrite(tn9_action,LOW);				// Make sensor start sending data   
  }

  if(tn9_irflag && tn9_ambflag){			        // If successful IR and Ambient reading...
    digitalWrite(tn9_action,HIGH);                  		// Make TN9 stop sending data. Ensure no Interrupts        	

    ftm_i = 0;
    digitalWrite(ftm_powerL,HIGH);                              // Excite lower 5TM...
    delay(200);
    char ftm_inputL[Serial2.available()];                       // Read lower 5TM Values
    while (Serial2.available()>0){ 
      ftm_inputL[ftm_i] = Serial2.read();
      ftm_i++;
    }

    ftm_i = 0;
    digitalWrite(ftm_powerU,HIGH);                              // Excite upper 5TM...
    delay(200);
    char ftm_inputU[Serial3.available()];                       // Read upper 5TM Values
    while (Serial3.available()>0){ 
      ftm_inputU[ftm_i] = Serial3.read();
      ftm_i++;
    }

    digitalWrite(ftm_powerL,LOW);				// Turn 5TM's off...
    digitalWrite(ftm_powerU,LOW);
    ftmParse(ftm_inputL,ftm_moisL,ftm_tempL);			// Parse 5TM's message...
    ftmParse(ftm_inputU,ftm_moisU,ftm_tempU);

    bmp_temp = bmp.readTemperature();			         // Measure BMP085 Temperature
    bmp_pres = bmp.readPressure();				// Measure Pressure

    dstemp.requestTemperatures();			        // Send command to get dallas temp sensor values
    ds_temp = dstemp.getTempC(dsaddress);	                // Read Temperature

    li_val = ads7841(li_code);					
    // PLACE BIT CONVERTING EQUATION HERE

    hih_raw = ads7841(hih_code);
    hih_hum = (float)hih_raw/25.3952 - 25.8065;
    hih_tchum = hih_hum/(1.0546 - 0.00216*ds_temp);


    now = RTC.now();						// Current time
    logfile.print(millis());					// Write to file...
    logfile.print(",");
    logfile.print(now.month(), DEC);
    logfile.print(",");
    logfile.print(now.day(), DEC);
    logfile.print(",");
    logfile.print(now.year(), DEC);
    logfile.print(",");
    logfile.print(now.hour(), DEC);
    logfile.print(",");
    logfile.print(now.minute(), DEC);
    logfile.print(",");
    logfile.print(now.second(), DEC);
    logfile.print(",");
    logfile.print(ds_temp,4);
    logfile.print(",");
    logfile.print(hih_hum);
    logfile.print(",");
    logfile.print(hih_tchum);
    logfile.print(",");
    logfile.print(bmp_pres);
    logfile.print(",");
    logfile.print(bmp_temp);
    logfile.print(",");
    logfile.print(tn9_ir);
    logfile.print(",");
    logfile.print(tn9_amb);
    logfile.print(",");
    logfile.print(ftm_tempL,2);
    logfile.print(",");
    logfile.print(ftm_moisL,5);
    logfile.print(",");
    logfile.print(ftm_tempU,2);
    logfile.print(",");
    logfile.print(ftm_moisU,5);
    logfile.print(",");
    logfile.println(li_val);
    logfile.flush();						// Save file !! NECESSARY !!


    digitalWrite(green_led, HIGH);				// Write confirmation
    delay(100);
    digitalWrite(green_led, LOW);

    time_dif = millis()-time_old;
    if(now.hour() == 0 && now.minute() == 0 && time_dif >= 86400000){   // If one day has passed...
      time_old = millis();                                         // Reset timers
      time_dif = 0;     
      char filename[] = "LOGGER00.CSV";                                                                       
      for (uint8_t i = 0; i < 100; i++) {                         // Check for existing filenames                                                     
        filename[6] = i/10 + '0';
        filename[7] = i%10 + '0';
        if (! SD.exists(filename)) {                              // If not existing, make new file
          logfile = SD.open(filename, FILE_WRITE); 
          break;                                                  // leave the loop
        }
      }
    }


    delay(8000);						// Wait for next reading
    tn9_irflag = false;						// Reset TN9 flags...
    tn9_ambflag = false;
    ftm_moisL = ftm_moisU = ftm_tempL = ftm_tempU = 0.0;	// Reset 5TM values
    digitalWrite(tn9_action,LOW);				// Make tn9 start sending data  
  }											

} //loop()




// FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void error(char *str){			                        // Used when initializing SD card... 
  Serial.print("error: ");
  Serial.println(str);
  while(1);
} //error()


void ftmParse(char input[], double &mois, double &temp){	// Used to parse 5TM input string...
  int values[3] = {
    0,0,0        };							// Moisture, checksum, temperature 
  int fieldIndex = 0;						// Values index count
  int check = 0;						// Stop byte position
  for(int j = 0; j < strlen(input); j++){    		        // Parse input string...
    if(input[j] >= '0' && input[j] <= '9' && fieldIndex < 3){
      values[fieldIndex] = (values[fieldIndex] * 10) + (input[j] - '0');      
    }
    else if(input[j] == ' '){
      fieldIndex++;
    }
    else{
      check = j; 
      break; 
    }
  }
  values[1] = int(input[strlen(input)-3]);			// Store sensor side checksum
  char crc = 0;                                			// Calculate arduino side checksum...
  for(int j = 0; j < check; j++){
    crc += input[j];   
  }
  crc = (crc + 0xD + 'x') % 64 + 32;
  mois = 0.0000043*pow(double(values[0])/50.0,3)-0.00055*pow(double(values[0])/50.0,2)+0.0292*(double(values[0])/50.0)-0.053; // Calculate moisture with topp equation
  if (values[2] <= 900){					// Calculate temperature...
    temp = (double(values[2])-400.0)/10.0;
  }
  if (values[2] > 900){
    temp = 900.0+5.0*(double(values[2])-900.0);
    temp = (temp - 400.0)/10.0; 
  }
  if( values[1] != crc){
    mois = 0.0;
    temp = -273.15; 
  }
} //ftm_parse()


void tn9Data(){							// TN9 Interrupt Function...
  tn9_cbit =  digitalRead(tn9_data);				// Read bit
  if(tn9_pos >= tn9_len) tn9_pos = 0;				// Keep index below 5
  tn9_rawval[tn9_pos] = (tn9_rawval[tn9_pos] << 1) | tn9_cbit; 	// Store to values
  tn9_n++;							// Increment bit count
  if(tn9_n == 8){		        			// Increment position count based on bits read in...
    tn9_pos++;
    tn9_n = 0; 
  }
  if(tn9_pos == tn9_len){ 					// If complete "packet" sent, stop sensor from sending 
    digitalWrite(tn9_action,HIGH);  		                // again until main loop allows it.
  }
} //tn9Data()


float tn9Temp(volatile byte tn9Values[]){		        // TN9 temperature calculation function...
  word tempword = 0;						// Initialize temperature isolation word
  float temperature = 0.0;					// Initialize return
  boolean crc = false;						// Initialize checksum
  int mcheck = (int)tn9Values[0] + (int)tn9Values[1] + (int)tn9Values[2];	// Checksum calculation
  int scheck = (int)tn9Values[3];				// sensor side checksum
  if(mcheck > 510) mcheck = mcheck - 512;		        // Handle sensor byte rollover
  if(mcheck > 255) mcheck = mcheck - 256;        		// Handle sensor byte rollover
  if(mcheck == scheck) crc = true;			        // Check checksum
  tempword = tempword | tn9Values[1];			        // Isolate temperature components...
  tempword = tempword << 8;
  tempword = tempword | tn9Values[2];
  if(crc){							// Calculate temperature if checksum valid...
    temperature = int(tempword)/16.0 - 273.15;	
  }
  else{
    temperature = -273.15;
  }
  return(temperature);						// Return
} //tn9Temp


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



