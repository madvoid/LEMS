// Sensor Station Final Code
// https://github.com/madvoid/LEMS/blob/master/Final/Final.ino
// Nipun Gunawardena
/* The lower 5TM sensor data wire will be connected to digital pin 17 (Serial2 RX)
 and the upper 5TM sensor data wire will be connected to digital pin 15 (Serial3 RX)
 This will not be defined because it will automatically be set in the setup loop.
 All sensors can be run from 5 V Arduino pin.  This code is only compatible with the Arduino Mega.
 Remember to run two jumper wires from analog pins 4 and 5 to digital pins 20 and 21 respectively.
 On the data logging shield, jumper wires will be required from digital pins 8 and 9 to the 
 red LED and green LED respectively.  Lines 22-28 have a list of preprocessor definitions.  Make sure
 each is set to 1 if being used or 0 if not being used.  These instructions will be updated if
 the custom shield works.  For more information about the code itself, please go to the end
 of the document.
 */

// MISO = 50;                               // SPI Pins as a reminder
// MOSI = 51;
// SCK = 52;
// SS = 53;
// SDA = 20;                                // I2C Pins as a reminder
// SCL = 21;

// !! THE HUMIDITY SENSOR REQUIRES A TEMPERATURE SENSOR FOR TEMPERATURE CORRECTION !!
// Replace temp component in line 320 with desired temp value (DS18B20 Recommended)
#define PRESSURE 1
#define TEMPERATURE 1  						// If temperature is included, so is humidity.  Make sure both are set to 1
#define UPPERSOIL 0                         // Serial3
#define LOWERSOIL 0                         // Serial2
#define INFRARED 1		
#define HUMIDITY 1
#define SUNLIGHT 0		// !!! REMEMBER TO INCLUDE CORRECT CALIBRATION CONSTANT AND RESISTOR VALUE (Line 36-37) !!!
#define WIND 1

#if SUNLIGHT
unsigned int li_val = 0;			    // Word to hold 12 bit sunlight values
const float cal_const = 93.40E-6/1000;  // Licor Calibration Constant. Units of (Amps/(W/m^2))
const float cal_resistor = 44300;		// Exact Resistor Value used by Op-Amp
float sunlight = 0.0;					// Converted Value
#endif


#include <SD.h>								// SD card library
#include <Wire.h>							// I2C library
#include <RTClib.h>							// Real Time Clock (RTC) library
#include <RTC_DS1307.h>
#include <SPI.h>							// SPI library
RTC_DS1307 RTC;								// Initialize RTC
File logfile;								// Initialize class called logfile
const int chipSelect = 10;					// Pin needed for data logger
const int green_led = 9;		    		// Datalogger green LED
const int red_led = 8;						// Datalogger red LED		
unsigned long time_old = 0;		 			// Variables used for timing controls
unsigned long time_dif = 0;
void error(char *str);                              // Error function prototype
char filename[] = "LOG_P_00.CSV";		    // !!! CHANGE IDENTIFICATION CODE FOR ALL CODES AND BELOW !!! 


// ADS7841 Control Codes
// The channel for the ADS7841 is chosen by these codes.  They get passed into the ads7841()
// function.
const byte li_code =  0b10010100;     		// ADC channel 0 - Used by Li200
const byte ch1_code = 0b11010100;     		// ADC channel 1 - Currently unused
const byte ch2_code = 0b10100100;     		// ADC channel 2 - Currently unused
const byte ch3_code = 0b11100100;     		// ADC channel 3 - Currently unused


#if PRESSURE
#include <Adafruit_BMP085.h>			// BMP085 Library
Adafruit_BMP085 bmp;					// Initialize class called bmp
float bmp_temp = 0.0;					// BMP085 Temperature
float bmp_pres = 0.0;					// BMP085 Pressure
#endif


#if TEMPERATURE
#include <SHT1x.h>
SHT1x sht1x(18, 19);			        // dataPin = 18 = TX1, clockPin = 19 = RX1
float sht_temp = 0.0;
#endif


int ftm_i = 0;					            // 5TM index counter
void ftmParse(char input[], double &mois, double &temp);    // Initialize ftmParse function
// The above line is initialized only once to avoid double initialization    	                                    	


#if UPPERSOIL
const int ftm_powerU = 7;	          	// Upper 5TM sensor power pin 
// Upper 5TM corresponds to Serial3 
double ftm_moisU = 0.0;     			// Upper 5TM moisture
double ftm_tempU = 0.0;     			// Upper 5TM temp
#endif


#if LOWERSOIL
const int ftm_powerL = 6;	          	// Lower 5TM sensor power pin 
// Lower 5TM corresponds to Serial2
double ftm_moisL = 0.0;  				// Lower 5TM moisture
double ftm_tempL = 0.0;     			// Lower 5TM temp
#endif


#if INFRARED
#include <Adafruit_MLX90614.h>  // Infrared Temp Library    https://github.com/adafruit/Adafruit-MLX90614-Library
float mlxIR;                // IR values from MLX90614
float mlxAmb;               // Ambient temp values from MLX90614
Adafruit_MLX90614 mlx = Adafruit_MLX90614();  // MLX90614 Class
#endif


boolean tn9_irflag = false;					// TN9 flag to indicate IR reading made
boolean tn9_ambflag = false;				// TN9 flag to indicate ambient temp reading made
// The variables defined in the preceding two lines are required regardless of TN9 presence


#if HUMIDITY
float sht_hum = 0.0;
#endif


#if WIND
const int windDirPin = A0;
unsigned long startTime;
volatile unsigned long windCount;
void counter();
#endif








// SETUP //////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(9600);							// For Serial output to computer, uncomment Serial.foo() lines			
  pinMode(chipSelect, OUTPUT);				// Data Logging Shield Chip Select
  pinMode(green_led, OUTPUT);
  pinMode(red_led, OUTPUT);
  digitalWrite(green_led, LOW);
  digitalWrite(red_led, LOW);


  //Serial.println("Type any character to start");		// Wait for serial input to start
  //while (!Serial.available());
  delay(2000);								// Delay so operator has time to look


  Wire.begin();  								// Initialize RTC communication
  if (!RTC.begin()) {							// RTC check
    digitalWrite(red_led, HIGH);
    error("RTC failed"); 
  }

  // RTC.adjust(DateTime(__DATE__, __TIME__));	// Set DS1307 to time of compilation
  // Do not uncomment above line unless you know what you are doing! It is used to set
  // RTC Time.



  //Serial.print("Initializing SD card...");	// Initialize Data Logger
  if (!SD.begin(chipSelect)) {  				// Card check...
    digitalWrite(red_led, HIGH);  		
    error("Card failed, or not present");
  }
  //Serial.println("Card initialized.");


  for (uint8_t i = 0; i < 100; i++) {            // Indexes file every time program is restarted	
    filename[6] = i/10 + '0';                  // !! Will stop at 99 !!	
    filename[7] = i%10 + '0';	
    if (! SD.exists(filename)) {	
      logfile = SD.open(filename, FILE_WRITE); 	
      break;  	
    }	
  }
  if (!logfile){								// File successfully opened check
    digitalWrite(red_led, HIGH);	
    error("Couldnt create file");               
  }
  logfile.print("Millis,Month,Day,Year,Hour,Minute,Second");	// The following logfile.print() functions collectively print the header
  //Serial.print("Logging to: ");
  //Serial.println(filename);

  SPI.begin();								// Begin SPI for ADS7841
  //  SPI.setBitOrder(MSBFIRST);
  //  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  digitalWrite(SS,HIGH);

#if TEMPERATURE
  logfile.print(",SHT Amb");
#endif

#if HUMIDITY
  logfile.print(",Rel Hum");
#endif

#if PRESSURE   
  bmp.begin();							// Begin BMP085
  logfile.print(",Pressure,BMP Amb");		
#endif	

#if INFRARED								// TN9 Thermometer
  mlx.begin();
  logfile.print(",MLX_IR,MLX_Amb");
#endif

#if LOWERSOIL
  Serial2.begin(1200);		    		// Lower 5TM
  pinMode(ftm_powerL, OUTPUT);
  digitalWrite(ftm_powerL, LOW);
  logfile.print(",Soil Lower Temp,Soil Lower Mois");	
#endif

#if UPPERSOIL
  Serial3.begin(1200);    				// Upper 5TM
  pinMode(ftm_powerU, OUTPUT);
  digitalWrite(ftm_powerU, LOW);
  logfile.print(",Soil Upper Temp,Soil Upper Mois");
#endif  

#if SUNLIGHT
  logfile.print(",Sunlight");
#endif

#if WIND
  pinMode(2, INPUT_PULLUP);
  logfile.print(",WindDir,WindSpd");
#endif

  logfile.println();  
  logfile.flush();  

  digitalWrite(green_led, HIGH);				// About to enter main loop confirmation
  delay(1500);									// Everything's worked up to this point if green LED is on
  digitalWrite(green_led, LOW);
  //Serial.println("Starting");

} //setup()






// LOOP ///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  DateTime now;





  now = RTC.now();						    			// Current time
  logfile.print(millis());								// Write Date & Time to file
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

#if TEMPERATURE
  sht_temp = sht1x.readTemperatureC();					// Get temperature from SHT15
  logfile.print(",");										// Write Temperature to file
  logfile.print(sht_temp,4);
#endif

#if HUMIDITY					
  sht_hum = sht1x.readHumidity();							// Get humidity from SHT15
  logfile.print(",");										// Write Humidity to file
  logfile.print(sht_hum);
#endif

#if PRESSURE
  bmp_temp = bmp.readTemperature();			            // Measure BMP085 Temperature
  bmp_pres = bmp.readPressure();				         	// Measure Pressure
  logfile.print(",");										// Write Pressure to file
  logfile.print(bmp_pres);
  logfile.print(",");
  logfile.print(bmp_temp);
#endif

#if INFRARED												// Write Infrared temp to file
  mlxAmb = mlx.readAmbientTempC();
  mlxIR = mlx.readObjectTempC();
  logfile.print(",");
  logfile.print(mlxIR);
  logfile.print(",");
  logfile.print(mlxAmb);
#endif    

#if LOWERSOIL
  ftm_i = 0;
  digitalWrite(ftm_powerL,HIGH);                          // Excite lower 5TM...
  delay(200);												// Wait for readings
  char ftm_inputL[Serial2.available()];                   // Read lower 5TM Values
  while (Serial2.available()>0){ 								
    ftm_inputL[ftm_i] = Serial2.read();
    ftm_i++;
  }
  digitalWrite(ftm_powerL,LOW);							// Turn 5TM off
  ftmParse(ftm_inputL,ftm_moisL,ftm_tempL);				// Parse 5TM's message...
  if (ftm_tempL > -273.0){
    logfile.print(",");										// Write lower 5TM to file			
    logfile.print(ftm_tempL,2);
    logfile.print(",");
    logfile.print(ftm_moisL,5);
  }
  else{
    logfile.print(",");
    logfile.print("NaN");
    logfile.print(",");
    logfile.print("NaN");
  }
#endif

#if UPPERSOIL
  ftm_i = 0;
  digitalWrite(ftm_powerU,HIGH);                          // Excite upper 5TM...
  delay(200);												// Wait for readings
  char ftm_inputU[Serial3.available()];                   // Read upper 5TM Values
  while (Serial3.available()>0){ 
    ftm_inputU[ftm_i] = Serial3.read();
    ftm_i++;
  }
  digitalWrite(ftm_powerU,LOW);							// Turn 5TM off
  ftmParse(ftm_inputU,ftm_moisU,ftm_tempU);				// Parse 5TM's message...
  if (ftm_tempU > -273.0){
    logfile.print(",");										// Write lower 5TM to file			
    logfile.print(ftm_tempU,2);
    logfile.print(",");
    logfile.print(ftm_moisU,5);
  }
  else{
    logfile.print(",");
    logfile.print("NaN");
    logfile.print(",");
    logfile.print("NaN");
  }
#endif

#if SUNLIGHT
  li_val = ads7841(li_code);								// Get sunlight from ads7841
  sunlight = (4.5/4095)*(1/cal_resistor)*(1/cal_const)*li_val;  // Convert to W/m^2
  logfile.print(",");										// Write sunlight to file
  logfile.print(sunlight);
#endif

#if WIND
  logfile.print(",");
  float wDir = (float)analogRead(windDirPin);
  wDir = (wDir) * (360.0) / (1024.0);  // Map from 0-1024 to 0-360
  logfile.print(wDir);
  startTime = millis();
  attachInterrupt(0, counter, RISING);
  while(millis() - startTime < 5000){
  }
  detachInterrupt(0);
  float wSpd = windCount*(2.25/5.0);  // Convert to mph
  wSpd = 0.447*wSpd;  // Convert to m/s
  logfile.print(",");
  logfile.print(wSpd);
#endif

  logfile.println();
  logfile.flush();											// Save file !! NECESSARY !!
  digitalWrite(green_led, HIGH);								// Write confirmation LED flash
  delay(150);
  digitalWrite(green_led, LOW);

  delay(2500);												// Wait for next reading

#if LOWERSOIL
  ftm_moisL = ftm_tempL = 0.0; 							// Reset lower 5TM values
#endif    

#if UPPERSOIL
  ftm_moisU = ftm_tempU = 0.0;							// Reset upper 5TM values
#endif

#if WIND
  windCount = 0;
#endif

} //loop()






// FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void error(char *str){			                        	// Used when initializing SD card... 
  Serial.print("error: ");
  Serial.println(str);
  while(1);
} //error()


void ftmParse(char input[], double &mois, double &temp){	// Used to parse 5TM input string...
  int values[3] = {
    0,0,0  };							    // Moisture, checksum, temperature 
  int fieldIndex = 0;						                // Values index count
  int check = 0;						                    // Stop byte position
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
  values[1] = int(input[strlen(input)-3]);			    // Store sensor side checksum
  char crc = 0;                                			// Calculate arduino side checksum...
  for(int j = 0; j < check; j++){
    crc += input[j];   
  }
  crc = (crc + 0xD + 'x') % 64 + 32;
  mois = 0.0000043*pow(double(values[0])/50.0,3)-0.00055*pow(double(values[0])/50.0,2)+0.0292*(double(values[0])/50.0)-0.053; // Calculate moisture with topp equation
  if (values[2] <= 900){					            	// Calculate temperature...
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



unsigned int ads7841(const byte control){    				// Function to read ADS7841
  int bitnum;                                				// Return value
  digitalWrite(SS,LOW);                      				// Activate ADS7841
  SPI.transfer(control);                     				// Transfer control byte
  byte msb = SPI.transfer(0);                				// Read MSB & LSB
  byte lsb = SPI.transfer(0);
  digitalWrite(SS,HIGH);                     				// Deactivate ADS7841
  msb = msb & 0x7F;                          				// Isolate readings and form final reading
  lsb = lsb >> 3;
  bitnum = (word(msb) << 5) | lsb;
  return bitnum;                             				// Return
}

#if WIND
void counter(){
  windCount++;  // Increment anemometer count
}
#endif




// CODE INFO //////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
/* The main sensor this code depends on is a TN9 infrared temperature sensor.  This is 
 because the TN9 has a very strange communication method that needs to bit-banged.  It sends 4 packets
 at 40 bits long, and the Arduino uses one of its external interrupts to receive the message
 (see function tn9data()).  The 4 packets are sent in the following order: 
 infrared, ambient, infrared, junk.  In the main loop, it checks to see whether an infrared
 packet and a ambient packet has been received and saved.  Once it has, it jumps into an
 if block where all the other sensors are polled.  All the other sensors are self-explanatory
 or use libraries to function.  Finally, the entire file is peppered with preprocessor directives
 that allow us to mix and match sensors without having to worry about the code failing. 
 */



