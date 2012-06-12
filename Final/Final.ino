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
#define TEMPERATURE 1
#define UPPERSOIL 0                         // Serial3
#define LOWERSOIL 0                         // Serial2
#define INFRARED 1			
#define HUMIDITY 1
#define SUNLIGHT 1		// !!! REMEMBER TO INCLUDE CORRECT CALIBRATION CONSTANT (Line 35) !!!


#if SUNLIGHT
	unsigned int li_val = 0;			    // Word to hold 12 bit sunlight values
	const float cal_const = 91.96E-6/1000;  // Licor Calibration Constant. Units of (Amps/(W/m^2))
	const float cal_resistor = 44090;		// Exact Resistor Value used by Op-Amp
	float sunlight = 0.0;					// Converted Value
#endif


#include <SD.h>								// SD card library
#include <Wire.h>							// I2C library
#include "RTClib.h"							// Real Time Clock (RTC) library
#include <SPI.h>							// SPI library
RTC_DS1307 RTC;								// Initialize RTC
File logfile;								// Initialize class called logfile
const int chipSelect = 10;					// Pin needed for data logger
const int green_led = 9;		    		// Datalogger green LED
const int red_led = 8;						// Datalogger red LED		
unsigned long time_old = 0;		 			// Variables used for timing controls
unsigned long time_dif = 0;


// ADS7841 Control Codes
// The channel for the ADS7841 is chosen by these codes.  They get passed into the ads7841()
// function.
const byte li_code =  0b10010100;     		// ADC channel 0 - Used by Li200
const byte hih_code = 0b11010100;     		// ADC channel 1 - Used by HIH4030
const byte ch2_code = 0b10100100;     		// ADC channel 2 - Currently unused
const byte ch3_code = 0b11100100;     		// ADC channel 3 - Currently unused


#if PRESSURE
	#include <Adafruit_BMP085.h>			// BMP085 Library
	Adafruit_BMP085 bmp;					// Initialize class called bmp
	float bmp_temp = 0.0;					// BMP085 Temperature
	float bmp_pres = 0.0;					// BMP085 Pressure
#endif


#if TEMPERATURE
	#include <OneWire.h>					// OneWire Library 
	#include <DallasTemperature.h>			// Dallas Temperature Library; Requires <OneWire.h> to run
	OneWire oneWire(38);					// Initialize OneWire Device on pin 23
	DallasTemperature dstemp(&oneWire);		// Initializes OneWire Device and Dallas Temperature Sensor
	DeviceAddress dsaddress;				// Sets Address;
	float ds_temp = 0.0;					// Dallas sensor temperature
#endif


int ftm_i = 0;					            // 5TM index counter
// The above line is initialized only once to avoid double initialization    	                                    	


#if UPPERSOIL
	const int ftm_powerU = 26;          	// Upper 5TM sensor power pin (White Wire)
	// Upper 5TM corresponds to Serial3 (Red Wire)
	double ftm_moisU = 0.0;     			// Upper 5TM moisture
	double ftm_tempU = 0.0;     			// Upper 5TM temp
#endif


#if LOWERSOIL
	const int ftm_powerL = 24;          	// Lower 5TM sensor power pin (White Wire)
	// Lower 5TM corresponds to Serial2
	double ftm_moisL = 0.0;  				// Lower 5TM moisture
	double ftm_tempL = 0.0;     			// Lower 5TM temp
#endif


#if INFRARED
	const int tn9_data = 2;						// TN9 data pin (Green Wire)
	const int tn9_clk = 3;						// TN9 clock pin (White Wire)
	const int tn9_action = 4;					// TN9 action pin (Edgemost Black Wire)
	const int tn9_len = 5;						// Length of tn9 values array
	volatile byte tn9_pos = 0;					// TN9 array position count
	volatile byte tn9_rawval[5] = {0,0,0,0,0};	// TN9 array
	byte tn9_n = 0;								// TN9 interrupt bit count
	byte tn9_cbit = 0;							// TN9 current bit read
	float tn9_ir = 0.0;							// TN9 IR temperature
	float tn9_amb = 0.0;						// TN9 ambient temperature
#endif


boolean tn9_irflag = false;					// TN9 flag to indicate IR reading made
boolean tn9_ambflag = false;				// TN9 flag to indicate ambient temp reading made
// The variables defined in the preceding two lines are required regardless of TN9 presence


#if HUMIDITY
	unsigned int hih_raw = 0;				// HIH4030 Bit Value
	float hih_hum = 0.0;                    // HIH4030 Humidity
	float hih_tchum = 0.0;                  // HIH4030 Temperature Corrected Humidity
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
	
	DateTime now;								// Class to store RTC values
	now = RTC.now();							// Store date & time
	char filename[13];							// Filename initialization
	filename[0] = now.month() / 10 + '0';		// Parse Month
	filename[1] = now.month() % 10 + '0';
	filename[2] = now.day() / 10 + '0';			// Parse Day
	filename[3] = now.day() % 10 + '0';
	filename[4] = (now.year() % 100) / 10 + '0';	// Parse Year
	filename[5] = now.year() % 10 + '0';
	filename[6] = '-';							// Version Number
	filename[7] = '-';
	filename[8] = '.';
	filename[9] = 'c';							// Extension
	filename[10] = 's';
	filename[11] = 'v';
	filename[12] = '\0';
	//Serial.println(filename);


	//Serial.print("Initializing SD card...");	// Initialize Data Logger
	if (!SD.begin(chipSelect)) {  				// Card check...
		digitalWrite(red_led, HIGH);  		
		error("Card failed, or not present");
	}
	//Serial.println("Card initialized.");

	
	for (uint8_t i = 0; i < 100; i++){      		// Adds prefix to filename in case Arduino started twice on same date
		filename[6] = i/10 + '0';					// !! Will stop at 99 !! If 99 is hit, will probably lead to unknown results
		filename[7] = i%10 + '0';
		if (!SD.exists(filename)) {
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
		dstemp.begin();							// Begin dallas temp sensor
		dstemp.getAddress(dsaddress, 0);		// Get dallas temp sensor address
		dstemp.setResolution(dsaddress, 12);	// Set dallas temp sensor resolution
		logfile.print(",Dallas Amb");
	#endif
	
	#if HUMIDITY
		logfile.print(",Rel Hum,Temp Corrected Rel Hum");
	#endif
	
	#if PRESSURE   
		bmp.begin();							// Begin BMP085
		logfile.print(",Pressure,BMP Amb");		
	#endif	
	
	#if INFRARED								// TN9 Thermometer
		pinMode(tn9_clk, INPUT);					
		pinMode(tn9_data, INPUT);
		pinMode(tn9_action, OUTPUT);
		digitalWrite(tn9_clk, HIGH);
		digitalWrite(tn9_data, HIGH);
		digitalWrite(tn9_action, HIGH);
		logfile.print(",IR,TN9 Amb");
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
	
	logfile.println();  
	logfile.flush();  
	
	digitalWrite(green_led, HIGH);				// About to enter main loop confirmation
	delay(1500);									// Everything's worked up to this point if green LED is on
	digitalWrite(green_led, LOW);
	//Serial.println("Starting");
	
	#if INFRARED
		attachInterrupt(1,tn9Data,FALLING);  		                // Interrupt
		digitalWrite(tn9_action,LOW);				        // Make sensor start sending data
	#endif
} //setup()






// LOOP ///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
	DateTime now;
	
	
	#if INFRARED
		if(tn9_pos == tn9_len && tn9_rawval[0] == 0x4C){		// If sensor has sent IR packet...
			tn9_ir = tn9Temp(tn9_rawval);						// Calculate temperature
			tn9_irflag = true;				        			// Indicate IR reading
			digitalWrite(tn9_action,LOW);						// Make sensor start sending data
		}
	
		if(tn9_pos == tn9_len && tn9_rawval[0] == 0x66){		// If sensor has sent ambient packet...
			tn9_amb = tn9Temp(tn9_rawval);						// Calculate temperature
			tn9_ambflag = true;					        		// Indicate Ambient reading
			digitalWrite(tn9_action,LOW);						// Make sensor start sending data    
		}
	
		if(tn9_pos == tn9_len && tn9_rawval[0] == 0x53){		// If sensor has sent junk packet...
			digitalWrite(tn9_action,LOW);						// Make sensor start sending data   
		}
	#else
		tn9_irflag = tn9_ambflag = true;						// If TN9 is not present, ensure flags are true so the rest of the sensors still activate
	#endif
	
	
	if(tn9_irflag && tn9_ambflag){			        			// If successful IR and Ambient reading (will always be true if TN9 not present)
		#if INFRARED
			digitalWrite(tn9_action,HIGH);                  	// Make TN9 stop sending data. Ensure no Interrupts        	
		#endif
	
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
			dstemp.requestTemperatures();			                // Send command to get dallas temp sensor values
			ds_temp = dstemp.getTempC(dsaddress);	                // Read Temperature
			logfile.print(",");										// Write Temperature to file
			logfile.print(ds_temp,4);
		#endif
	
		#if HUMIDITY					
			hih_raw = ads7841(hih_code);							// Get Humidity from ADS7841 
			hih_hum = (float)hih_raw/25.3952 - 25.8065;				// Calculate humidity
			hih_tchum = hih_hum/(1.0546 - 0.00216*ds_temp);			// Account for temperature
			logfile.print(",");										// Write Humidity to file
			logfile.print(hih_hum);
			logfile.print(",");
			logfile.print(hih_tchum);
		#endif
	
		#if PRESSURE
			bmp_temp = bmp.readTemperature();			            // Measure BMP085 Temperature
			bmp_pres = bmp.readPressure();				         	// Measure Pressure
			logfile.print(",");										// Write Pressure to file
			logfile.print(bmp_pres);
			logfile.print(",");
			logfile.print(bmp_temp);
		#endif
	
		#if INFRARED												
			logfile.print(",");										// Write Infrared temp to file
			logfile.print(tn9_ir);
			logfile.print(",");
			logfile.print(tn9_amb);
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
			logfile.print(",");										// Write lower 5TM to file			
			logfile.print(ftm_tempL,2);
			logfile.print(",");
			logfile.print(ftm_moisL,5);
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
			logfile.print(",");										// Write upper 5TM to file
			logfile.print(ftm_tempU,2);
			logfile.print(",");
			logfile.print(ftm_moisU,5);
		#endif
		
		#if SUNLIGHT
			li_val = ads7841(li_code);								// Get sunlight from ads7841
			sunlight = (4.5/4095)*(1/cal_resistor)*(1/cal_const)*li_val;  // Convert to W/m^2
			logfile.print(",");										// Write sunlight to file
			logfile.print(sunlight);
		#endif
		
		logfile.println();
		logfile.flush();											// Save file !! NECESSARY !!
		digitalWrite(green_led, HIGH);								// Write confirmation LED flash
		delay(150);
		digitalWrite(green_led, LOW);
	
		time_dif = millis()-time_old;
		if(now.hour() == 0 && now.minute() == 0 && time_dif >= 10000){   // If new day has started and sketch started before 23:50...
			time_old = millis();                                  	   // Reset timers
			time_dif = 0;     
			char filename[13];
			filename[0] = now.month() / 10 + '0';
			filename[1] = now.month() % 10 + '0';
			filename[2] = now.day() / 10 + '0';
			filename[3] = now.day() % 10 + '0';
			filename[4] = (now.year() % 100) / 10 + '0';
			filename[5] = now.year() % 10 + '0';
			filename[6] = '-';
			filename[7] = '-';
			filename[8] = '.';
			filename[9] = 'c';
			filename[10] = 's';
			filename[11] = 'v';
			filename[12] = '\0';                                                                            
			for (uint8_t i = 0; i < 100; i++){                   	// Check for existing filenames                                                     
				filename[6] = i/10 + '0';
				filename[7] = i%10 + '0';
				if (! SD.exists(filename)){                         // If not existing, make new file
			  		logfile = SD.open(filename, FILE_WRITE); 
			  		break;                                          // leave the loop
				}
		  	}
		}
		delay(7500);												// Wait for next reading
		
		#if LOWERSOIL
			ftm_moisL = ftm_tempL = 0.0; 							// Reset lower 5TM values
		#endif    
		
		#if UPPERSOIL
			ftm_moisU = ftm_tempU = 0.0;							// Reset upper 5TM values
		#endif    
		
		#if INFRARED    
			tn9_irflag = false;										// Reset TN9 flags...
			tn9_ambflag = false;
			digitalWrite(tn9_action,LOW);							// Make tn9 start sending data if present  
		#endif    
	}											
} //loop()






// FUNCTIONS //////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

void error(char *str){			                        	// Used when initializing SD card... 
	Serial.print("error: ");
	Serial.println(str);
	while(1);
} //error()


void ftmParse(char input[], double &mois, double &temp){	// Used to parse 5TM input string...
	int values[3] = {0,0,0};							    // Moisture, checksum, temperature 
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


#if INFRARED
	void tn9Data(){							            	// TN9 Interrupt Function...
		tn9_cbit =  digitalRead(tn9_data);					// Read bit
		if(tn9_pos >= tn9_len) tn9_pos = 0;					// Keep index below 5
		tn9_rawval[tn9_pos] = (tn9_rawval[tn9_pos] << 1) | tn9_cbit; 	// Store to values
		tn9_n++;							                // Increment bit count
		if(tn9_n == 8){		        			        	// Increment position count based on bits read in...
			tn9_pos++;
			tn9_n = 0; 
		}
		if(tn9_pos == tn9_len){ 					       // If complete "packet" sent, stop sensor from sending 
			digitalWrite(tn9_action,HIGH);  		       // again until main loop allows it.
		}
	} //tn9Data()
	
	
	float tn9Temp(volatile byte tn9Values[]){		        // TN9 temperature calculation function...
		word tempword = 0;									// Initialize temperature isolation word
		float temperature = 0.0;							// Initialize return
		boolean crc = false;								// Initialize checksum
		int mcheck = (int)tn9Values[0] + (int)tn9Values[1] + (int)tn9Values[2];	// Checksum calculation
		int scheck = (int)tn9Values[3];						// sensor side checksum
		if(mcheck > 510) mcheck = mcheck - 512;		        // Handle sensor byte rollover
		if(mcheck > 255) mcheck = mcheck - 256;        		// Handle sensor byte rollover
		if(mcheck == scheck) crc = true;			        // Check checksum
		tempword = tempword | tn9Values[1];			        // Isolate temperature components...
		tempword = tempword << 8;
		tempword = tempword | tn9Values[2];
		if(crc){											// Calculate temperature if checksum valid...
			temperature = int(tempword)/16.0 - 273.15;	
		}
		else{
			temperature = -273.15;
		}
		return(temperature);								// Return
	} //tn9Temp
#endif


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


