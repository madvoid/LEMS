// TN9 Infrared Temperature Sensor

byte n = 0;						        // Interrupt Bit Count				
volatile byte pos = 0;						// Values Position Count
volatile byte values[5] = {0,0,0,0,0};		                // Values to be stored by sensor
byte cbit = 0;						        // Current bit read in

boolean irFlag = false;						// Flag to indicate IR reading has been made
boolean ambFlag = false;					// Flag to indicate ambient temp reading has been made


byte irValues[5] = {0,0,0,0,0};		          	        // Variable to store IR reading bytes
byte ambValues[5] = {0,0,0,0,0};			        // Variable to store Ambient reading bytes

const int len = 5;					        // Length of values array
const int clkPin = 3;						// Pins
const int dataPin = 2;
const int actionPin = 4;

float irtemp = 0.0;                                             // Infrared Temperature
float ambtemp = 0.0;                                            // Ambient Temperature

void setup(){
  Serial.begin(9600);						

  pinMode(clkPin, INPUT);					// Initialize pins
  pinMode(dataPin, INPUT);
  pinMode(actionPin, OUTPUT);
  digitalWrite(clkPin, HIGH);
  digitalWrite(dataPin, HIGH);
  digitalWrite(actionPin, HIGH);

  Serial.println("Type to Start...");		                // Wait for input to start
  while(!Serial.available());
  Serial.println("Starting...");
  Serial.println("IR (C), Ambient (C), Time Since Start (ms)");

  attachInterrupt(1,tn9Data,FALLING);  		                // Attach Interrupt
  digitalWrite(actionPin,LOW);			  	        // Make sensor start sending data (LOW == send, HIGH == no send)
}

void loop(){

  if(pos == len && values[0] == 0x4C){		        // If sensor has sent IR packet...
    irtemp = tn9Temp(values);                           // Calculate IR temp
    irFlag = true;				        // Indicate IR reading
    digitalWrite(actionPin,LOW);			// Make sensor start sending data again
  }

  if(pos == len && values[0] == 0x66){		        // If sensor has sent ambient packet...
    ambtemp = tn9Temp(values);                          // Calculate Ambient temp
    ambFlag = true;				        // Indicate Ambient reading
    digitalWrite(actionPin,LOW);			// Make sensor start sending data    
  }

  if(pos == len && values[0] == 0x53){		        // If sensor has sent junk packet
    digitalWrite(actionPin,LOW);			// Make sensor start sending data   
  }

  if(irFlag && ambFlag){				// If successful IR and Ambient reading...
    digitalWrite(actionPin,HIGH);			// Make sensor stop sending data.  Because Timing is weird, I want to ensure the interrupts do not happen during this section.   
    Serial.print(millis());                           // Print time for logging purposes
    Serial.print(",");
    Serial.print(irtemp);
    Serial.print(",");
    Serial.println(ambtemp);
    irFlag = false;					// Reset flags
    ambFlag = false;
    delay(2000);					// Simulate other sensors or code
    digitalWrite(actionPin,LOW);			// Make sensor start sending data  
  }											


}


void tn9Data(){						// Interrupt Function
  cbit =  digitalRead(dataPin);				// Read bit
  if(pos >= len) pos = 0;				// Keep index below 5
  values[pos] = (values[pos] << 1) | cbit;	        // Store to values
  n++;							// Increment bit count
  if(n == 8){						// Increment position count based on bits read in (Increment after 1 byte)
    pos++;
    n = 0; 
  }
  if(pos == len){ 					// If complete "packet" sent, stop sensor from sending 
    digitalWrite(actionPin,HIGH);  			// again until main loop allows it.
  }
}


float tn9Temp(volatile byte tn9Values[]){				// TN9 temperature calculation function...
  	word tempword = 0;						// Initialize temperature isolation word
  	float temperature = 0.0;					// Initialize return
  	boolean crc = false;						// Initialize checksum
	int mcheck = (int)tn9Values[0] + (int)tn9Values[1] + (int)tn9Values[2];	// Checksum calculation
 	int scheck = (int)tn9Values[3];				        // sensor side checksum
  	if(mcheck > 510) mcheck = mcheck - 512;		                // Handle sensor byte rollover
  	if(mcheck > 255) mcheck = mcheck - 256;		                // Handle sensor byte rollover :: 255 check MUST come after 510
  	if(mcheck == scheck) crc = true;			        // Validate checksum
  	tempword = tempword | tn9Values[1];			        // Isolate temperature components...
  	tempword = tempword << 8;
  	tempword = tempword | tn9Values[2];
  	if(crc){					                // Calculate temperature if checksum valid...
  		temperature = int(tempword)/16.0 - 273.15;	
  	}
  	else{
  		temperature = -273.15;                                  // Temperature = 0 Kelvin if cheksum invalid
  	}
  	return(temperature);					        // Return
} 
