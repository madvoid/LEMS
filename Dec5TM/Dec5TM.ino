//Decagon 5TM soil moisture and temperature sensor

double moisture = 0.0;        // Soil Moisture
double temperature = 0.0;     // Soil Temperature
const int powerPin = 3;       // Sensor Power

void setup(){
  Serial.begin(1200);                              // Communicates @ 1200 baud
  Serial3.begin(1200);                             // Must use serial pins 
  pinMode(powerPin,OUTPUT);                        // Set powerPin as output
  digitalWrite(powerPin,LOW);
  Serial.println("Type any character to start");   
  while (!Serial.available());                     // Wait for start
  Serial.println("Raw Input,Moisture,Temperature");
}


void loop(){
  int i = 0;                                        // Serial buffer count
  digitalWrite(powerPin,HIGH);                      // Start 5TM 
  delay(200);                                       // Wait for conversion
  char input[Serial3.available()];                  // Read Sensor Values
  while (Serial3.available()>0){ 
    input[i] = Serial3.read();
    i++;
  }

  digitalWrite(powerPin,LOW);                     // Reset sensor
  delay(50);                                      // Make sure sensor is reset properly
  if(Serial3.available() == 0){                   // If everything read in
    ftmParse(input,moisture,temperature);         // Parse input string
    Serial.println(input);                        // Print raw input, moisture, temperature
    Serial.print(moisture,5);                     
    Serial.print(",");
    Serial.println(temperature,5);
  }
  moisture = 0.0;                                  // Reset variables
  temperature = 0.0;
}

  
void ftmParse(char input[], double &mois, double &temp){          // 5TM string parse function

  double values[3] = {0,0,0};                                     // Initialize temporary values storage
  int fieldIndex = 0;                                             // Index
  int check = 0;                                                  // Checksum location

  for(int j = 0; j < strlen(input); j++){                         // Parse string, if number add to one of values indices, if space, go to next index, anything else, leave loop, mark position
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

  values[1] = int(input[strlen(input)-3]);                        // Store sensor side checksum
  char crc = 0;                                                   // Calculate checksum
  for(int j = 0; j < check; j++){
    crc += input[j];   
  }
  crc = (crc + 0xD + 'x') % 64 + 32;                              // From datasheet, needed to calculate checksum


  mois = 0.0000043*pow(double(values[0])/50.0,3)-0.00055*pow(double(values[0])/50.0,2)+0.0292*(double(values[0])/50.0)-0.053;
                                                                  // Calculate moisture with topp equation

  if (values[2] <= 900){                                          // Calculate temperature
    temp = (float(values[2])-400.0)/10.0;
  }
  if (values[2] > 900){
    temp = 900.0+5.0*(float(values[2])-900.0);
    temp = (temp - 400.0)/10.0; 
  }

  if( values[1] != crc){                                           // If checksum isn't valid, return impossible numbers
    mois = 0;
    temp = -273.15; 
  }
}






