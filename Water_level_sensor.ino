// Water level sensor init
#define sensorPower 7
#define sensorPin A0

// Value for storing water level
int val = 0;
// Threshold for water level value
int thresh = 500;

//////
const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
int inputPin = A0;
///////////
int n = 0;

void setup() {
  // Set D7 as an OUTPUT
 pinMode(sensorPower, OUTPUT);
  
  // Set to LOW so no power flows through the sensor
  digitalWrite(sensorPower, LOW);
  
  Serial.begin(9600);  
     
    // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
    
  }
}

void loop() {
  Serial.println();

  n++;
  Serial.println("n is : ");
  Serial.print(n);

  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = analogRead(inputPin);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits
  
   
  // The sensor can only be read from every 1-2s, and requires a minimum
  // 2s warm-up after power-on.
  delay(2000);


  //get the reading from the function below and print it
  int level = readSensor();
  Serial.print("Number of readings: ");
  Serial.println(numReadings);
  Serial.print("Read index is: ");
  Serial.println(readIndex);
  Serial.print("Water level: ");
  Serial.println(level);
  Serial.print("Avg water level: ");
  Serial.println(average);

//enable motor 3 (pump) untill water level reaches 'threshold' else turn it off
    if (readIndex >= numReadings) {
            if (average <= thresh) {
              // Pump on
            }

  } else {

  }
  
}

//This is a function used to get the reading
int readSensor() {
  digitalWrite(sensorPower, HIGH);  // Turn the sensor ON
  delay(2000);              // wait 10 milliseconds
  val = analogRead(sensorPin);    // Read the analog value form sensor
  digitalWrite(sensorPower, HIGH);   // Turn the sensor OFF
  return val;             // send current reading
}
