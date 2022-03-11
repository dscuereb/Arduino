#include <dht11.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#define seaLevelPressure_hPa 1013.25
#define DHT11PIN A2
#define sensorPower 7
#define sensorPin A0

#define TdsSensorPin A1
#define VREF 5.0 // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

dht11 DHT11;
Adafruit_BMP085 bmp;

int pinOut_R1 = 2;
int pinOut_R2 = 4;

int val = 0;
int thresh = 200;
int n = 0;

////// Average waterlevel value:
const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
int inputPin = A0;
///////////

void setup() {
  Serial.begin(9600);
  pinMode(pinOut_R1, OUTPUT);
  pinMode(pinOut_R2, OUTPUT);
  pinMode(sensorPower, OUTPUT);
  pinMode(TdsSensorPin, INPUT);
  digitalWrite(sensorPower, LOW);
  digitalWrite(pinOut_R2, LOW);
  Wire.begin();



  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;

  }

}

void loop() {
Wire.requestFrom(27, 77);
  
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

  ////////////////////////////////////////////

    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");

    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(bmp.readSealevelPressure());
    Serial.println(" Pa");

    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(seaLevelPressure_hPa * 100));
    Serial.println(" meters");


////////////////////////////////////////////
  

  int chk = DHT11.read(DHT11PIN);
  int h = DHT11.humidity;
  Serial.print("Humidity (%): ");
  Serial.println((float)DHT11.humidity, 2);
  Serial.print("Temperature (C): ");
  Serial.println((float)DHT11.temperature, 2);
  //get the reading from the function below and print it
  int level = readSensor();
  Serial.print("Water level: ");
  Serial.println(level);


  //Serial.print("Number of readings: ");
  //Serial.println(numReadings);
  //Serial.print("Read index is: ");
  //Serial.println(readIndex);


  n++;
  if (n > 20) {
    Serial.print("Avg water level: ");
    Serial.println(average);
    Serial.println();
    // Control R2: Pump and Water levle sensor
    if (average <= thresh) {
      digitalWrite(pinOut_R2, HIGH);
      Serial.println("- Pump ON");
      //Serial.println();
    }
  }
  digitalWrite(pinOut_R2, LOW);
  Serial.println("- Pump OFF");
  //Serial.println();

  // Control R1: Fan and Temp/Humidity sensor
  if (h <= 60) {
    digitalWrite(pinOut_R1, LOW);
    Serial.println("- Fan OFF");
    //Serial.println();
  }
  else {
    digitalWrite(pinOut_R1, HIGH);
    Serial.println("- Fan ON");
    //Serial.println();
  }
  Serial.println();
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) //every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); //read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0; // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0); //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    tdsValue = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value
    Serial.print("voltage:");
    Serial.print(averageVoltage, 2);
    Serial.print("V ");
    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");
  }
}
int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
  delay(2000);

}
//This is a function used to get the reading
int readSensor() {
  digitalWrite(sensorPower, HIGH);  // Turn the sensor ON
  delay(2000);              // wait 10 milliseconds
  val = analogRead(sensorPin);    // Read the analog value form sensor
  digitalWrite(sensorPower, HIGH);   // Turn the sensor OFF
  return val;             // send current reading
}
