//IMU related stuff
#include<Wire.h>
const int MPU6050_addr=0x68;
int16_t ax, ay, az;
int16_t axC, ayC, azC;
int8_t numSamples = 0, totalSamples = 10;
bool calibrated = false;

//----------------------------Library stuff----------------------------------

int imuData[3];   //for user sensor data (not in library)

//Goes in library
int dataChannels = 0;  //number of sensor channels
int lastRun, lastSec; //millis for last execution
float windowArr[20];  //rolling window
int bucketItems = 0;  //current length of filled rolling window
int threshold = 30;  //bigger than this is considered active data
int windowWidth = 10; // 10 items in rolling window
bool deviceActive = true; //device status (prolonged inactivity will result in sleep)
int timeout = 0;  //runtime var
int maxInactivity = 10; //number of seconds of inactivity before sleep action

float runFusion(int *sensorPacket, int numItems) {
  int sum = 0;
  for (int i = 0; i < numItems; i++) {
    sum += abs(sensorPacket[i]);
  }
  //Serial.println(sum / numItems);
  return sum / numItems;
}

float average(float * arr, int len) {  // assuming array is int.
  long sum = 0L ;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++) {
    sum += arr[i];
  }
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}

void rollWindow(float newData) {
  if (bucketItems < windowWidth) {  //fill next in array
    windowArr[bucketItems] = newData;  //push onto end of array
    bucketItems++;
  }
  else  //shift array to roll window
  {
    for (int i = 0; i < windowWidth; i++) { //push data
      if (i + 1 < windowWidth) {
        //shift the next stuff backwards
        windowArr[i] = windowArr[i + 1];   
      }
    }
    windowArr[windowWidth - 1] = newData;  //push onto end of array
  }
}

void analyzePoint(float newData) {
  float avg = average(windowArr, windowWidth);  //get average given windowArr
  float devMean = abs(newData - avg); //find the difference of this point compared to mean

  if (devMean > threshold) {
    //Serial.println("Activity detected");
    deviceActive = true;
  }
  else {
    //Serial.println("No activity");
    deviceActive = false;
  }
  
  rollWindow(newData);  //move on
}

void inactivityDetect(int interval, int *sensorData, int items) {
  if (millis() - lastRun > interval) {  //run code every interval
    dataChannels = items;
    analyzePoint(runFusion(sensorData, items));
  }
  if (deviceActive) {
    timeout = 0;  //hold the clock
  }
  else {  //device is inactive, start countdown
    if (millis() - lastSec > 1000) {  //run code every second
      Serial.println(timeout);
      timeout++;
      lastSec = millis(); //update lastSec
    }
    if (timeout > maxInactivity) { //time is up, begin sleep
      timeout = 0;
      Serial.println("Now going to deep sleep...");
      delay(5000);
    }
  }
}

void setMaxInactivity(int maxSeconds) {
  maxInactivity = maxSeconds;
}

void setup() {
  //IMU stuff
  Wire.begin();
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  Serial.begin(115200);

  setMaxInactivity(60); //set device to sleep in 30 sec of inactivity
}

void loop() {
  //IMU stuff
  Wire.beginTransmission(MPU6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_addr,14,true);
  ax=map(Wire.read()<<8|Wire.read(), 0, 65536, 0, 1023);
  ay=map(Wire.read()<<8|Wire.read(), 0, 65536, 0, 1023);
  az=map(Wire.read()<<8|Wire.read(), 0, 65536, 0, 1023);

  if (calibrated == false) {  //IMU calibration
    if (numSamples < totalSamples) {
      axC += ax;
      ayC += ay;
      azC += az;
      numSamples++;
    }
    else {
      axC = ((float)axC / totalSamples);
      ayC = ((float)ayC / totalSamples);
      azC = ((float)azC / totalSamples);
      calibrated = true;
    }
  }
  else {
    //Compensate
    ax = ax - axC;
    ay = ay - ayC;
    az = az - azC;
    
    //Get your calibrated data here---------Library stuff--------------
    imuData[0] = ax;
    imuData[1] = ay;
    imuData[2] = az;
    
    //detect inactivity - data period 100ms, data array, number of items
    inactivityDetect(200, imuData, 3);

  }

  delay(20);
}
