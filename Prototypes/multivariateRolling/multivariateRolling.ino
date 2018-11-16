int sensorData[2];   //for user sensor data (not in library)
int ind = 0;  //just for testing

int lastRun; //for library
int windowArr[50][20];  //rolling window
int bucketItems = 0;  //current length of array filled
int thresPercent = 15;  //15% noise tolerance
int windowWidth = 20; // 20 items in rolling window

void rollWindow(int *data) {
  if (bucketItems < windowWidth) {  //fill next in array
    for (int i = 0; i < sizeof(data); i++) {
      windowArr[bucketItems][i] = data[i];  //copy x, y, z into array
    }
    bucketItems++;
  }
  else  //shift array to roll window
  {
    for (int i = 0; i < windowWidth; i++) { //push data
      if (i + 1 < windowWidth) {
        //shift the next stuff backwards
        for (int j = 0; j < sizeof(data); j++) {
          windowArr[i][j] = windowArr[i + 1][j];
        }    
      }
    }
    for (int i = 0; i < sizeof(data); i++) {
      windowArr[windowWidth - 1][i] = data[i];  //copy x, y, z into array
    }
  }
}

void inactivityDetect(int interval, int *data) {
  if (millis() - lastRun > interval) {
    rollWindow(data);

    //print multidimensional array
    for (int i = 0; i < windowWidth; i++) { //push data
      for (int j = 0; j < sizeof(data) - 1; j++) {
        Serial.print(windowArr[i][j]);
        Serial.print("\t");
      }
      Serial.println();
    }
    Serial.println();
  }
}

void setWindowWidth(int w) {
  windowWidth = w;
}

void setThreshold(int perc) {
  thresPercent = perc;
}

void setup() {
  Serial.begin(115200);
}

void loop() {

  sensorData[0] = ind;
  sensorData[1] = ind + 1;
  sensorData[2] = ind + 2;
  
  //detect inactivity - data every 100ms, data
  inactivityDetect(100, sensorData);

  ind++;
  delay(100);
}
