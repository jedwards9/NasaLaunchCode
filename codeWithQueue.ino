#include <ArduinoQueue.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int MAX_QUEUE_SIZE = 50;
const float THRESHOLD = 1;

int PWMPin = 11;
int dirPin1 = 12;
int dirPin2 = 13;

float threshVal = 0.5;
float smoothVal = 0.8;
float runVal = 0;

bool isLanded = false;
bool isLeveled = false;
bool collectData;

int currentQueueSize; 
int counter;
float curSum;
float temp;
float previousValue;

ArduinoQueue<float> data(MAX_QUEUE_SIZE);



void setup() {
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);

  /*
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float initXRot = 
  float initYRot = 
  float initZRot = 
  */

  for(int i = 0; i < MAX_QUEUE_SIZE; i++) {
    data.enqueue(0);
  }

  currentQueueSize = 1;   // Initializing to 1 because the value is incremented at the end of the function
  counter = 0;
  curSum = 0;
  temp = 0;
  previousValue = 0;

}

void loop() {
  delay(100);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float normalized = normalize( a.acceleration.x, a.acceleration.y, a.acceleration.z );
  queueLogic( normalized );

  if(!detectLanding()) {  
  }
  if(detectLanding()) {
    motorLogic( a.acceleration.y );
  }
  
  Serial.print("AccelX:");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print("AccelY:");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print("AccelZ:");
  Serial.print(a.acceleration.z);
  Serial.println(" ");

  /*
    // Platform is leveled
    if(isLeveled) {
      // Stop movement

    }
  */
}


/**
 * Reads values from the sensor and determines 
 * which way to spin the motors
 */
void motorLogic(float readVal) {
  float runVal = (readVal * smoothVal) + (runVal * (1-smoothVal));

  if(abs(runVal) > threshVal){
    motorWrite(runVal < 0, 20*abs(runVal));
  }
  else{
    motorWrite(false, 0);
    isLeveled = true;
  }
  Serial.println(runVal);
  delay(100);
}


/** 
 * Sets the speed and direction of the motors
 */
void motorWrite(bool dir, int speed) {
  digitalWrite(dirPin1, dir);
  digitalWrite(dirPin2, !dir);
  analogWrite(PWMPin, speed);
}

/**
 * Will perform a running average on the queue, 
 * Both adding a removing values.  
 */
void queueLogic(float newValue) {
  float difference = abs(newValue - previousValue);
  float head = data.dequeue();

  // Setting the current running average to 
  curSum = curSum + difference - head;
  previousValue = newValue;

  // Putting the new reading on the queue
  data.enqueue(difference);

  //Serial.println( curSum );
  //Serial.println( difference );
}

float normalize(float x, float y, float z) {
  float normalizedValue = sqrt(sq(x) + sq(y) + sq(z));
  return normalizedValue;
}

/**
 * Will output true if the threshold value has been passed for  
 * the running average of the sensor data.  
 */
bool detectLanding() {
  if(curSum < THRESHOLD) {
    return true;
  }
  else {
    return false;
  }
}

/**
 *
 */
void rotationProtection(bool direction, float y) {
  
}












