#include <ArduinoQueue.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int MAX_QUEUE_SIZE = 50;
const long THRESHOLD;

int PWMPin = 11;
int dirPin1 = 12;
int dirPin2 = 13;

float threshVal = 0.5;
float smoothVal = 0.8;
float runVal = 0;

bool isLanded = false;
bool isLeveled = false;
bool collectData;

// Initializing to 1 because the value is incremented at the end of the function
int currentQueueSize = 1; 

long curSum = 0;
long temp = 0;

ArduinoQueue<long> data(MAX_QUEUE_SIZE);



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

  // Initializing queue
  for(int i = 0; i < MAX_QUEUE_SIZE; i++) {
    data.enqueue(0);
  }

}

void loop() {
  delay(100);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  queueLogic( a.acceleration.x );
  Serial.println(curSum);
  
  // Landing detected
  if(detectLanding()) {
    motorLogic( a.acceleration.y );
  }
  
  // Platform is leveled
  if(isLeveled) {
    // Stop movement

  }
}


/**
 * Reads values from the sensor and determines 
 * which way to spin the motors
 */
void motorLogic(float readVal) {
  //sensors_event_t a, g, temp;
  //mpu.getEvent(&a, &g, &temp);
  //float readVal = a.acceleration.y;

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
 * Will perform a running average on the queue, both adding and removing 
 * 
 * Currently does not update curSum
 */
void queueLogic(long newValue) {
  // Store the current value divided by the queue size to save time when dequeue-ing
  temp = newValue / currentQueueSize;

  // Adding newValue's weight to the current running average
  curSum = curSum + (temp);

  // Incrementing Queue size for next iteration
  if(currentQueueSize < MAX_QUEUE_SIZE) {
    currentQueueSize += 1;
  }
  // Since the queue is full, start dequeue-ing
  else {
    // Removing the dequeued value's weight from the running average
    curSum = curSum - (data.dequeue() / 50);
  }

  // Pushing newValue onto the queue
  data.enqueue(temp);
}

/**
 * Will output true if the threshold value has been passed for  
 * the running average of the sensor data.  
 */
bool detectLanding() {
  // This inequality might need to be flipped
  if((THRESHOLD < curSum) && collectData) {
    return true;
  }
  // Else: fall through 
  return false;
}











