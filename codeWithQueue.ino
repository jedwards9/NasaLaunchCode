#include <ArduinoQueue.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int MAX_QUEUE_SIZE = 50;
const int MAX_ANGLE = 10;     // Maximum the accelerometer can output
const int MIN_ANGLE = -10;    // Minimum the accelerometer can output
const float THRESHOLD = 1;    // Used for the running average in landing detection to see if the rocket is moving or not
const float EPSILON = 0.02;   // Used for comparing floats in the rotation protection function

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
int rotationCounter;
int state;
int prevState;

float initAngel;
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

  for(int i = 0; i < MAX_QUEUE_SIZE; i++) {
    data.enqueue(0);
  }

  currentQueueSize = 1;   // Initializing to 1 because the value is incremented at the end of the function
  state = 0;
  prevState = 1;
  rotationCounter = 0;
  initAngle = 100;
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

  rotationProtection(a.acceleration.y);

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
  if( abs(curSum - THRESHOLD) <= 0) {
    return true;
  }
  else {
    return false;
  }
}

/**
 *
 */
void rotationProtection(bool dir, float y) {
  
/*
Every passing of max / min, relative to the starting position, is a half turn
  Determining max and min would be +10 or -10 from the starting position
If we keep track of the number of times we pass max / min and add / subtract based on the direction we're going, 
  and mod 2, then we could tell if we made a full turn

Better idea

State machine: 
  State 0 (starting state) = Initial value
  State 1a                 = Negative extreme
  State 1b                 = Positive extreme
  State 2                  = Negative of the original value
  State 3a                 = Positive extreme
  State 3b                 = Negative extreme
  State 4 (or 0)           = Initial value


*/

  float comparisonVal = 0;

  switch(state) {
    case 0: /**  @ Hold / Reset  **/
      if(start) {
        initAngel = y;
        state = prevState;    // prevState is initial set to 1
        return
      }
      else { state = 0; }
      
      break;

    case 1: /**  @ Initial Angel  **/
      if( (dir == CCW) && (abs(y - MAX_ANGLE) <= EPSILON) ) {
        state = 2
      }
      else if( (dir == CW) && (abs(y - MIN_ABGLE) <= EPSILON) ) {
        state = 3;
      }
      else if( !start ) {
        state = 0;
        prevState = 1;
      }
      else { state = 1; }

      break;

    case 2: /**  Extreme 1  **/
      // Moving to Negative
      if( abs(y - initValue) <= EPSILON ) {
        state = 4;
      }
      // Moving back to Initial Value
      else if( abs(y - initValue) <= EPSILON ) {
        state = 1;
      }
      // Motor stopped spinning, moving to Hold
      else if( !start ) { 
        state = 0;
        prevState = 2;
      }
      else { state = 2; }

      break;

    case 3: /**  Extreme 2  **/
      // Moving to Negative
      //   y + because the value should be the negative of the initial, saves a multiplication                                                            // TODO: Be sure this is right 
      if( abs(y + initValue) <= EPSILON ) {
        state = 4;
      }
      else if() {

      }
      else if( !start ) {
        state = 0;
        prevState = 3;
      }
      else { state = 3; }

      break;

    case 4: /**  @ Negative  **/
      break;
    case 5: /**  @ Extreme 2  **/
      break;
    case 6: /**  @ Extreme 1  **/
      break;
    default
      // Error...
      break;
  }


  Serial.print(y);
  Serial.print(" ");
  Serial.print(rotationCounter);
  Serial.println(" ");

}












