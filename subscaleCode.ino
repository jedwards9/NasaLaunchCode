#include <ArduinoQueue.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int SAMPLE_PERIOD = 100;
const int MAX_QUEUE_SIZE = 50;
const int MAX_ANGLE = 10;     // Maximum the accelerometer can output
const int MIN_ANGLE = -10;    // Minimum the accelerometer can output
const float THRESHOLD = 1;    // Used for the running average in landing detection to see if the rocket is moving or not
const float EPSILON = 0.02;   // Used for comparing floats in the rotation protection function
const bool CW = true;
const bool CCW = false;
const int MAX_ROT_RATE = 8;
const int LAUNCH_THRESHOLD = 19;

int curState = 0;
int PWMPin = 11;
int dirPin1 = 12;
int dirPin2 = 13;

float threshVal = 0.5;
float smoothVal = 0.8;
float runVal = 0;

bool hasLanded;
bool hasLaunched;
bool isLeveled = false;
bool collectData;
bool prevLaunchSign;

int launchCounter;
int currentQueueSize; 
int rotationCounter;
int state;
int prevState;

float initValue;
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

  hasLanded = false;
  hasLaunched = false;
  currentQueueSize = 1;   // Initializing to 1 because the value is incremented at the end of the function
  state = 0;
  prevState = 1;
  rotationCounter = 0;
  initValue = 100;
  curSum = 0;
  temp = 0;
  previousValue = 0;
  launchCounter = 0;

}

void loop() {
  delay(SAMPLE_PERIOD);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  curState = (int)hasLaunched + (int)hasLanded;
  float normalized = normalize( a.acceleration.x, a.acceleration.y, a.acceleration.z );
  queueLogic( normalized );

  if(hasLaunched) {
    if(hasLanded) {
      motorLogic( a.acceleration.x, a.acceleration.z );
    }
    else {
      hasLanded = detectImpulse( a.acceleration.x );
      motorWrite(false, 0);
    }
  }
  else {
    if(abs(a.acceleration.y) > LAUNCH_THRESHOLD) {
      if(prevLaunchSign == (a.acceleration.y > 0)) {
        launchCounter++;
      }      
      else {
        launchCounter = 0;
      }
    }
    else {
      launchCounter = 0;
    }
    if(!hasLaunched && launchCounter >= 1000 / SAMPLE_PERIOD) {
      hasLaunched = true;
    }
  }

  Serial.print(curState);
  Serial.print(", ");
  Serial.print(a.acceleration.x);
  Serial.print(", ");
  Serial.print(a.acceleration.y);
  Serial.print(", ");
  Serial.println(a.acceleration.z);
  
  // rotationProtection( ,  , a.acceleration.x);
}


/**
 * Reads values from the sensor and determines 
 * which way to spin the motors
 */
void motorLogic(float readVal, float z) {
  float runVal = (readVal * smoothVal) + (runVal * (1-smoothVal));

  if(z < 0 || runVal > MAX_ROT_RATE){
    if(runVal > 0){
      runVal = MAX_ROT_RATE;
    }
    else{
      runVal = -MAX_ROT_RATE;
    }
  }

  if( abs(runVal) > threshVal ) {
    motorWrite( runVal < 0, 20*abs(runVal) );
  }
  else {
    motorWrite(false, 0);
    isLeveled = true;
  }

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
}

float normalize(float x, float y, float z) {
  float normalizedValue = sqrt(sq(x) + sq(y) + sq(z));
  return normalizedValue;
}

/**
 * Will output true if the threshold value has been passed for  
 * the running average of the sensor data.  
 */
bool detectImpulse(float initialAngle) {
  if( abs(curSum) <= THRESHOLD) {
    initValue = initialAngle;
    return true;
  }
  else {
    return false;
  }
}

/**
 *
 */
void rotationProtection(bool start, bool dir, float axis) {

  float comparisonVal = 0;

  switch(state) {
    case 0: /**  @ Hold / Reset  **/
      if(start) {
        state = prevState;    // prevState is initial set to 1
        return;
      }
      else { state = 0; }
      
      break;

    case 1: /**  @ Initial Angle  **/
      if( (dir == CCW) && (abs(axis - MAX_ANGLE) <= EPSILON) ) {
        state = 2;
      }
      else if( (dir == CW) && (abs(axis - MIN_ANGLE) <= EPSILON) ) {
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
      if( abs(axis - initValue) <= EPSILON ) {
        state = 4;
      }
      // Moving back to Initial Value
      else if( abs(axis - initValue) <= EPSILON ) {
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
      if( abs(axis - initValue) <= EPSILON ) {
        state = 1;
      }
      // y + initValue since y should be negative (the opposite of initValue)
      //   They should then cancel out and give a zero
      else if( abs(axis + initValue) <= EPSILON ) {
        state = 5;
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
    default:
      // Error...
      break;
  }
}












