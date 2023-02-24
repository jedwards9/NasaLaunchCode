// Flip X axis once adapter is taken off 
#include <ArduinoQueue.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include "uRTCLib.h";
#include "Globals.h";

void setup() {
  pinMode(telescopePin, OUTPUT);
  pinMode(swivelPin, OUTPUT);
  pinMode(tiltPin, OUTPUT);
  pinMode(mainPWM, OUTPUT);
  pinMode(mainDir1, OUTPUT);
  pinMode(mainDir2, OUTPUT);
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);

  // RTC setup 
  URTCLIB_WIRE.begin();

  // MPU setup 
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

  // Initializing servo pins
  tiltServo.attach(tiltPin);
  rotationServo.attach(swivelPin);

  // Setting servos initially to off
  tiltServo.write(90);
  rotationServo.write(90);

  // Initializing queue with 0s
  for(int i = 0; i < MAX_QUEUE_SIZE; i++) {
    data.enqueue(0);
  }

  // Variable initialization 
  rocket_state = ON_PAD;    // Stage counter 
  currAxis = NONE;               // Axis being leveled
  initial_angle = 0.0;                  // Starting position of camera upon landing
}

void loop() {
  delay(SAMPLE_PERIOD);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  sensorReadings readings = { a.acceleration.x, a.acceleration.y, a.acceleration.z };
  float magnitude = mag(readings);
  int queueSum = queueLogic(magnitude);

  switch(rocket_state) {
    case ON_PAD:
      // Waiting for takeoff
      if( impulseDetection(LAUNCH_THRESH, readings, queueSum) ) {
        rocket_state = IN_AIR;
      }
      // Else: do nothing
      break;
    case IN_AIR:
      // Wait for landing 
      if( impulseDetection(LANDING_THRESH, readings, queueSum) ) {
        rocket_state = LANDED;
        initial_angle = readings.z;
      }
      break;
    case LANDED:
      // Level axes
      if ( currAxis == NONE ) {
        // Wait for no movement
        impulseDetection(LANDING_THRESH, readings, queueSum);
      }
      else if( currAxis == MAIN)  {
        if(rotationProtection) {
          // Begin leveling
          motorLogic(readings.z, readings);
        }
        /*
          Past level... do something (or not)
          If we do nothing, it might be close enough to level 
          to just stop and call it leveled
        */
      }  
      else if( currAxis == R_AXIS ) {
        motorLogic(readings.x, readings);
      }
      else if ( currAxis == LEVELED ) {
      /*  Need a for loop to take the first two characters of the string and determine what they are
          Commands are also separated by a space, += 3 gets the next character, the space, and moves
          to the beginning of the next command -- same reason for (i < length - 2)
      //*/
      String comStr = getRadioData();
      for(int i = 0; i < (comStr.length() - 2); i += 3) {
        String curCommand = comStr.substring(i,i+1);
          if(curCommand.equalsIgnoreCase("A1")) {
            // Motor logic?
          }
          else if(curCommand.equalsIgnoreCase("B2")) {
            
          }
          else if(curCommand.equalsIgnoreCase("C3")) {
            // Take picture
            cameraCommands(TAKE_PICTURE);
          }
          else if(curCommand.equalsIgnoreCase("D4")) {
            // Change camera mode from color to grayscale
            cameraCommands(TO_GRAY);
          }
          else if(curCommand.equalsIgnoreCase("E5")) {
            // Change camera mode from grayscale to color
            cameraCommands(TO_COLOR);
          }
          else if(curCommand.equalsIgnoreCase("F6")) {
            // Rotate image 180o
            cameraCommands(FLIP_180);
          }
          else if(curCommand.equalsIgnoreCase("G7")) {
            // Apply special filter
            cameraCommands(APPLY_SPECIAL);
          }
          else if(curCommand.equalsIgnoreCase("H8")) {
            // Remove all filters
            cameraCommands(REMOVE_FILTER);
          }
          else {
            // Panic
          }
        }
      }
      break;
    default:
      // Throw error or do nothing? 
      break;
  }
}

String getRadioData(){
  return "";
}

void motorWrite(bool dir, int speed) {
  switch(currAxis) {
    case NONE:
      digitalWrite(mainDir1, 0);
      digitalWrite(mainDir2, 0);
      analogWrite(mainPWM, 0);
      telescopeServo.write(0);
      tiltServo.write(90);
      rotationServo.write(90);
      break; 

    case MAIN:
      digitalWrite(mainDir1, dir);
      digitalWrite(mainDir2, !dir);
      analogWrite(mainPWM, speed);
      break;

    case LEVELED:
      // Camera head movement
      if( dir ) {
        tiltServo.write(speed);
        delay(DELAY_60o);
        tiltServo.write(90);
      }
      else if( !dir ) {
        tiltServo.write(90 + speed);
        delay (DELAY_60o);
        tiltServo.write(90);
      }
      break;

    default:
      // Turning off all the pins
      digitalWrite(mainDir1, 0);
      digitalWrite(mainDir2, 0);
      analogWrite(mainPWM, 0);

      digitalWrite(tiltPin, 0);
      digitalWrite(swivelPin, 0);
      break;
  }
}

void motorLogic(float sensorVal, sensorReadings readings) {
  static float runVal = 0;
  static int tilt_pos = 0;
  runVal = (sensorVal* MOTOR_SMOOTHING) + (runVal * (1-MOTOR_SMOOTHING));

  switch(currAxis) {
    case NONE:
      // Do nothing 
      break;

    case MAIN:
      if(readings.z < 0 || runVal > ROTATION_SPEED){
        if(runVal > 0){
          runVal = ROTATION_SPEED;
        }
        else{
          runVal = -ROTATION_SPEED;
        }
      }

      if( abs(runVal) > INITIAL_THRESH ) {
        //motorWrite( runVal < 0, 20*abs(runVal) );
        motorWrite( runVal < 0, ROTATION_SPEED );
      }
      else {
        motorWrite(false, 0);
      }
      
      if (readings.z < MAIN_EPSILON) {
        currAxis = TELESCOPE;
      }

      break;
    
    case TELESCOPE:
      for (int i = 0; i <= 180; i += 1) {
        telescopeServo.write(i);
        delay(25);
      }
      currAxis = R_AXIS;
      
      break;

    case R_AXIS:
      if( abs(readings.y) > TILT_EPSILON ) {
        if( readings.y - TILT_EPSILON > 0 ) {
          tilt_pos += 1;
          delay(250);
        }
        if( readings.y - TILT_EPSILON < 0 ) {
          tilt_pos -= 1;
          delay(250);
        }
      }
      else if( abs(readings.y) < TILT_EPSILON ) { 
        currAxis = LEVELED;
      }

      break;

    case LEVELED:

      break;

    default:
      break;
  }
}

bool impulseDetection(int thresh, sensorReadings readings, int queueSum) {
  static bool prevLaunchSign = false;
  static int stateCounter = 0;
  // Transition between ON_PAD and IN_AIR
  if( (rocket_state == ON_PAD) && (abs(queueSum) > thresh) ) {

    if(prevLaunchSign == (readings.y > 0)) {
      stateCounter++;
      prevLaunchSign = (readings.y > 0);

      if( stateCounter >= (1000 / SAMPLE_PERIOD) ) {
        // If no movement for 1 second: 
        stateCounter = 0;
        return true;
      }
      return false;
    }
    else { 
      stateCounter = 0; 
      prevLaunchSign = (readings.y > 0);
      return false;
    }


  }
  // Transition between IN_AIR and LANDED
  else if( (rocket_state == IN_AIR) && (abs(queueSum) < thresh) ) {
    // Counter to make sure there is no movement:
    stateCounter++;
    if( stateCounter >= (1000 / SAMPLE_PERIOD) ) {
      // If no movement for 1 second: 
      stateCounter = 0;
      return true;
    }
    else {
      return false;
    }

  }
  // Is a LANDED case needed? 
  else if(rocket_state == LANDED){
    stateCounter = 0;
    return false;
  }
  return false;
}

int queueLogic(float nextValue) {
  static int previousValue = 0;
  static int currSum = 0;
  float difference = abs(nextValue - previousValue);
  float head = data.dequeue();

  // Setting the current running average to 
  currSum = currSum + difference - head;
  previousValue = nextValue;

  // Putting the new reading on the queue
  data.enqueue(difference);
  return currSum;
}

float mag(sensorReadings readings) {
  float magnitude = sqrt(sq(readings.x) + sq(readings.y) + sq(readings.z));
  return magnitude;
}

//TODO: check state changes 
bool rotationProtection(float axis) {  
  static int state = 1;
  switch(state) {
    case 1: /**  @ Initial Angle  **/
      // Next state (at vertical)
      if(abs(axis - MAX_ANGLE <= EPSILON)) {
        state = 2;
      }
      // Hold: else { state = 1; }
      return true;
      break;

    case 2: /**  @ Vertical  **/
      // Next state (past vertical)
      if( abs(axis + initial_angle) <= EPSILON ) {
        state = 3;
      }
      // Previous state 
      else if( abs(axis - MAX_ANGLE) > EPSILON) {
        state = 1;
      }
      // Hold: else { state = 2; }
      return true;
      break;

    case 3: /**  @ Past Vertical **/
      if( abs(axis - MAX_ANGLE) <= EPSILON ) {
        state = 2;
      }
      // Hold: else { state = 3; }
      return false;
      break;

    // Maybe add another state for back to initial angle 
    default:
      // Error...
      break;
  }
}

void cameraCommands(int camera_command) {
  /*
    const int TAKE_PICTURE = 1;
    const int TO_GRAY = 2;
    const int TO_COLOR = 3;
    const int APPLY_SPECIAL = 4;
    const int REMOVE_FILTER = 5;
    const int FLIP_180 = 6;
  */  
  if(camera_command > 6) {
    // Not a camera command ...
    // just return? or throw error? 
    return;
  }
  
  rtc.refresh();

  Serial.print(camera_command);
  Serial.print(" ");
  // delay(2000)
  Serial.print(rtc.month());
  Serial.print('/');
  Serial.print(rtc.day());
  Serial.print('/');
  Serial.print(rtc.year());

  Serial.print(rtc.hour());
  Serial.print(':');
  Serial.print(rtc.minute());
  Serial.print(':');
  Serial.print(rtc.second());
  
}











