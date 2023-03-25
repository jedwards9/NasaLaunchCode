// Flip X axis once adapter is taken off 
#include <ArduinoQueue.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include "uRTCLib.h";
#include "Globals.h";

void setup() {
  delay(5000);
  pinMode(telescopePin, OUTPUT);
  pinMode(swivelPin, OUTPUT);
  pinMode(tiltPin, OUTPUT);
  pinMode(mainPWM, OUTPUT);
  pinMode(mainDir1, OUTPUT);
  pinMode(mainDir2, OUTPUT);
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(airbagDeploy, OUTPUT);

  digitalWrite(airbagDeploy, HIGH);

  // RTC setup 
  uRTCLib rtc(0x68);
  URTCLIB_WIRE.begin();

  // MPU setup 
  Serial.begin(115200);
  if (!mpu.begin(0x69)) {
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
  telescopeServo.attach(telescopePin);

  // Setting servos initially to off
  tiltServo.write(75);
  rotationServo.write(90);
  telescopeServo.write(0);

  // Initializing queue with 0s
  for(int i = 0; i < MAX_QUEUE_SIZE; i++) {
    data.enqueue(sensorReadings{0,0,0});
  }

  // Variable initialization 
  rocket_state = ON_PAD;          // Stage counter 
  currAxis = MAIN;                // Axis being leveled
  initial_angle = 0.0;            // Starting position of camera upon landing
  moveCamera = false; 
}

void loop() {
  static sensorReadings prevReadings;
  static sensorReadings deltaReadings;
  static sensorReadings readings = {0,0,0};
  delay(SAMPLE_PERIOD);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //prevReadings = readings;
  readings = { a.acceleration.x, a.acceleration.y, a.acceleration.z };
  if(rocket_state == IN_AIR){
    readings.y = abs(readings.y);
  }
  //deltaReadings = {readings.x - prevReadings.x, readings.y - prevReadings.y, readings.z - prevReadings.z};
  sensorReadings queueSum = queueLogic(readings);


  switch(rocket_state) {
    case ON_PAD:
      //blink LED to indicate on the pad
      digitalWrite(LED_BUILTIN, millis() % 500 < 250);
      // Waiting for takeoff
      if( impulseDetection(LAUNCH_THRESH, readings, queueSum.y) ) {
        rocket_state = IN_AIR;
        launchTime = millis();
        //wait until after apogee
        delay(LAUNCH_DEAD_TIME);
      }
      // Else: do nothing
      break;


    case IN_AIR:
      //check if secondary needs deployed yet
      if(millis() - launchTime > AIRBAG_DELAY_TIME){
        digitalWrite(airbagDeploy, LOW);
      }
      digitalWrite(LED_BUILTIN, LOW);
      // Wait for landing 
      if( impulseDetection(LANDING_THRESH, readings, queueSum.y) ) {
        rocket_state = LANDED;
        initial_angle = readings.z;
      }
      break;


    case LANDED:
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(airbagDeploy, HIGH);
      // Level axes
      // if ( currAxis == NONE ) {
      //   // Wait for no movement
      //   impulseDetection(LANDING_THRESH, readings, mag(queueSum));
      // }
      if( currAxis == MAIN)  {
        if(rotationProtection) {
          // Begin leveling
          motorLogic(readings.x, readings);
        }
        /*
          Past level... do something (or not)
          If we do nothing, it might be close enough to level 
          to just stop and call it leveled
        */
      }
      else if( currAxis == TELESCOPE){
          Serial.println("Telescope");
          motorLogic(readings.x, readings); //Doesn't Matter the Inputs I dont think.
          currAxis = R_AXIS;
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
            // Right 60o
            moveCamera = true;
            motorWrite(1, 0);
          }
          else if(curCommand.equalsIgnoreCase("B2")) {
            // Left 60o
            moveCamera = true;
            motorWrite(0, 0);
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
  return "A1C3B2C3F6D3";
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
      if(moveCamera) {
        if(dir) { // Right
          rotationServo.write(60);
          delay(DELAY_60deg);
        }
        else {    // Left
          rotationServo.write(120);
          delay(DELAY_60deg);
        }
        rotationServo.write(90);
      }
      moveCamera = false;
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
  static int tilt_pos = 90;
  runVal = (sensorVal* MOTOR_SMOOTHING) + (runVal * (1-MOTOR_SMOOTHING));
  if(abs(runVal) < MIN_ROTATION_SPEED) {
    runVal = (runVal / abs(runVal)) * MIN_ROTATION_SPEED;
  }

  switch(currAxis) {
    case NONE:
      // Do nothing 
      break;

    case MAIN:
      if(readings.z < 0 || runVal > MAX_ROTATION_SPEED){
        if(runVal > 0){
          runVal = MAX_ROTATION_SPEED;
        }
        else{
          runVal = -MAX_ROTATION_SPEED;
        }
      }

      if( abs(sensorVal) > INITIAL_THRESH ) {
        //motorWrite( runVal < 0, 20*abs(runVal) );
        motorWrite( runVal < 0, abs(runVal));
      }
      else {
        Serial.println("TELESCOPE");
        motorWrite(false, 0);
        currAxis = TELESCOPE;
      }
      
      // if (readings.z < MAIN_EPSILON) {
      //   Serial.println("Telescope");
      //   currAxis = TELESCOPE;
      // }

      break;
    
    case TELESCOPE:
      Serial.println("Telescope");
      for (int i = 0; i <= 180; i += 1) {
        telescopeServo.write(i);
        delay(20);
      }
      currAxis = R_AXIS;
      
      break;

    case R_AXIS:
      if( abs(readings.y) > TILT_EPSILON ) {
        if( readings.y - TILT_EPSILON > 0 ) {
          tilt_pos -= 1;
          delay(20);
        }
        if( readings.y - TILT_EPSILON < 0 ) {
          tilt_pos += 1;
          delay(20);
        }
        tiltServo.write(tilt_pos);
      }
      else if( abs(readings.y) < TILT_EPSILON ) { 
        currAxis = LEVELED;
        Serial.println("Done leveling");
        digitalWrite(LED_BUILTIN, LOW);
        delay(3000);
        takePicture();
        delay(3000);
      }

      break;

    case LEVELED:

      break;

    default:
      break;
  }
}

bool impulseDetection(int thresh, sensorReadings readings, float queueSum) {
  static bool prevLaunchSign = false;
  static int stateCounter = 0;
  Serial.println(queueSum / MAX_QUEUE_SIZE);
  // Transition between ON_PAD and IN_AIR
  if( (rocket_state == ON_PAD) && (queueSum > thresh) ) {
    return true;
  }
  // Transition between IN_AIR and LANDED
  else if( (rocket_state == IN_AIR) && (queueSum < thresh) ) {
    return true;
  }
  return false;
}

sensorReadings queueLogic(sensorReadings currVal) {
  static sensorReadings currSum = {0, 0, 0};
  currSum = addReadings(currSum, currVal);
  sensorReadings head = {0, 0, 0};
  if(data.isFull()){
    head = data.dequeue();
  }
  currSum = subReadings(currSum, head);
  data.enqueue(currVal);
  return currSum;
}

sensorReadings addReadings(sensorReadings A, sensorReadings B){
  sensorReadings result = {A.x + B.x, A.y + B.y, A.z + B.z};
  return result;  
}

sensorReadings subReadings(sensorReadings A, sensorReadings B){
  sensorReadings result = {A.x - B.x, A.y - B.y, A.z - B.z};
  return result;  
}

sensorReadings absReadings(sensorReadings A){
  sensorReadings result = {abs(A.x), abs(A.y), abs(A.z)};
  return result;
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

void takePicture(){
  rtc.refresh();
    
    String message = "1 PICTURE Date_";
    message += rtc.month();
    message += "-";
    message += rtc.day();
    message += "-";
    message += rtc.year();
    message += " Time_ ";
    message += rtc.hour();
    message += ";";
    message += rtc.minute();
    message += ";";
    message += rtc.second();

    Serial.println(message);
}