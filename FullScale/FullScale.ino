// Flip X axis once adapter is taken off 
#include <ArduinoQueue.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include "uRTCLib.h";

/**  Pin Definitions  **/
const int mainPWM = 11;
const int mainP1 = 12;
const int mainP2 = 13;
const int tiltPin = 9;
const int rotationPin = 8;
const int telescopePin = 7;
const int STBY = 5;

/**  Program Constants  **/  
const int SAMPLE_PERIOD = 100;
const int MAX_QUEUE_SIZE = 50;
const int MAX_ANGLE = 10;
const int TILT_SERVO_SPEED = 45;
const int DELAY_60o = 500;

// might want different speeds for the different axes
const int ROTATION_SPEED = 8;     // Rotation speed for main axis

/**  Threshold Values  **/
const float INITIAL_THRESH = 0.5; // Main axis threshold
const int LANDING_THRESH = 1;     // Landing movement threshold
const int LAUNCH_THRESH = 19;     // Launch movement threshold
const float EPSILON = 0.02;       // Float comparison error threshold
const float MAIN_EPSILON = 0.2;   // Float comparison error threshold for MAIN leveling 
const float TILT_EPSILON =  0.5;  // Float comparison error threshold for TILT leveling

/** Picture Commands **/
const int TAKE_PICTURE = 1;
const int TO_GRAY = 2;
const int TO_COLOR = 3;
const int APPLY_SPECIAL = 4;
const int REMOVE_FILTER = 5;
const int FLIP_180 = 6;

/**  Enums Variables  **/
enum flightStage { 
  ON_PAD, 
  IN_AIR, 
  LANDED 
};
enum levelAxis {
  NONE,
  MAIN,
  TELESCOPE,
  R_AXIS,
  LEVELED
};
struct sensorReadings { 
  float x;
  float y;
  float z;
};

/**  Function Specific Variables  **/
float previous_value;   // Queue logic
float curSum;           // Queue logic
float smoothVal;        // Motor logic
float runVal;           // Motor logic
float normalized;       // Normalize
float initial_angle;    // Loop -- IN_AIR
int state;              // Rotation Protextion
int stateCounter;       // Loop -- ON_PAD
bool prevLaunchSign;    // 
int tilt_pos;           // Motor Logic
String comStr;

/**  Global Variables  **/
sensorReadings readings;
flightStage rocket_state;
levelAxis level;
ArduinoQueue<float> data(MAX_QUEUE_SIZE);
Adafruit_MPU6050 mpu;
uRTCLib rtc(0x68);
Servo tiltServo;
Servo rotationServo;
Servo telescopeServo;


void setup() {
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);

  // RTC setup 
  URTCLIB_WIRE.begin();

  // ESP32 communication setup
  Serial.begin(9600);

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
  rotationServo.attach(rotationPin);

  // Setting servos initially to off
  tiltServo.write(90);
  rotationServo.write(90);

  // Initializing queue with 0s
  for(int i = 0; i < MAX_QUEUE_SIZE; i++) {
    data.enqueue(0);
  }

  // Variable initialization 
  normalized = 0;                       // Normalized vector
  flightStage rocket_state = ON_PAD;    // Stage counter 
  levelAxis level = NONE;               // Axis being leveled
  stateCounter = 0;                     // Durration of still time on ground
  prevLaunchSign = false;               // Launch counter direction
  initial_angle = 0.0;                  // Starting position of camera upon landing
  state = 0;                            // Rotation protection FSM state
  smoothVal = 0.8;                      // Movement constant for motorLogic
  runVal = 0;                           // Corrected value for motorLogic
  comStr = "";                          // 
}

void loop() {
  delay(SAMPLE_PERIOD);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  readings = { a.acceleration.x, a.acceleration.y, a.acceleration.z };
  normalized = normalize();
  queueLogic( normalized );

  switch(rocket_state) {
    case ON_PAD:
      // Waiting for takeoff
      if( impulseDetection(LAUNCH_THRESH) ) {
        rocket_state = IN_AIR;
      }
      // Else: do nothing
      break;
    case IN_AIR:
      // Wait for landing 
      if( impulseDetection(LANDING_THRESH) ) {
        rocket_state = LANDED;
        initial_angle = readings.z;
      }
      break;
    case LANDED:
      // Level axes
      if ( level == NONE ) {
        // Wait for no movement
        impulseDetection(LANDING_THRESH);
      }
      else if( level == MAIN)  {
        if(rotationProtection) {
          // Begin leveling
          motorLogic(readings.z);
        }
        /*
          Past level... do something (or not)
          If we do nothing, it might be close enough to level 
          to just stop and call it leveled
        */
      }  
      else if( level == R_AXIS ) {
        motorLogic(readings.x);
      }
      else if ( level == LEVELED ) {
      /*  Need a for loop to take the first two characters of the string and determine what they are
          Commands are also separated by a space, += 3 gets the next character, the space, and moves
          to the beginning of the next command -- same reason for (i < length - 2)
      //*/
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

void motorWrite(bool dir, int speed) {
  switch(level) {
    case NONE:
      digitalWrite(mainP1, 0);
      digitalWrite(mainP2, 0);
      analogWrite(mainPWM, 0);
      telescopeServo.write(0);
      tiltServo.write(90);
      rotationServo.write(90);
      break; 

    case MAIN:
      digitalWrite(mainP1, dir);
      digitalWrite(mainP2, !dir);
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
      digitalWrite(mainP1, 0);
      digitalWrite(mainP2, 0);
      analogWrite(mainPWM, 0);

      digitalWrite(tiltPin, 0);
      digitalWrite(rotationPin, 0);
      break;
  }
}

void motorLogic(float axis) {
  float runVal = (axis * smoothVal) + (runVal * (1-smoothVal));

  switch(level) {
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
        level = TELESCOPE;
      }

      break;
    
    case TELESCOPE:
      for (int i = 0; i <= 180; i += 1) {
        telescopeServo.write(i);
        delay(25);
      }
      level = R_AXIS;
      
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
        level = LEVELED;
      }

      break;

    case LEVELED:

      break;

    default:
      break;
  }
}

bool impulseDetection(int T) {
  // Transition between ON_PAD and IN_AIR
  if( (rocket_state == ON_PAD) && (abs(curSum) > T) ) {

    if(prevLaunchSign == (readings.y > 0)) {
      stateCounter++;
      prevLaunchSign = (readings.y > 0);

      if( stateCounter >= (1000 / SAMPLE_PERIOD) ) {
        // If no movement for 1 second: 
        stateCounter = 0;
        return true;
      }
      return true;
    }
    else { 
      stateCounter = 0; 
      prevLaunchSign = (readings.y > 0);
      return false;
    }


  }
  // Transition between IN_AIR and LANDED
  else if( (rocket_state == IN_AIR) && (abs(curSum) < T) ) {
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
  else {
    stateCounter = 0;
    return false;
  }
  
}

void queueLogic(float next_value) {
  float difference = abs(next_value - previous_value);
  float head = data.dequeue();

  // Setting the current running average to 
  curSum = curSum + difference - head;
  previous_value = next_value;

  // Putting the new reading on the queue
  data.enqueue(difference);
}

float normalize() {
  float normalizedValue = sqrt(sq(readings.x) + sq(readings.y) + sq(readings.z));
  return normalizedValue;
}

//TODO: check state changes 
bool rotationProtection(float axis) {  
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












