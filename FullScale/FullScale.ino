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
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(airbagDeploy, OUTPUT);
  pinMode(debugRed, OUTPUT);
  pinMode(debugYellow, OUTPUT);

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
  mainMotor.attach(mainPWM);

  // Setting servos initially to off
  tiltServo.write(90);
  rotationServo.write(90);
  telescopeServo.write(0);
  mainMotor.write(90);

  // Initializing queue with 0s
  for(int i = 0; i < MAX_QUEUE_SIZE; i++) {
    data.enqueue(sensorReadings{0,0,0});
  }

  // Variable initialization 
  rocket_state = ON_PAD;          // Stage counter 
  currAxis = MAIN;                // Axis being leveled
  moveCamera = false;
  landingCounter = 0; 
}

void loop() {
  static sensorReadings readings = {0,0,0};
  delay(SAMPLE_PERIOD);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  readings = { a.acceleration.x, a.acceleration.y, a.acceleration.z };
  if(rocket_state == IN_AIR){
    readings.y = abs(readings.y);
  }
  sensorReadings queueSum = queueLogic(readings);


  switch(rocket_state) {
    case ON_PAD:
      Serial.println(readings.y);
      // Blink LEDs to indicate on the pad
      digitalWrite(debugRed, millis() % 500 < 250);
      digitalWrite(debugYellow, millis() % 500 < 250);
      // Waiting for takeoff
      if(impulseDetection(LAUNCH_THRESH, queueSum.y) ) {
        rocket_state = IN_AIR;
        launchTime = millis();
        // Launched: wait to continue detecting (30sec delay)
        delay(LAUNCH_DEAD_TIME);
      }
      // Else: do nothing
      break;

    case IN_AIR:
      digitalWrite(debugRed, HIGH);
      digitalWrite(debugYellow, HIGH);
      
      //check if secondary needs deployed yet (60sec since launch)
      if(millis() - launchTime > AIRBAG_DELAY_TIME){
        digitalWrite(airbagDeploy, LOW);
      }
      
      // Wait for landing 
      if( impulseDetection(LANDING_THRESH, queueSum.y) ) {
        landingCounter += 1;
        Serial.println("landing");
        if( landingCounter > 25000 ) {
          rocket_state = LANDED;
        }
      }
      else { 
        landingCounter = 0;
      }
      break;

    case LANDED:
      digitalWrite(debugRed, LOW);
      digitalWrite(debugYellow, HIGH);
      digitalWrite(airbagDeploy, HIGH);

      if( currAxis == MAIN)  {
        motorLogic(readings);
      }
      else if( currAxis == TELESCOPE) {
          Serial.println("Telescope");
          motorLogic(readings); // The inputs don't matter I think.
          currAxis = R_AXIS;
      }  
      else if( currAxis == R_AXIS ) {
        motorLogic(readings);
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
              rotationServo.write(60);
              delay(DELAY_60deg);
              rotationServo.write(90);
            }
            else if(curCommand.equalsIgnoreCase("B2")) {
              // Left 60o
              rotationServo.write(120);
              delay(DELAY_60deg);
              rotationServo.write(90);
              moveCamera = false;
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

// removed float sensorVal (which was just one of the readings)
void motorLogic(sensorReadings readings) {
  static int tilt_pos = 90;
  static int main_pos = 90;
  
  switch(currAxis) {
    case MAIN:
      if( abs(readings.y) > MAIN_EPSILON ) {
        if( readings.y - MAIN_EPSILON > 0 ) {
          main_pos += 1;
          delay(20);
        }
        if( readings.y - MAIN_EPSILON < 0 ) {
          main_pos -= 1;
          delay(20);
        }
        mainMotor.write(main_pos);
      }
      else if( abs(readings.y) < MAIN_EPSILON ) { 
        currAxis = TELESCOPE;
        mainMotor.write(90);
        Serial.println("Done leveling");
      }
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
        digitalWrite(LED_BUILTIN, millis() % 500 < 250);
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

bool impulseDetection(int thresh, float queueSum) {
  static bool prevLaunchSign = false;
  static int stateCounter = 0;

  if( (rocket_state == ON_PAD) && (queueSum > thresh) ) {
    return true;
  }
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

void cameraCommands(int camera_command) {
  Serial.println("Debug 6");
  delay(picDelay);
  switch(camera_command) {
    case 1:
      takePicture();
      break;
    case 2: // Gray
    case 3: // Color
    case 4: // Special
    case 5: // Remove Special
    case 6: // Flip 
      String message = String(camera_command) + " PICTURE ------------- ";
      Serial.println(message);
      break;
    default:
      // Command shouldn't be greater than 6
      break;
  }
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

void writeToFile(sensorReadings readings) {
  String message = "WRITE ";
  message += String(readings.x) + "," + String(readings.y) + "," + String(readings.z) + '\n';
  Serial.println(message);
}

void writeToFile(String value) {
  String message = "WRITE ";
  message += value;
}
