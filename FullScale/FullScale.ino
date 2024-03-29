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
  pinMode(hotWirePin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  digitalWrite(airbagDeploy, HIGH);
  digitalWrite(hotWirePin, LOW);

  // RTC setup 
  uRTCLib rtc(0x68);
  URTCLIB_WIRE.begin();

  // MPU setup 
  Serial.begin(115200);
  if (!mpu.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      tone(buzzerPin, 1397, 230);
      delay(250);
      noTone(buzzerPin);
      tone(buzzerPin, 1480, 230);
      delay(250);
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
  tiltServo.write(90);          // slight tilt so the head doesn't catch the wind.
  rotationServo.write(90);
  telescopeServo.write(90);
  mainMotor.write(90);

  // Initializing queue with 0s
  for(int i = 0; i < MAX_QUEUE_SIZE; i++) {
    data.enqueue(sensorReadings{0,0,0});
  }

  // Variable initialization 
  rocket_state = ON_PAD;          // Stage counter 
  currAxis = MAIN;
  hasRunDefaultCommands = false;
  picNumber = 1;
  playSong();
}

void loop() {
  static sensorReadings readings = {0,0,0};
  delay(SAMPLE_PERIOD);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  readings = { a.acceleration.x, a.acceleration.y, a.acceleration.z };
  if(rocket_state == IN_AIR) {
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
       // delay(LAUNCH_DEAD_TIME);
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
      if(millis() - launchTime > LAUNCH_DEAD_TIME){
        rocket_state = LANDED;
        landedTime = millis();
      }
      break;

    case LANDED:
      digitalWrite(airbagDeploy, HIGH);
      if( currAxis == MAIN)  {
        digitalWrite(debugRed, LOW);
        digitalWrite(debugYellow, HIGH);
        motorLogic(readings);
      }
      else if( currAxis == TELESCOPE) {
          tiltServo.write(90);  //Resets tilt before popping out so it doesn't hit the tube.
          delay(50);
          Serial.println("Telescope");
          motorLogic(readings); // The inputs don't matter I think.
          // currAxis = R_AXIS;
          currAxis = LEVELED;
      }  
      else if( currAxis == R_AXIS ) {
        motorLogic(readings);
      }
      else if ( currAxis == LEVELED ) {
        digitalWrite(debugRed, HIGH);
        digitalWrite(debugYellow, LOW);
        String comStr = getRadioData();
        for(int i = 0; i < (comStr.length() - 2); i += 2) {
          String curCommand = comStr.substring(i,i+2);
          Serial.println(curCommand);
            if(curCommand.equalsIgnoreCase("A1")) {
              // Right 60o
              rotationServo.write(75);
              delay(DELAY_60deg);
              rotationServo.write(90);
            }
            else if(curCommand.equalsIgnoreCase("B2")) {
              // Left 60o
              rotationServo.write(105);
              delay(DELAY_60deg);
              rotationServo.write(90);
            }
            else if(curCommand.equalsIgnoreCase("C3")) {
              // Take picture
              cameraCommands(TAKE_PICTURE);
              delay(DELAY_PIC);
            }
            else if(curCommand.equalsIgnoreCase("D4")) {
              // Change camera mode from color to grayscale
              cameraCommands(TO_GRAY);
              delay(DELAY_PIC);
            }
            else if(curCommand.equalsIgnoreCase("E5")) {
              // Change camera mode from grayscale to color
              cameraCommands(TO_COLOR);
              delay(DELAY_PIC);
            }
            else if(curCommand.equalsIgnoreCase("F6")) {
              // Rotate image 180o
              cameraCommands(FLIP_180);
              delay(DELAY_PIC);
            }
            else if(curCommand.equalsIgnoreCase("G7")) {
              // Apply special filter
              cameraCommands(APPLY_SPECIAL);
              delay(DELAY_PIC);
            }
            else if(curCommand.equalsIgnoreCase("H8")) {
              // Remove all filters
              cameraCommands(REMOVE_FILTER);
              delay(DELAY_PIC);
            }
        }
          while(true) { //Woohoo!!! All done!!
            digitalWrite(debugRed, millis() % 250 < 125);
            digitalWrite(debugYellow, millis() % 250 > 125);
      
            if( (millis() - landedTime > RADIO_FAILURE_TIME) && (!hasRunDefaultCommands) ) {
              defaultCameraCommands();
            }  
          } 
      }

      break;
    default:
      break;
  }
}

String getRadioData(){
  return "A1C3B2C3F6G7C3D3";
}

void motorLogic(sensorReadings readings) {
  static int tilt_pos = 90;
  static int main_pos = 90;
  
  switch(currAxis) {
    case MAIN:
      if( abs(readings.x) > MAIN_EPSILON ) {
        if( readings.x - MAIN_EPSILON > 0 ) {
          main_pos = 155;
          delay(20);
        }
        if( readings.x - MAIN_EPSILON < 0 ) {
          main_pos = 25;
          delay(20);
        }
        mainMotor.write(main_pos);
      }
      if( abs(readings.x) < MAIN_EPSILON ) { 
        currAxis = TELESCOPE;
        mainMotor.write(90);
        Serial.println("Done leveling");
      }
      break;
    
    case TELESCOPE:
      Serial.println("Telescope");
      // for (int i = 0; i <= 180; i += 5) {
      //   telescopeServo.write(i);
      //   delay(20);
      // }
      
      // Continuous servo code
      delay(1000);
      digitalWrite(hotWirePin, HIGH);
      delay(hotWireDelay);
      telescopeServo.write(100);
      delay(telescopeDelay);
      telescopeServo.write(90);
      delay(1000);
      digitalWrite(hotWirePin, LOW);

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
      }
      break;

    case LEVELED:
      //should never get called
      break;

    default:
      break;
  }
}

bool impulseDetection(int thresh, float queueSum) {
  static bool prevLaunchSign = false;
  static int stateCounter = 0;

  if( (rocket_state == ON_PAD) && (queueSum < thresh) ) {
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
  String message;
  switch(camera_command) {
    case 1:
      takePicture();
      break;
    case 2: // Gray
      message = String(camera_command) + " PIC ------------- ";
      Serial.println(message);
    case 3: // Color
      message = String(camera_command) + " PIC ------------- ";
      Serial.println(message);
    case 4: // Special
      message = String(camera_command) + " PIC ------------- ";
      Serial.println(message);
    case 5: // Remove Special
      message = String(camera_command) + " PIC ------------- ";
      Serial.println(message);
    case 6: // Flip 
      message = String(camera_command) + " PIC ------------- ";
      Serial.println(message);
      break;
    default:
      // Command shouldn't be greater than 6
      break;
  }
}

void takePicture(){
  rtc.refresh();
    
  String message = "1 PIC Date_";
  message += String(rtc.month());
  message += "_";
  message += String(rtc.day());
  message += "_";
  message += String(rtc.year());
  message += " Time_ ";
  message += String(rtc.hour());
  message += ";";
  message += String(rtc.minute());
  message += ";";
  message += String(rtc.second());
  message += " ";
  message += String(picNumber);

  Serial.println(message);
  picNumber += 1;
}

void playSong(){
  int tempo = 80;
  int melody[] = {
    587,1,//D5
    880,1,//A5
    1175,1,//D6
    0,8,//REST
    1480,8,//FS6
    1397,-1//F6
  };
  int notes = sizeof(melody) / sizeof(melody[0]) / 2;
  int wholenote = (60000 * 2) / tempo;
  int divider = 0, noteDuration = 0;

  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    // calculates the duration of each note
    divider = melody[thisNote + 1];
    if (divider > 0) {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    // we only play the note for 90% of the duration, leaving 10% as a pause
    tone(buzzerPin, melody[thisNote], noteDuration*0.9);

    // Wait for the specief duration before playing the next note.
    delay(noteDuration);
    
    // stop the waveform generation before the next note.
    noTone(buzzerPin);
  }
}

void defaultCameraCommands() {
  rotationServo.write(120);
  delay(DELAY_60deg);
  delay(DELAY_60deg);
  rotationServo.write(90);

  for(int i = 0; i < 6; i++) {
    rotationServo.write(60);
    delay(DELAY_60deg);
    rotationServo.write(90);
    // Basic picture
    cameraCommands(TAKE_PICTURE);
    cameraCommands(TO_GRAY);
    delay(500);
    // Gray picture
    cameraCommands(TAKE_PICTURE);
    cameraCommands(APPLY_SPECIAL);
    delay(500);
    // Special filter
    cameraCommands(TAKE_PICTURE);
    cameraCommands(REMOVE_FILTER);
    cameraCommands(FLIP_180);
    delay(500);
    // Flipped
    cameraCommands(TAKE_PICTURE);
    cameraCommands(TO_GRAY);
    delay(500);
    // Gray flipped
    cameraCommands(TAKE_PICTURE);
    cameraCommands(APPLY_SPECIAL);
    delay(500);
    // Special flipped 
    cameraCommands(TAKE_PICTURE);
    delay(750);
  }

  hasRunDefaultCommands = true;
}