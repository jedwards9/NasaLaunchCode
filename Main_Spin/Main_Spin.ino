#include <ArduinoQueue.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include "uRTCLib.h";
//#include "Globals.h";

//False is CW, true is CCW with motor facing away from you
bool spinDir = false;


int dir1 = A0;
int dir2 = A1;

void setup() {
  // put your setup code here, to run once:
  pinMode(A7, INPUT);
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(10, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(dir1, !spinDir);
  digitalWrite(dir2, spinDir);
  analogWrite(10, 60);
}
