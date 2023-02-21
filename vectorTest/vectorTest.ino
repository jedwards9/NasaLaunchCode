#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "vector3.cpp";

Adafruit_MPU6050 mpu;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("I'm awake");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(500);
  Vector3 accel = readAccel();
  Serial.print(accel.x);
  Serial.print(", ");
  Serial.print(accel.y);
  Serial.print(", ");
  Serial.println(accel.z);
  
}

Vector3 readAccel(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return *( new Vector3( a.acceleration.x, a.acceleration.y, a.acceleration.z ));
}