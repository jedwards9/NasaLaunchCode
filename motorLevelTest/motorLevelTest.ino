#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

int PWMPin = 11;
int dirPin1 = 12;
int dirPin2 = 13;
float threshVal = 0.5;
float smoothVal = 0.8;
float runVal = 0;

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
}

void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float readVal = a.acceleration.y;
  float runVal = (readVal * smoothVal) + (runVal * (1-smoothVal));
  if(abs(runVal) > threshVal){
    motorWrite(runVal < 0, 20*abs(runVal));
  }
  else{
    motorWrite(false, 0);
  }
  Serial.println(runVal);
  delay(100);
}

void motorWrite(bool dir, int speed){
  digitalWrite(dirPin1, dir);
  digitalWrite(dirPin2, !dir);
  analogWrite(PWMPin, speed);
}
