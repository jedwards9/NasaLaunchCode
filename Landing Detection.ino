#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ArduinoQueue.h>
Adafruit_MPU6050 mpu;

int green = 7;
float current = 0;
float past = 0;
bool firstFlag = 0;
float average = 0;
int counter = 0;

ArduinoQueue<float> valQueue(50);

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
  pinMode(green, OUTPUT);
  digitalWrite(green, LOW);
}

void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  float yVal = a.acceleration.y;
  float xVal = a.acceleration.z;
  float zVal = a.acceleration.x;
  
  if(!firstFlag){
    past = sqrt(sq(yVal) + sq(xVal) + sq(zVal));
    firstFlag = HIGH;
  }
  else{
    current = sqrt(sq(yVal) + sq(xVal) + sq(zVal));
    float diff = abs(current - past);
    past = current;
    Serial.println(diff);
    valQueue.enqueue(diff);
  }
  if(valQueue.isFull()){
    
  }

  counter = counter + 1;
  delay(50);

}

