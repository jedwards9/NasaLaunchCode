#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>


// File Global Variables

  Adafruit_MPU6050 mpu;

  const bool CW = 1;
  const bool CCW = 0;
  const int MAX_SPEED = 125;

  struct accelData {
    double x;
    double y;
    double z;
  };

  accelData up;
  accelData accel;
  accelData normalized;

  double oldX;
  double oldY;
  double oldZ;

  int speed;
  bool curDirection;

// Motor Pin Setup
  const int MOTOR_DIR_CW = 13;
  const int MOTOR_DIR_CCW = 12;
  const int MOTOR_PULSE_WIDTH = 11;

// Accelerometer Pin Setup
  // SDA & SCL, Power & GND

// RTC

// Radio

// Camera 



void setup() {
  setupAccel();

  up = {0,0,1};        // TODO: Make sure this is right
  accel = {0,0,0};
  normalized = {0,0,0};

  oldX = 0;
  oldY = 0;
  oldZ = 0;

  int speed = MAX_SPEED;
  bool curDirection = 1;

}

void loop() {
  setMotorValues(1,100);
  delay(2000);
  setMotorValues(0,255);
  delay(2000);
}











void setupAccel() {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

/*
  Reads accelerometer data and formats it in a struct
*/
void readAccel() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accel.x = a.acceleration.x;
  accel.y = a.acceleration.y;
  accel.z = a.acceleration.z;
  Serial.println(accel.z);
}

/*
Direction comes from accelerometer 

Speed -> 0 to 255
*/
void setMotorValues(bool direction, int speed) {
  if(direction == CW) {
    digitalWrite(MOTOR_DIR_CW, HIGH);
    digitalWrite(MOTOR_DIR_CCW, LOW);
    analogWrite(MOTOR_PULSE_WIDTH, speed);
  }
  else if(direction == CCW) {
    digitalWrite(MOTOR_DIR_CW, LOW);
    digitalWrite(MOTOR_DIR_CCW, HIGH);
    analogWrite(MOTOR_PULSE_WIDTH, speed);
  }
  // Else: Shouldn't happen
}

/*
  Calls setMotorValues and determines direction and speed
*/
void motorLogic() {
// TODO: determine shortest path to up

  // Normalize Accelerometer Data
  double length = sqrt(sq(accel.x) + sq(accel.y) + sq(accel.z));
  normalized.x = accel.x / length;
  normalized.y = accel.y / length;
  normalized.z = accel.z / length;

  // Compare normalized vector against up
  //   we want z to be as high as can be (some form of maxing function)
  //   and x about equal to y

  if(normalized.z > oldZ) {
    setMotorValues(!curDirection, speed);

    oldX = normalized.x;
    oldY = normalized.y;
    oldZ = normalized.z;
    curDirection = !curDirection;
  }
  else if(normalized.z < oldZ) {
    setMotorValues(!curDirection, speed);

    oldX = normalized.x;
    oldY = normalized.y;
    oldZ = normalized.z;
    curDirection = !curDirection;
  }
  else if(normalized.z == oldZ) {
    setMotorValues(!curDirection, 0);
  }
  // else: shouldn't happen

}


// Detect landing function...
