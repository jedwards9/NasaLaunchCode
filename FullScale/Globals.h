/**  Pin Definitions  **/
const int telescopePin = 6;
const int swivelPin = 5;
const int tiltPin = 9;
const int mainPWM = 3;
const int button2 = 11;
const int button1 = 12;
// const int mainDir2 = A0;
// const int mainDir1 = A1;
const int debugRed = A1;
const int debugYellow = A2;
const int airbagDeploy = 8;

/** Hardware declarations **/
Adafruit_MPU6050 mpu;
uRTCLib rtc(0x68);
Servo tiltServo;
Servo rotationServo;
Servo telescopeServo;
Servo mainMotor;

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

/**  Program Constants  **/  
const int SAMPLE_PERIOD = 50;
const int MAX_QUEUE_SIZE = 25;
const int ESP_DELAY = 1000;
const int DELAY_60deg = 500;
const int DELAY_PIC = 5000;
const int LAUNCH_DEAD_TIME = 10000; //180000 miliseconds for 3 mins
const unsigned int AIRBAG_DELAY_TIME = 10000;
const int picDelay = 2000;

// might want different speeds for the different axes
const int MIN_ROTATION_SPEED = 70;     // Rotation speed for main axis
const int MAX_ROTATION_SPEED = 255;

/**  Threshold Values  **/
const float INITIAL_THRESH = 1; // Main axis threshold
const float LANDING_THRESH = 0.7 * MAX_QUEUE_SIZE;     // Landing movement threshold
const int LAUNCH_THRESH = -7 * MAX_QUEUE_SIZE;          // Launch movement threshold
const float MAIN_EPSILON = 0.2;                        // Float comparison error threshold for MAIN leveling 
const float TILT_EPSILON =  0.25;                      // Float comparison error threshold for TILT leveling

/** Picture Commands **/
const int TAKE_PICTURE = 1;
const int TO_GRAY = 2;
const int TO_COLOR = 3;
const int APPLY_SPECIAL = 4;
const int REMOVE_FILTER = 5;
const int FLIP_180 = 6;

/**  Hall of Shame  **/
flightStage rocket_state;
levelAxis currAxis;
ArduinoQueue<sensorReadings> data(MAX_QUEUE_SIZE);
unsigned long launchTime;
bool moveCamera;
int landingCounter;