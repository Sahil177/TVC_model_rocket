#include <Servo.h>
Servo servo1;
Servo servo2;
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
volatile uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
volatile uint16_t fifoCount;     // count of all bytes currently in FIFO
volatile uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
volatile static float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//PID variables
double kp = 0.16;
double ki= 0.0001;
double kd = 0.01;
double setpoint = 0;
double errorY, errorP;
double TVC_max = 5*M_PI/180;
double gim_ratio = 4/1.2;
double errorRateY, errorRateP;
double T = 1.0/50; // time period
static double lastErrorY =0  , lastErrorP =0 ;
static double cumErrorY=0, cumErrorP=0;
static double phiY =0 , phiP =0;

void isr(){
  servo1.write(phiY*180/M_PI+90); // servo update
  servo2.write(phiP*180/M_PI+90);
  
  //Serial.println("************");
  //Serial.print("phiY ");
  //Serial.println(phiY);
  //Serial.print("phiP ");
  //Serial.println(phiP);
  //Serial.print("errorRate ");
  //Serial.println(errorRateY);
  //Serial.print("cumerror ");
  //Serial.println(cumErrorY);
  SPCR = (1 << SPIE); // enable SPI interupt
  sei(); // enable global interupts
  mpu.getFIFOBytes(fifoBuffer, packetSize); 
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  cli(); // disable global interupts
  SPCR = (0 << SPIE); // disable SPI interupt

  errorY = ypr[1] - setpoint;
  errorP = ypr[2] - setpoint;

  errorRateY = (errorY -lastErrorY)/T;
  errorRateP = (errorP - lastErrorP)/T;
  
  cumErrorY += ((lastErrorY + errorY)/2)*T;
  cumErrorP += ((lastErrorP + errorP)/2)*T ;

  phiY = kp*errorY+kd*errorRateY+ki*cumErrorY;
  phiP = kp*errorP+kd*errorRateP+ki*cumErrorP;

  if (phiY > TVC_max){
    phiY = TVC_max;
  }
  else if ( phiY < -TVC_max){
    phiY = -TVC_max;
  }

  if (phiP > TVC_max){
    phiP = TVC_max;
  }
  else if ( phiP < -TVC_max){
    phiP = -TVC_max;
  }

  phiY = phiY*gim_ratio;
  phiP = phiP*gim_ratio;

  lastErrorY = errorY;
  lastErrorP = errorP;

}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    servo1.attach(5);
    servo2.attach(3);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-53);
    mpu.setYGyroOffset(6);
    mpu.setZGyroOffset(98);
    mpu.setZAccelOffset(1777); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        delay(2);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), isr, RISING);
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}



void loop() {
}
