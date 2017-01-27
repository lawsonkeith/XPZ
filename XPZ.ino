/*
 * 
 * @file BBot.ino
 *
 * K Lawson 2015
 *
 * Bee Bot / Big Trak robot clone.
 *
 * A programmable robot using 9110 H Bridge and MPU 6050 IMU
 * The robot can be programmed with a keypad and follows a 
 * programmed path.  The IMU prevents drift and allows for
 * precise direction changes without encoders. 
 * A sharp IR range finder
 *
 */

//
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

//
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;


//
//   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
//   depends on the MPU-6050's INT pin being connected to the Arduino's
//   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
//   digital I/O pin 2.


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//
// Define commands that can be executed, first 2 are internally generated
#define NONE 0
#define IMU_WARM 1
#define CRASH 2
#define CANCEL 3

#define UP 5
#define DN 6
#define LEFT 7
#define RIGHT 8
#define GO 9

//
// ISR
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


//
// Setup

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
   
    // initialize device
    Serial.println(F("Initializing I2C devices...f="));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    /*while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again*/

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    // Note - use the 'raw' program to get these.  
    // Expect an unreliable or long startup if you don't bother!!! 
    mpu.setXGyroOffset(66);
    mpu.setYGyroOffset(25);
    mpu.setZGyroOffset(45);
    mpu.setXAccelOffset(-1640);
    mpu.setYAccelOffset(1240);
    mpu.setZAccelOffset(1200);
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT); //LED

    pinMode(11,OUTPUT); //spkr
    pinMode(10,OUTPUT); //PWM Ctrl Dir
    
    pinMode(12,INPUT_PULLUP);

}//END Setup



//
// Main loop of program
// Repeat forever

void loop() 
{
  static int    SubLoop, Demand;
  static long   checktime;
  static byte   Moving;
  static float  Heading,HeadingTgt,oHeading,kp,ki,kd;
  static byte Command;
  float Reduction;
  static bool btn,old_btn,init=1;
  int i;
  SubLoop++;

    
  // if programming failed, don't try to do anything
  if (!dmpReady) return;  

  // *********************************************************************
  // ** 100Hz Fast Loop                                                 **
  // *********************************************************************
  
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
     
  }
  
  // =====================================================
  // ==== WARMUP LOOP 
  // =====================================================
  if(init){
    GetHeading(&Heading);        
    // check it's stabilised
    if(millis() > checktime){
      checktime += 4000;
      if(fabs(oHeading - Heading) < .8) {
        Serial.println("Done checking IMU!"); // ok
        init=0;
      }else{
        Serial.println(fabs(oHeading - Heading)); // err
      }
      oHeading = Heading;
    }
   
  }else{  
  // =====================================================
         
    // ==== Start / stop command processor ====
    btn = digitalRead(12); 
    if((btn == 0) && (old_btn == 1) ) {
      if(Moving)
        HeadingTgt+=30;
      else
        HeadingTgt = Heading;
        
      Moving = 1;
    }
    old_btn = btn;
    
    // ==== main app      					
    GetHeading(&Heading);               					// Get heading        
    PID(Heading,HeadingTgt,&Demand,kp,ki, kd  ,Moving);	// If not moving zero integral
    
    // Neet thrus calculation
    analogWrite(10,Demand / 4);
    analogWrite(11,Demand / 4);
  }  
  // =====================================================
   
   
  // 1Hz  blink LED to indicate activity
  if(((SubLoop % 100) == 0) && (init ==0))  { 
    kp = analogRead(2) / 10.0;
    ki = analogRead(1) / 100.0;
    kd = analogRead(0);
    Serial.print(">> ");
    Serial.print(Heading);
    Serial.print(" , ");
    Serial.print(kp);
    Serial.print(" , ");
    Serial.print(ki);
    Serial.print(" , ");
    Serial.print(kd);
    Serial.print(" , ");
    Serial.print(Moving);
    Serial.print(" , ");
    Serial.println(HeadingTgt);
    
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}//END Loop




//
// Check to see if the IMU has settled down and is giving a steady heading.  
// If it hasn't disable the go button.

void CheckIMU(byte *Command, float  Heading)
{
  static int Init=1,Count;
  static float oHeading;
  
  if(Init)
  {
      //If IMU not stable don't allow the robot to start navigating.
      if(*Command == GO)
        *Command = NONE;
      
      Count++;
      if(Count==100)
      {
        Count = 0; 
        if(abs(Heading - oHeading) < 1)
        {
          Init = 0;    
           *Command = IMU_WARM;
        }
        else
          oHeading = Heading;
      }  
  }
}//END Check IMU

//
// A PID implementation; control an error with 3 constants and
// of 350 as a result of the motor tests.  If not moving do nothing.

void PID(float Hdg,float HdgTgt,int *Demand, float kP,float kI,float kD, byte Moving)                                                 
{
  static unsigned long lastTime; 
  static float Output; 
  static float errSum, lastErr,error ; 

  // IF not moving then 
  if(!Moving)
  {
        errSum = 0;
        lastErr = 0;
        return;
  }

  //error correction for angular overlap
  error = Hdg-HdgTgt;
  if(error<180)
    error += 360;
  if(error>180)
    error -= 360;
  
      
  //http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

  /*How long since we last calculated*/
  unsigned long now = millis();    
  float timeChange = (float)(now - lastTime);       
  /*Compute all the working error variables*/
  //float error = Setpoint - Input;    
  errSum += (error * timeChange);   

  //integral windup guard
  LimitFloat(&errSum, -300, 300);

  float dErr = (error - lastErr) / timeChange;       

  /*Compute PID Output*/
  *Demand = kP * error + kI * errSum + kD * dErr;       
  /*Remember some variables for next time*/
  lastErr = error;    
  lastTime = now; 

  //limit demand 
  LimitInt(Demand, -1000, 1000);

}//END getPID


//
// Use the IMU to get the curreent heading.  This is 0-360 degrees.
// It's not relative to North but where the robot was pointing when the
// GO button was pressed.

void  GetHeading(float *Heading)               
{

  {
      //calc heading from IMU
      // reset interrupt flag and get INT_STATUS byte
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();

      // get current FIFO count
      fifoCount = mpu.getFIFOCount();

      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } 
      else if (mpuIntStatus & 0x02) 
      {
          // wait for correct available data length, should be a VERY short wait
          while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

          // read a packet from FIFO
          mpu.getFIFOBytes(fifoBuffer, packetSize);
        
          // track FIFO count here in case there is > 1 packet available
          // (this lets us immediately read more without waiting for an interrupt)
          fifoCount -= packetSize;
          
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          //Serial.print("ypr\t");
          *Heading = (ypr[0] * 180/M_PI) + 180;
  
      }//done
  }
  
 
}//END GetHeading



//
//  LimitInt
//  Clamp an int between a min and max.  

void LimitInt(int *x,int Min, int Max)
{
  if(*x > Max)
    *x = Max;
  if(*x < Min)
    *x = Min;

}//END LimitInt

//
// Clamp a float between a min and max.  Note doubles are the same 
// as floats on this platform.

void LimitFloat(float *x,float Min, float Max)
{
  if(*x > Max)
    *x = Max;
  if(*x < Min)
    *x = Min;

}//END LimitInt


//
// Drive port / stbd motors fwd or backwards using PWM.
// A breakout calc is needed to linearise the response since
// torque is proportional to voltage on a DC mottor the wheels
// don't move on the lower 25% range.
// 
// A L9110 IC controls the motors and 2 PWMs are used.  2 DOs control 
// direction.
// 
// IA(DO) IB(PWM) Motor State 
// L      L       Off 
// H      L       Forward 
// L      H       Reverse 
// H      H       Off 
//
// Inputs
// -----
// DriveVal    +/-1000  
//
// Note
// ----
// Demand>     0  100 200 300 400 500 600 700 800 900 1000
// Distance>  33  70  93  110 128 140 151 164* 168 168 168
//
// Motor demands are linear ish up to 700 with AAA hybrio batteries.
// For PID Distance = 400 and PID gets 300
//
void DriveMotors(int PDrive,int SDrive,byte Moving)
{
  int Mag;

  if(!Moving)
  {
    PDrive = 0;
    SDrive = 0;
  }
  
  LimitInt(&PDrive, -1000,1000);
  LimitInt(&SDrive, -1000,1000);

  // ========= Port drive =========
  Mag = abs(PDrive) * .205 + 45;
 if(PDrive == 0) Mag = 0;
  
  if(PDrive < 0)
  {
    // fwds
    digitalWrite(9, HIGH);
    analogWrite(3, 255 - Mag);
  }
  else
  {
    // backwards
    digitalWrite(9, LOW);
    analogWrite(3, Mag);
  } 

  // ========= Stbd drive =========
  Mag = abs(SDrive) * .205 + 45;
  if(SDrive == 0) Mag = 0;
  
  if(SDrive < 0)
  {
    // 
    digitalWrite(10, HIGH);
    analogWrite(11, 255 - Mag); //you haave to do this; look at the truth table
  }
  else
  {
    // fwd
    digitalWrite(10, LOW);
    analogWrite(11, Mag);
  }  
}//END DriveMotors

//
// Check if a b & c are in range
// This is a comparisson function for cleaning up analog inputs

byte InRange(int hi, int lo, int a, int b, int c)
{
  if(a<hi && a>lo)
    if(b<hi && b>lo)
      if(c<hi && c>lo)
        return(1);

  return(0);
}//END InRange



