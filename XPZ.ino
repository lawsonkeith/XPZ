/*
 * 
 * @file XPZ.ino
 *
 * K Lawson 2017
 *
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

// GPIO
#define   LED_PIN  13 
#define   BTN      12
// PWM
#define   MTR_A    10
#define   MTR_B    11
// AI
#define   KP       2
#define   KI       1 
#define   KD       0
// ESC Vals
#define PULSE_MIN   186  //1500us
#define PULSE_MAX   236  //1900
//
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
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    /*while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again*/

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    // Note - use the 'raw' program to get these.  
    // Expect an unreliable or long startup if you don't bother!!! 

    mpu.setXAccelOffset(1620);
    mpu.setYAccelOffset(1411);
    mpu.setZAccelOffset(1440);//16384
   
    mpu.setXGyroOffset(161);
    mpu.setYGyroOffset(2);
    mpu.setZGyroOffset(24);
    
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
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
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

    pinMode(MTR_A    ,OUTPUT); 
    pinMode(MTR_B    ,OUTPUT); 
    
    pinMode(BTN,INPUT_PULLUP);

    // INIT MODE
    if( digitalRead(BTN) == 0){
       Serial.println("PWM DIRECT DRIVE MODE, reset CPU to exit!");
       int pota,potb;
       
       // drive PWM of pots
       do{
         delay(500);
         Serial.print("WARNING!! SAFE VALS> ");
         Serial.print(PULSE_MIN);
         Serial.print("-");
         Serial.print(PULSE_MAX);
         pota = analogRead(KP) / 4;
         potb = analogRead(KI) / 4;
         LimitInt(&pota,0,255);
         LimitInt(&potb,0,255);
         analogWrite(MTR_A,pota);
         analogWrite(MTR_B,potb);
         Serial.print("        A> ");
         Serial.print(pota);
         Serial.print("   , B> ");
         Serial.println(potb);
       }while(1);
    }
    /* setup ESC pulse widths
    analogWrite(10,186);//1500us
    analogWrite(10,236);//1900us
    
    analogWrite(11,186);//1500us
    analogWrite(11,236);//1900us
    */
}//END Setup



//
// Main loop of program
// Repeat forever

void loop() 
{
  static int    SubLoop, Demand;
  static unsigned long   checktime;
  static byte   Moving;
  static float  Heading,HeadingTgt,oHeading=-999999;
  static double kp,ki,kd;
  static bool btn,old_btn,init=1;
  int mtr_a,mtr_b;
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
    if((millis() > checktime)){
      checktime = millis() + 2000;
      if(fabs(oHeading - Heading) < .8) {
        Serial.println("Done checking IMU!"); // ok
        init=0;
      }else{
        Serial.println(fabs(oHeading - Heading)); // err
      }
      oHeading = Heading;
    }
    // Null motors
    analogWrite(MTR_A,0);
    analogWrite(MTR_B,0);
   
  }else{  
  // =====================================================
         
    // ==== Start / stop command processor ====
    btn = digitalRead(BTN); 
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
    
    // ==== motor drive =========
    
    // Do rotation kinematics...
    // demand = +/- 1000 from PID
    // put in descent then put to 0-500 range 
    mtr_a = 150;        // fixed rate descent 0-300
    mtr_a += Demand/5; // PID action is +/-200
    
    mtr_b = 150;       
    mtr_b -= Demand/5; 
 
    // normalise to 0-50   
    mtr_a /= 10;        // put in range 0-50 (50 is PWM # controllable steps for this ESC)
    mtr_b /= 10;        
    LimitInt(&mtr_a,0,50); // hard clamp just in case
    LimitInt(&mtr_b,0,50);
 
    // impose pwr limit
    if(!Moving){
      mtr_a = 0;
      mtr_b = 0; // f
    }
    
    analogWrite(MTR_A,mtr_a + PULSE_MIN);
    analogWrite(MTR_B,mtr_b + PULSE_MIN);
  }  
  // =====================================================
   
   
  // 1Hz  blink LED to indicate activity
  if(((SubLoop % 250) == 0) && (init ==0))  { 
    kp = analogRead(KP) / 10.0;
    ki = analogRead(KI) / 1024.0;
    kd = analogRead(KD) * 100;
    Serial.print("\tHGG>  ");
    Serial.print(Heading);
    Serial.print(" , ");
    Serial.print(HeadingTgt);
    Serial.print("\tKPID>  ");
    Serial.print(kp);
    Serial.print(" , ");
    Serial.print(ki);
    Serial.print(" , ");
    Serial.print(kd);
    Serial.print("\t\tMOTOR>  ");
    Serial.print(mtr_a+PULSE_MIN);
    Serial.print(" , ");
    Serial.println(mtr_b+PULSE_MIN);
    
    
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}//END Loop



//
// A PID implementation; control an error with 3 constants and
// of 350 as a result of the motor tests.  If not moving do nothing.
void PID(float Hdg,float HdgTgt,int *Demand, double kP,double kI,double kD, byte Moving)                                                 
{
  static unsigned long lastTime; 
  static double Output, Iterm; 
  static double lastErr,error ; 

  // IF not moving then 
  if(!Moving) {
    Iterm = 0;
    lastErr = 0;
    lastTime =  millis();    
    return;
  } 
  /*How long since we last calculated*/
  unsigned long now = millis();    
  unsigned long timeChange = (float)(now - lastTime);       
  /*Compute all the working error variables*/
  error = Hdg-HdgTgt;
  if(error<180)
    error += 360;
  if(error>180)
    error -= 360; 
  
  Iterm += (kI * error * timeChange); 
  LimitDouble(&Iterm,-1000,1000);
    
  double dErr = (error - lastErr) / timeChange;       
  
  /*Compute PID Output*/
  *Demand = kP * error + Iterm + kD * dErr;  

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

}//END LimitFloat



void LimitDouble(double *x,double Min, double Max)
{
  if(*x > Max)
    *x = Max;
  if(*x < Min)
    *x = Min;

}//END LimitFloat

