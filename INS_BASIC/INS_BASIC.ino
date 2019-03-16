//////////////// IF WE NEED TO CALCULATE THE THETA USING YPR //////////////////////
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <microsmooth.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_WORLDACCEL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[800]; // FIFO storage buffer
//uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
uint16_t *ptr;


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

unsigned long startTime, currentTime= 0;
float currentYaw = 0;
float currentPitch = 0;
float currentRoll = 0; 
float accelX, accelY, accelZ =0;
float baseLineCorrection =0;
static char myArray[4];
int pinState = 0;
int buttonState =0;
unsigned long  time1=0, time = 0;
unsigned long timeholder1, timeholder2 =0;
unsigned long  dt = 0;
unsigned long  i, counter =0, count = 0;
float newAcceleration, oldAcceleration, deltaAcceleration, oldVelocityX = 0 , newVelocityX= 0, deltaVelocityX=0, newAccelX=0, oldAccelX=0, decayedVelocityX =0 ;
float newPositionX, oldPositionX, deltaPosition, totalPosition = 0, emaPositionX;
float sumAccelX =0, averageAccelX, decayedAccelX =0 , averageAccelY =0, decayedAccelY = 0, bandpassAccelY = 0, bandpassAccelX = 0;
float alpha =0.1, alpha2 = 0.13;
//float setpointAccelerationX ,filteredAccelXLow =0, filteredAccelXHigh = 0,  bandpassVelocityX = 0, emaVelocityX=0, finalVelocityX = 0,setPointSumVelX =0;
int accelerationSetpoint = 0;
long setpointAccelerationX ,filteredAccelXLow =0, filteredAccelXHigh = 0,  bandpassVelocityX = 0, emaVelocityX=0,setPointSumVelX =0;
float areaNewPositive, areaOldPositive, areaNewNegative, areaOldNegative =0, finalVelocityX = 0, newInputVelocityX =0 , oldInputVelocityX =0;

float newValue, alphaLow, alphaHigh, bandpassValue, filteredHigh, filteredLow, sensorReadings = 0, tValue = 0, decayFactor = 0, rValue=0, decayFactor1 =0, sgaFilter;

float bandpassFilter(float sensorReadings, float alphaLow, float alphaHigh){
      newValue = sensorReadings;
      filteredLow = (1-alphaLow)*filteredLow + alphaLow*newValue; ////ema lowpass filter
      filteredHigh = (1-alphaHigh)*filteredHigh + alphaHigh*newValue; /// ema highpass filter
      bandpassValue = filteredHigh - filteredLow;
      return bandpassValue;
  }

float decayFilter(float rawValue, float rValue, float tValue){
    decayFactor = pow(rValue,tValue-abs(rawValue));
    if (abs(rawValue)<tValue){
        rawValue = rawValue * decayFactor;
      }
    return rawValue;
  }
float exponentialMovingFilter(float sensorReading , float alphaValue){
      float averagedExpoValue = 0 ;
      averagedExpoValue = (1-alphaValue)*(averagedExpoValue) + (sensorReading)*(alphaValue);
      return averagedExpoValue;
  }

float averagingFilter(float unAveragedFilter, int rangeOfFilter){
   float filterSum=0, finalOutput=0;
   int count;
        for (count = 0 ; count < rangeOfFilter ; count++){
             filterSum = filterSum + unAveragedFilter;
            }
    finalOutput = (filterSum / rangeOfFilter);
    return finalOutput;
  }

void movementEndCheck(){
   if (abs(newAcceleration)<0.01){count++;}
   else {count = 0;}
   
   if (count>25){newVelocityX = 0; oldVelocityX= 0;} 
  }
void orientationAngleCalculation(){
  currentYaw = ypr[0] * 180/M_PI;
  currentPitch = ypr[1] * 180/M_PI;
  currentRoll = ypr[2] * 180/M_PI;
  //delay(10);
  buttonState = digitalRead(8);
  //Serial.println (buttonState);
  if (buttonState == 0){
  //Serial.println(currentYaw);
  //Serial.println(currentPitch);
  //Serial.println(currentRoll);
  //Serial.print(",");
  //Serial.println(accelX);
  //Serial.print(averageAccelX);
  //Serial.println(decayedAccelX);
  //Serial.println(decayedAccelY);

  
    }
  }

  void orientationAccelerationWorldCalculation(){
    count++;
    accelX = aaWorld.x;
    accelY = aaWorld.y;
    accelZ = aaWorld.z;
    //averageAccelX = averagingFilter(accelX , 200);
    averageAccelX = aaWorld.x;
    sumAccelX = 0;
    i = 0;

  }

  void emaFilter(){
      timeholder1 = millis();
      bandpassAccelX = bandpassFilter(averageAccelX, 0.1 , 0.13); ///////(rawSignal, lowerThreshold, upperThreshold)
      decayedAccelX = decayFilter(bandpassAccelX, 0.01 , 5);///////// (bandpassedSignal, exponential function, cutoff value)
      //sgaFilter = rdp_filter(decayedAccelX,ptr);
      Serial.print(",");
      Serial.println(decayedAccelX);
      Serial.print(bandpassAccelX);
    }

  void velocityCalculation(){
    if (abs(decayedAccelX) < 0.2 ){
      count++;
      accelerationSetpoint = 1;
      }

///////////////// code to be used with bandpassAccelX bandpass filter ////////////////////////

   if (accelerationSetpoint == 1 &&  (timeholder1 > 5000)){
    currentTime = millis();
    dt = currentTime - startTime;
    newAcceleration = decayedAccelX;
    ////////////////////////////AREA CALCULATION CODE///////////
    
    /*if (newAcceleration>0){areaNewPositive = areaOldPositive + (((newAcceleration - oldAcceleration)*dt)/2) + (oldAcceleration*dt)       ;}
    else if (newAcceleration <0){areaNewNegative = areaOldPositive + (((newAcceleration - oldAcceleration)*dt)/2) + (oldAcceleration*dt) ;}
    finalVelocityX = (areaNewPositive - areaNewNegative);*/
    
    /*Serial.println("==================================");
    Serial.print("Positive Area:  ");
    Serial.println(areaNewPositive);
    Serial.print("Negative Area:  ");
    Serial.println(areaNewNegative);
    Serial.print("Velocity:       ");
    Serial.println(finalVelocityX);
    Serial.println("================================="); */
    
    //////////////////////////////////////////////////////////
    /////////////////Velocity calculation/////////////////////
    
    deltaAcceleration = ((newAcceleration-oldAcceleration));
    newVelocityX = oldVelocityX + (oldAcceleration*dt) + ((deltaAcceleration/2)*dt);
    //movementEndCheck();
    emaVelocityX = exponentialMovingFilter( newVelocityX , 0.2 );
    decayedVelocityX = decayFilter(emaVelocityX, 0.02,2);
    
    //////////////////////////////////////////////////////////////
    /////////////////Displacement calculation/////////////////////
    
    newInputVelocityX = decayedVelocityX;
    deltaVelocityX = ((newInputVelocityX - oldInputVelocityX));
    newPositionX = oldPositionX + (oldInputVelocityX *dt) + ((deltaVelocityX/2)*dt);
    emaPositionX = exponentialMovingFilter(newPositionX, 0.2);
    //Serial.println(newPositionX/1000000);
    
    //////////////////////////////////////////////////////////////
    ////////////////// RESET EVERYTHING HERE /////////////////////
    
    oldAcceleration = newAcceleration;
    oldVelocityX = newVelocityX;
    oldInputVelocityX = newInputVelocityX;
    oldPositionX = newPositionX;
    startTime = currentTime;
    areaOldNegative = areaNewNegative;
    areaOldPositive = areaNewPositive;
    
    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////
   
      }

  }



    
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

  //////////// condition for microsmooth //////////
    ptr = ms_init(RDP);
    if(ptr == NULL) Serial.println("No memory");
    startTime =millis();
  /////////////////////////////////////////////////
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        Wire.setClock(500000);
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

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
   // while (Serial.available() && Serial.read()); // empty buffer
   // while (!Serial.available());                 // wait for data
   // while (Serial.available() && Serial.read()); // empty buffer again
    

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
     // mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
      mpu.setXGyroOffset(71);
      mpu.setYGyroOffset(45);
      mpu.setZGyroOffset(65);
      mpu.setZAccelOffset(1935);
      mpu.setXAccelOffset(-1436);
      mpu.setYAccelOffset(-2234);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first i  nterrupt..."));
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
    pinMode(LED_PIN, OUTPUT);
    pinMode(8,INPUT);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop 
          fifoCount = mpu.getFIFOCount();
        }  
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.resetFIFO();
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            orientationAngleCalculation();
        #endif
        
        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            orientationAngleCalculation();
            orientationAccelerationWorldCalculation();
            emaFilter();
            velocityCalculation();
        #endif


        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

    }
}
