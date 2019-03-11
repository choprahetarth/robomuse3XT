//////////////// IF WE NEED TO CALCULATE THE THETA USING YPR //////////////////////
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

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
#define OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_TEAPOT
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
int time=0, time1=0;
int timeholder1, timeholder2 =0;
int dt = 0;
int i, counter =0;
float newAcceleration, oldAcceleration, deltaAcceleration, oldVelocity = 0 , newVelocity= 0, deltaVelocity=0, newAccelX=0, oldAccelX=0, decayedVelocityX =0 ;
float newPosition, oldPosition, deltaPosition, totalPosition = 0;
float sumAccelX =0, averageAccelX, decayedAccelX =0 , averageAccelY =0, decayedAccelY = 0, bandpassAccelY = 0, bandpassAccelX = 0;
float alpha =0.1, alpha2 = 0.13;
float setpointAccelerationX ,filteredAccelXLow =0, filteredAccelXHigh = 0,  bandpassVelocityX = 0;
int accelerationSetpoint = 0;

float newValue, alphaLow, alphaHigh, bandpassValue, filteredHigh, filteredLow, sensorReadings = 0, tValue = 0, decayFactor = 0, rValue=0, decayFactor1 =0;

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

float averagingFilter(float unAveragedFilter, int rangeOfFilter){
   float filterSum=0, finalOutput=0;
   int count;
        for (count = 0 ; count < rangeOfFilter ; count++){
             filterSum = filterSum + unAveragedFilter;
            }
    finalOutput = (filterSum / rangeOfFilter);
    return finalOutput;
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
    accelX = aaWorld.x;
    accelY = aaWorld.y;
    accelZ = aaWorld.z;
    averageAccelX = averagingFilter(accelX , 300);
    sumAccelX = 0;
    i = 0;
  }

  void emaFilter(){
      bandpassAccelX = bandpassFilter(averageAccelX, 0.1 , 0.13); ///////(rawSignal, lowerThreshold, upperThreshold)
      //bandpassAccelY = bandpassFilter(averageAccelY , 0.1, 0.13);
      decayedAccelX = decayFilter(bandpassAccelX, 0.2 , 2);///////// (bandpassedSignal, exponential function, cutoff value)
      //decayedAccelY = decayFilter(bandpassAccelY, 0.2, 2);
    }

  void velocityCalculation(){
    if (abs(decayedAccelX) < 0.2){
      accelerationSetpoint = 1;
      }

///////////////// code to be used with bandpassAccelX bandpass filter ////////////////////////
   if (accelerationSetpoint == 1 ){
    currentTime = millis();
    dt = currentTime - startTime;
    Serial.println(dt);
    newAcceleration = decayedAccelX;
    //////////////////////////////////////////////////////////////
    deltaAcceleration = ((newAcceleration + oldAcceleration)*0.5);
    //if (abs(newAcceleration) > 250){newAcceleration = oldAcceleration;}
    //Serial.println(deltaAcceleration);
    //Serial.print(",");
    //Serial.println(newAcceleration);
    //deltaAcceleration = ((newAcceleration-oldAcceleration));
    newVelocity = oldVelocity + (deltaAcceleration*dt);
    deltaVelocity = ((newVelocity + oldVelocity)*0.5);
    //deltaVelocity = newVelocity - oldVelocity;
    //Serial.print(",");
    //Serial.print(newVelocity);
    
    //////////////////////////////////////////////////////////////
    
    //bandpassVelocityX = bandpassFilter(newVelocity,0.05,0.1);
    //Serial.println(bandpassVelocityX);
    //bandpassVelocityX = bandpassFilter(deltaVelocity, 0.05, 0.1);
    //decayedVelocityX = decayFilter(bandpassVelocityX , 0.1 , 2);
    //Serial.print(",");
    //Serial.println(decayedVelocityX);
    //Serial.print(",");
    //Serial.print(bandpassVelocityX);
    //Serial.print(newPosition);
    
    //////////////////////////////////////////////////////////////
    
    //if (time> 10000){
      //newPosition = oldPosition + (bandpassVelocityX*dt) + (0.5*deltaAcceleration*dt*dt);
      //newPosition = newPosition + (bandpassVelocityX*dt);
      //newPosition = newPosition + (decayedVelocityX*dt);
      newPosition = oldPosition + (deltaVelocity*dt);
      //deltaPosition = newPosition - oldPosition;
      //}
      
    //////////////////////////////////////////////////////////////
    /////// RESET EVERYTHING HERE ////////////////////////////////
    oldAcceleration = newAcceleration;
    oldVelocity = newVelocity;
    oldPosition = newPosition;
    startTime = currentTime;
    
    /////////////////////////////////
    
    //Serial.print(",");
    //Serial.println(deltaVelocity);
    //Serial.println(bandpassVelocityX);
    //Serial.println(bandpassAccelX);
    //Serial.print(",");
    //Serial.println(deltaPosition);*/

//////////////////////////////////////////////////////////
   
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

    startTime =millis();

  
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
 //   mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
      mpu.setXGyroOffset(123);
      mpu.setYGyroOffset(61);
      mpu.setZGyroOffset(4);
      mpu.setZAccelOffset(941);
      //mpu.setXAccelOffset(-5467);
      //mpu.setYAccelOffset(786);

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
            //positionCalculation();
        #endif


        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);

    }
}
