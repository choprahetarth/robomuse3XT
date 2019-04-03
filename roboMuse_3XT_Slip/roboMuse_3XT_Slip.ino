#include <Sabertooth.h>   // Header file for Sabertooth Motor Driver
#include <Encoder.h>      // Header file for the Encoders
#include <PID_v1.h>       // Header file for PID controller
#include <math.h>         // Header file for Math Calculations


// Safety Feature added in the Navigation Mode
int interApt =100;

////// VARIABLES FOR THE MACHINE /////

// Flags
String modeSelected;
String motionType;

// Ramp Function Variable
float ramp = 0;
float dRamp = 0.05; 
float speedValue=0;
float rampOutput=10;

// Sabertooth arguments
float leftMotorSpeed = 0;
float rightMotorSpeed = 0;
int a = 0; double prevq = 0; float sum = 0;
float non_inc_theta = 0;

// Odometry Variables
double speedTurn = 0, prevTheta = 0;
double centreIncremental = 0, x = 0, y = 0;
double originalTheta = 0; 
double thetaInRadians = 0;
double oldLeftEncoderValue = 0, newLeftEncoderValue = 0;
double oldRightEncoderValue = 0, newRightEncoderValue = 0;
double leftEncoderIncrement = 0, rightEncoderIncrement = 0;
float conversionFactorLeft = 0.1;   // rotary to linear
//float conversionFactorRight = 0.0989;
float conversionFactorRight = 0.1;

double leftWheelIncrement = 0, rightWheelIncrement = 0;
//double width = 515;   // distance between wheels
double width = 518;
double theta = 0;
float feedbackVariable = 0;    // Error value
//double orientation_vector[4] = {85,175,265,355};
float orientation_vector[4] = {1.47, 4.50,9.24,12.45};
float orientation_angle[4] = {1.54, 4.66,9.38,12.53};
float orientation_vector_start[4] = {0,-1.55,-4.68,-9.39};
//double orientation_angle[4] = {90,180,270,360};
float originalThetaIterable = 0;


// PID Variables
double minVal = -30, maxVal = 30;
double input = 0.00, setpoint = 0.00; // Input_Error_value, Controler_Output_value, Desired_Error_value
//double KpL = 75, KiL = 20, KdL = 0.1, outputL = 0; // Proportional, Integral & Derivative coefficients
//double KpR = 75, KiR = 20, KdR = 0.1, outputR = 0;  // of respective motors for PID control
//double KpL = 30, KiL = 1, KdL = 0, outputL = 0; // Proportional, Integral & Derivative coefficients
//double KpR = 30, KiR = 1, KdR = 0, outputR = 0;  // of respective motors for PID control
//double KpL = 40, KiL = 3, KdL = 0, outputL = 0; // Proportional, Integral & Derivative coefficients
//double KpR = 40, KiR = 3, KdR = 0, outputR = 0;  // of respective motors for PID control
double KpL = 2, KiL = 0.1, KdL = 0, outputL = 0; // Proportional, Integral & Derivative coefficients
double KpR = 2, KiR = 0.1, KdR = 0, outputR = 0;  // of respective motors for PID control
//double KpL = 1, KiL = 0.1, KdL = 0, outputL = 0; // Proportional, Integral & Derivative coefficients
//double KpR = 1, KiR = 0.1, KdR = 0, outputR = 0;  // of respective motors for PID control

Sabertooth saberTooth(128, Serial2);  // Packetized serial mode, Non-LI, 128 bit Addr. (0,0,1,1,1,1)
Encoder enCoder_1(20, 21); // Left hand side enc., +ve value means forward
Encoder enCoder_2(2, 3);  // Right hand side enc., -ve value means forward
PID PID_L(&input, &outputL, &setpoint, KpL, KiL, KdL, P_ON_M, DIRECT); // Direct mode : Increase output to  increase input
PID PID_R(&input, &outputR, &setpoint, KpR, KiR, KdR, P_ON_M, DIRECT);


/////angular velocity calculation variables/// 
int startTime, currentTime, dt;
double angularVelocityLeft, angularVelocityRight, angularVelocityCentre, velocityLeft, velocityRight, velocityCentre;
double velocityLeftWheel, velocityRightWheel, centreWheelVelocity, centreWheelVelocityAngular;



// Odometry Calculation
void odometryCalc() {
  newLeftEncoderValue = double(enCoder_1.read());
  newRightEncoderValue = double(enCoder_2.read());
  leftEncoderIncrement = newLeftEncoderValue - oldLeftEncoderValue;
  rightEncoderIncrement = newRightEncoderValue - oldRightEncoderValue;
  leftWheelIncrement = leftEncoderIncrement * conversionFactorLeft; // left side advanced-by-distance
  rightWheelIncrement = rightEncoderIncrement * conversionFactorRight * (-1); // right side advanced-by-distance
  theta = atan((rightWheelIncrement - leftWheelIncrement) / width);
  theta = theta * (180/M_PI);
  feedbackVariable = theta;
  oldLeftEncoderValue = newLeftEncoderValue;
  oldRightEncoderValue = newRightEncoderValue;
  centreIncremental = ((leftWheelIncrement + rightWheelIncrement) / 2);
  x = x + centreIncremental * cos(originalTheta + theta / 2)*(-1);   ///ORIGINAL XY
  y = y + centreIncremental * sin(originalTheta + theta / 2);                
  originalTheta = originalTheta + theta;
  originalThetaIterable = originalThetaIterable + theta;
}


/// velocity approximation from encoder ticks

void velocityApproximation(){
  currentTime = millis();
  //dt = currentTime-startTime;
  dt=30;  //////DETERMINE THIS EXPERIMENTALLY BEFORE TRYING IT OUT YOURSELVES
  
  /*angularVelocityLeft = leftEncoderIncrement / dt;
  angularVelocityRight = rightEncoderIncrement / dt;
  angularVelocityCentre = centreIncremental / dt;
  
  velocityLeft = angularVelocityLeft * 63.5;     //63.5 being the total radius of the wheel in milimeters.
  velocityRight = angularVelocityRight * 63.5;
  velocityCentre = angularVelocityCentre * 63.5;*/
  
  velocityLeftWheel = leftWheelIncrement / dt;
  velocityRightWheel = rightWheelIncrement / dt;
  centreWheelVelocity = (velocityLeftWheel + velocityRightWheel)/2;
  centreWheelVelocityAngular = centreWheelVelocity/0.518; 
  //Serial.print("new velocity:");
  //Serial.println(centreWheelVelocity);
  startTime = currentTime;
  }

/// reset co ordinates ////
void resetCoordinates() {
  enCoder_1.write(0);
  enCoder_2.write(0);
  oldLeftEncoderValue=0; oldRightEncoderValue=0;
  newLeftEncoderValue=0; newRightEncoderValue=0;
  leftEncoderIncrement=0; rightEncoderIncrement=0;
  leftWheelIncrement = 0; rightWheelIncrement = 0;
  //prevTheta = 0;
  x = 0; y = 0; theta = 0; //thetaInRadians = 0;
  originalThetaIterable = 0;
}


//// Used to calculate the turning speed

void calSpeed (double angle, double maxspeed, double theta) {
  speedTurn = (int) (maxspeed - ((maxspeed - 15) / abs(angle)) * abs(theta));
  if (angle < 0) {
    // left turn is needed
    speedTurn *= (-1);
  }
  return;
}

void setup() {
  startTime = millis();
  Serial.begin(115200);   // Serial communication with rasPi
  Serial2.begin(9600);  // Serial communication with Sabertooth motor driver, default baud rate
  Serial3.begin(115200);  // Serial Communication with the Arduino Uno for I2C with IMU
  pinMode(34,OUTPUT);
  PID_L.SetOutputLimits(minVal, maxVal);  // [Min,Max] values of output
  PID_L.SetMode(AUTOMATIC);  // Automatic = ON, Manual = OFF
  PID_R.SetOutputLimits(minVal, maxVal);
  PID_R.SetMode(AUTOMATIC);
  resetCoordinates();
}

/*void loop() {
  Serial.println("Please choose the mode of operation -");
  Serial.println("\t m = Manual,\n\t a = Autonomous,\n\t n = Navigation.");
  while (!Serial.available()) {
    // Wait.
  }
  interApt=100;
  modeSelected = Serial.readString();
  if (modeSelected == "m") {
    Serial.println("Mode Selected : Manual !");
    manualMode();
  }
  else if (modeSelected == "a") {
    Serial.println("Mode Selected : Autonomous !");
    autonomousMode();
  }
  else if (modeSelected == "n") {
    Serial.println("Mode Selected : Navigation !");
    navigationMode();
  }
  else {
    Serial.println("Invalid operand !!\n");
  }
}*/

void loop() {
  //Serial.println("Please choose the mode of operation -");
  //Serial.println("\t m = Manual,\n\t a = Autonomous,\n\t n = Navigation.");
  //while (!Serial.available()) {
     //Wait.
  //}
  interApt=100;
  //modeSelected = Serial.readString();
  delay(0);
  modeSelected = "d";
  //Serial.println(modeSelected);
  if (modeSelected == "m") {
    Serial.println("Mode Selected : Manual !");
    manualMode();
  }
  else if (modeSelected == "a") {
    Serial.println("Mode Selected : Autonomous !");
    autonomousMode();
  }
  else if (modeSelected == "n") {
    Serial.println("Mode Selected : Navigation !");
    navigationMode();
  }
  else if (modeSelected == "d") {
    //Serial.println("Mode Selected : Debugging !");
    debugMode();
  }
  else {
    Serial.println("Invalid operand !!\n");
  }
}
