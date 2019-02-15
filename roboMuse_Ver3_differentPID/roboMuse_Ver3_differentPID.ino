#include <Sabertooth.h>   // Header file for Sabertooth Motor Driver
#include <Encoder.h>      // Header file for the Encoders
#include <PID_v1.h>       // Header file for PID controller
#include <math.h>         // Header file for Math Calculations
#include <Plotter.h>      // Header file for Plotting the X-Y Trajectory Graph

// Safety Feature added in the Navigation Mode
int interApt =100;

/// VARIABLES FOR PLOTTING ///
//Plotter myPlot;

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
double centreIncremental = 0, x = 0, y = 0, originalTheta = 0, thetaInRadians = 0;
double oldLeftEncoderValue = 0, newLeftEncoderValue = 0;
double oldRightEncoderValue = 0, newRightEncoderValue = 0;
double leftEncoderIncrement = 0, rightEncoderIncrement = 0;
float conversionFactor = 0.1;   // rotary to linear
double leftWheelIncrement = 0, rightWheelIncrement = 0;
double width = 515;   // distance between wheels
float theta = 0;
float feedbackVariable = 0;    // Error value
//double orientation_vector[4] = {85,175,265,355};
float orientation_vector[4] = {1.55, 4.68,9.39,12.53};
float orientation_angle[4] = {1.55, 4.68,9.39,12.53};
//double orientation_angle[4] = {90,180,270,360};

// PID Variables
double minVal = -15, maxVal = 15;
double input = 0, setpoint = 0; // Input_Error_value, Controler_Output_value, Desired_Error_value
//double KpL = 75, KiL = 20, KdL = 0.1, outputL = 0; // Proportional, Integral & Derivative coefficients
//double KpR = 75, KiR = 20, KdR = 0.1, outputR = 0;  // of respective motors for PID control
double KpL = 40, KiL = 3, KdL = 0, outputL = 0; // Proportional, Integral & Derivative coefficients
double KpR = 40, KiR = 3, KdR = 0, outputR = 0;  // of respective motors for PID control
//double KpL = 40, KiL = 3, KdL = 0, outputL = 30; // PID Values if we need to move the Robot in a Trajectory.
//double KpR = 40, KiR = 3, KdR = 0, outputR = 30;  // ""

Sabertooth saberTooth(128, Serial2);  // Packetized serial mode, Non-LI, 128 bit Addr. (0,0,1,1,1,1)
Encoder enCoder_1(20, 21); // Left hand side enc., +ve value means forward
Encoder enCoder_2(2, 3);  // Right hand side enc., -ve value means forward
PID PID_L(&input, &outputL, &setpoint, KpL, KiL, KdL, P_ON_M, DIRECT); // Direct mode : Increase output to  increase input
PID PID_R(&input, &outputR, &setpoint, KpR, KiR, KdR, P_ON_M, DIRECT);



// Odometry Calculation
void odometryCalc() {
  //newLeftEncoderValue = double(enCoder_1.read());
  newLeftEncoderValue = double(enCoder_1.read());
  newRightEncoderValue = double(enCoder_2.read());
  int time = millis();
  leftEncoderIncrement = newLeftEncoderValue - oldLeftEncoderValue;
  rightEncoderIncrement = newRightEncoderValue - oldRightEncoderValue;
  //Serial.println(newRightEncoderValue);
  //Serial.println(newLeftEncoderValue);
  //Serial.print("Left wheel : ");
  //Serial.println(leftEncoderIncrement);
  //Serial.print("Right wheel : ");
  //Serial.println(rightEncoderIncrement);
  leftWheelIncrement = leftEncoderIncrement * conversionFactor; // left side advanced-by-distance
  rightWheelIncrement = rightEncoderIncrement * conversionFactor * (-1); // right side advanced-by-distance
  theta = atan((rightWheelIncrement - leftWheelIncrement) / width);
  //Serial.print("Theta : ");
  //Serial.println(theta);
  feedbackVariable = theta;
  oldLeftEncoderValue = newLeftEncoderValue;
  oldRightEncoderValue = newRightEncoderValue;
  //Serial.println(theta);
  centreIncremental = ((leftWheelIncrement + rightWheelIncrement) / 2);
  x = x + centreIncremental * cos(originalTheta + theta / 2)*(-1);
  y = y + centreIncremental * sin(originalTheta + theta / 2);
  //Serial.println(x);
  //Serial.println(centreIncremental);
  // Serial.print ("   ");
  // Serial.print(y);
  originalTheta = originalTheta + theta;
//  if (originalTheta > 6.28) {
//    originalTheta = originalTheta - 6.28;
//  }
//  else if (originalTheta < -6.28) {
//    originalTheta = originalTheta + 6.28;
//  }
//  thetaInRadians = originalTheta * (180 / 3.14);
}


// without time odometry calculation /////


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
}


//Used to calculate the turning speed

void calSpeed (double angle, double maxspeed, double theta) {
  speedTurn = (int) (maxspeed - ((maxspeed - 15) / abs(angle)) * abs(theta));
  if (angle < 0) {
    // left turn is needed
    speedTurn *= (-1);
  }
  return;
}

void setup() {
  Serial.begin(9600);   // Serial communication with rasPi
  Serial2.begin(9600);  // Serial communication with Sabertooth motor driver, default baud rate
  Serial3.begin(9600);  // Serial Communication with the Plotter

  PID_L.SetOutputLimits(minVal, maxVal);  // [Min,Max] values of output
  PID_L.SetMode(AUTOMATIC);  // Automatic = ON, Manual = OFF
  PID_R.SetOutputLimits(minVal, maxVal);
  PID_R.SetMode(AUTOMATIC);
  
  // Begin the Plotter
  //myPlot.Begin();

  // Add the X and Y Values
  //myPlot.AddXYGraph("Trajectory", 1000, "X Axis", x, "Y Axis", y);
//  myPlot.AddTimeGraph(" Relative Error". 1000, "Angle Deviation", theta); 
  
  resetCoordinates();
}

void loop() {
  Serial.println("Please choose the mode of operation -");
  Serial.println("\t m = Manual,\n\t a = Autonomous,\n\t n = Navigation.");
  while (!Serial.available()) {
    // Wait.
  }
  modeSelected = Serial.readString();
  interApt=100;
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
}
