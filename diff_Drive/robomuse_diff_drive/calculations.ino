//////odometry variables ///////////////////////////////////////////////////

double newLeftEncoderValue, newRightEncoderValue, leftEncoderIncrement, rightEncoderIncrement, leftWheelIncrement, rightWheelIncrement, theta, feedbackVariable, centreIncremental;
double x, y, originalTheta, originalThetaIterable;
double width = 518; //in millimeters

/////////////////////////////////////////////////////////////////////////////
////// filter variables /////////////////////////////////////////////////////

float alpha4, alphaQ =0;
float complimentaryMeasurement1 , complimentaryMeasurement2 =0;
float filterValue =0, gainValue = 0;
float newFilteredTheta, oldFilteredTheta, filteredThetaInRadians;

//////////////////////////////////////////////////////////////////////////////
//////// code to calculate odometry //////////////////////////////////////////

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

///// code to filter the heading angles /////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////


void weightedFilter(){
    //setpoint//
    filterSetPoint = 0;
    //sensor priority//
    alpha4 = 1;// complete priority
    alphaQ = 1*((abs(originalTheta-YAW)/(abs(originalTheta-YAW)+gainValue)));    // lesser the value of the gain Value, more priority to Encoders
    if(centreWheelVelocity < 0.20){
          gainValue = 5;
      }
    else {
          gainValue = 0.1/centreWheelVelocity;
      }

    //filter//
    complimentaryMeasurement1 = originalTheta; 
    complimentaryMeasurement2 = YAW;
    if ( abs(originalTheta - YAW) >0.1 && abs(originalTheta - YAW) <10)      {filterValue = alphaQ; /*Serial.println("Low Slippage")*/    ;}
    else if ( abs(originalTheta - YAW) >10 )                                 {filterValue = alpha4; /*Serial.println("Odometry Lost")*/   ;}
    filteredTheta = (1-filterValue)*originalTheta + filterValue*(YAW);
    newFilteredTheta = filteredTheta;
    filteredThetaInRadians = filteredTheta * (M_PI/180);
    deltaFilteredTheta = newFilteredTheta - oldFilteredTheta;
    oldFilteredTheta = newFilteredTheta;
  }

  
////// code in case the imu is thrown away ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


void plugAndPlay(){
    if (!Serial3.available()){
      filterValue = 0; /// full priority to the encoders
      Serial.println("Switched to Encoder Navigation");
      }
   }

//////////////////////////////////////////////////////////////////////////////
////// slip detection combined code //////////////////////////////////////////

void slipDetection(){
    /// condition 1 : when there is a change in pitch  
        //singleDifferentiation();x
    /// condition 2 : when there is a difference between the wheels
        weightedFilter();
    /// condition 3 : when the actual odometries dont match 
        // insert the INS Function once it is completed
    /// condition 4 : in case the imu sensor is plugged off and thrown away 
        plugAndPlay();
  }
//////////////////////////////////////////////////////////////////////////////
//////// code to approximate the velocity ///////////////////////////////////

void velocityApproximation(){
  currentTime = millis();
  //dt = currentTime-startTime;
  dt=30;  //////DETERMINE THIS EXPERIMENTALLY BEFORE TRYING IT OUT YOURSELVES
  velocityLeftWheel = leftWheelIncrement / dt;
  velocityRightWheel = rightWheelIncrement / dt;
  centreWheelVelocity = (velocityLeftWheel + velocityRightWheel)/2;
  centreWheelVelocityAngular = centreWheelVelocity/0.518; 
  //Serial.print("new velocity:");
  //Serial.println(centreWheelVelocity);
  startTime = currentTime;
  }
