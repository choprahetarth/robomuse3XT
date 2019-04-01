float diffEncIMU = 0.0;
float deltaIMUTheta =0;
float newRollFromIMU = 0;
float deltaIMUROLLTheta =0;
float oldRollFromIMU =0;
float totalIMURollTheta =0;
///////// y pose estiate variables /////
int poseTimer;
float position1, position2, positionPredicted, alphaY;
float endYPose, speed1, angularSpeed, rotationTime, translationTime;
///////////////////////////////////////

////// Kalman Filter Variables ////////

int timeHolder = 0;
int initializationFlag = 1;
double initialThetaValue, initialEstimateUncertainity, predictedThetaValue, originalMeasurement,originalMeasurementError, originalKalmanGain, originalCovariance, estimatedThetaValue, updatedCovariance;
int counterAddition = 0;

///////////////////////////////////////
////// Weighted Filter Variables //////

float filterSetPoint = 0;
float alpha1 , alpha2, alpha3 , alpha4, alphaQ =0;
float complimentaryMeasurement1 , complimentaryMeasurement2 =0;
float filterValue =0, gainValue = 0;
float newFilteredTheta, oldFilteredTheta;

///////////////////////////////////////
////// Differentiation function variables ////////
unsigned long timeholder1 =0,  timeholder2=0;
unsigned long timeDiff=0;
double derivativeDeltaRoll = 0;

////////////////////////////////////////////////

//////////// Y Pose estimation /////////////////

void yPoseEstimate(){
    poseTimer = millis(); 
    Serial.println(poseTimer);
       position1 = poseCalculate(1,centreWheelVelocity);
       position2 = poseCalculate(2,centreWheelVelocity);
       positionPredicted = ((alphaY)*(position1) + (1-alphaY)*(position2));
  }

float poseCalculate(int choice, float speed1){
      angularSpeed = (speed1 / 0.259);
      rotationTime = (filteredTheta / angularSpeed );
      translationTime = (5 - rotationTime);
      if (choice == 1){
          endYPose = (0.256*(1-cos(filteredTheta)))*1000;
        }
      else if (choice == 2){
          endYPose = ((0.256*(1-cos(filteredTheta)))+(speed1*translationTime*sin(filteredTheta)))*1000;
        }
      return endYPose;
  }

///////////////////////////////////////////////

////////////////////////////////////////////////


void navigationMode() {
  while (abs(x) <= 8000 && interApt == 100) {
  //while (time < 5000 && interApt == 100) {
    time = millis();
    safeCheck();
    motionType = "s";
    imuRead();         //// to start reading the IMU 
    angleProcessing(); //// to start transmitting the angles 
    leftMotorSpeed = 30;
    rightMotorSpeed = 30;
    odometryCalc();
    velocityApproximation();
    errorDifference();
    setpoint = 0;
            //newKalmanFilter();
    slipDetection();
    input = filteredTheta;
    PID_L.Compute(); PID_R.Compute();
    leftMotorSpeed += outputL;
    rightMotorSpeed -= outputR;
    saberTooth.motor(1, leftMotorSpeed);
    saberTooth.motor(2, rightMotorSpeed);
    delay(0);

  }
  saberTooth.stop();
  delay(500);
  resetCoordinates();
  startIMUReading(1);
  readOnceVariable = 1;
}

void speedRamp(float powerOfFunction, float speedValue){
    while (rampOutput < speedValue){
      ramp = ramp + (pow(dRamp,powerOfFunction));
      rampOutput = ramp*speedValue;
      //Serial.println(rampOutput);
    }
}



void errorDifference(){
  diffEncIMU = feedbackFromIMU - originalTheta;
  //Serial.println(diffEncIMU);
  }

/*void newKalmanFilter(){
  //after 10 miliseconds the function should be initialized
    counterAddition++;
  ////Step 0 Initialization ////// runs only once 
  
    if (initializationFlag == 1 ){
      initialThetaValue = originalTheta;
      //initialThetaValue = 0;
      originalCovariance = 1;
      /////Prediction step ///////
      predictedThetaValue = initialThetaValue;
      //updatedCovariance = initialEstimateUncertainity;
      initializationFlag = 0;
      } 


   if ( initializationFlag == 1 ){
      initialThetaValue = originalTheta;
      //initialThetaValue = 0;
      originalCovariance = 0.1;
      /////Prediction step ///////
      predictedThetaValue = initialThetaValue;
      //updatedCovariance = initialEstimateUncertainity;
      initializationFlag = 0;
      } 

      
///////// code in case the imu sensor is thrown away /////  
   if (!Serial3.available()){
      originalKalmanGain=1;
      predictedThetaValue = originalTheta;
      Serial.println("Switched to Encoder Navigation");
      }
//////////////////////////////////////////////////////////      

    //// Step 1 Measurement //////// runs like a loop
    originalMeasurement = YAW;
    originalMeasurementError = 0.01;
    //// Step 2 Update ////////////
    originalKalmanGain =  originalCovariance /(originalCovariance + originalMeasurementError);
    //// Step 3 Estimate //////////
    estimatedThetaValue = predictedThetaValue + (originalKalmanGain*(originalMeasurement - predictedThetaValue));
    updatedCovariance = (1-originalKalmanGain)*originalCovariance;
    //Serial.println(originalKalmanGain);
    //Serial.print(",");
    //Serial.println(estimatedThetaValue);
    //Serial.print(YAW);
    //Serial.println(originalKalmanGain);
    //// Step 4 Predict ///////////
    predictedThetaValue  = estimatedThetaValue;
    originalCovariance = updatedCovariance;
  }*/

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
          //Serial.println(gainValue);
      }

    //filter//
    complimentaryMeasurement1 = originalTheta; 
    complimentaryMeasurement2 = YAW;
    if ( abs(originalTheta - YAW) >0.1 && abs(originalTheta - YAW) <10)      {filterValue = alphaQ; /*Serial.println("Low Slippage")*/    ;}
    else if ( abs(originalTheta - YAW) >10 )                                         {filterValue = alpha4; /*Serial.println("Odometry Lost")*/   ;}
    filteredTheta = (1-filterValue)*originalTheta + filterValue*(YAW);
    newFilteredTheta = filteredTheta;
    deltaFilteredTheta = newFilteredTheta - oldFilteredTheta;
    oldFilteredTheta = newFilteredTheta;
    Serial.print("FILTER");
    Serial.println(filteredTheta);
    //Serial.print("ENCODER");
    //Serial.print(originalTheta);
    //Serial.print("YAW");
    //Serial.print(YAW);
    //Serial.print(",");
    //Serial.print(gainValue);
    /*Serial.print("FILTERED VALUE :");
    Serial.println(filteredTheta);
    Serial.print("ENCODER THETA: ");
    Serial.println(originalTheta);
    Serial.print("IMU THETA: ");
    Serial.println(YAW);*/
  }

void slipDetection(){
    /// condition 1 : when there is a change in pitch  
        //singleDifferentiation();
    /// condition 2 : when there is a difference between the wheels
        weightedFilter();
    /// condition 3 : when the actual odometries dont match 
        // insert the INS Function once it is completed
  }

void singleDifferentiation(){
  timeholder1=millis();
  delay(1);
  timeholder2=millis();
  int a = 5;
  timeDiff = timeholder2-timeholder1;
  derivativeDeltaRoll =  deltaIMUROLLTheta / (timeDiff);
  if (derivativeDeltaRoll > (0.3) || derivativeDeltaRoll < (-0.3)){
    //Serial.print(",");
    Serial.print("PATH DISTURBANCE DETECTED");
    saberTooth.stop();
    
    }
}
