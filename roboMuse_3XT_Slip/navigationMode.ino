float diffEncIMU = 0.0;
//float totalIMUYAW =30;
float totalIMUYAW = 0;
float deltaIMUTheta =0;
float newRollFromIMU = 0;
float deltaIMUROLLTheta =0;
float oldRollFromIMU =0;
float totalIMURollTheta =0;

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
float filterValue =0;
float filteredTheta = 0;

///////////////////////////////////////
////// Differentiation function variables ////////
unsigned long timeholder1 =0,  timeholder2=0;
unsigned long timeDiff=0;
double derivativeDeltaRoll = 0;

////////////////////////////////////////////////


void navigationMode() {
  while (abs(x) <= 8000 && interApt == 100) {
    safeCheck();
    motionType = "s";
    imuRead();         //// to start reading the IMU 
    angleProcessing(); //// to start transmitting the angles 
    leftMotorSpeed = 30;
    rightMotorSpeed = 30;
    odometryCalc();
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
    originalMeasurement = totalIMUYAW;
    originalMeasurementError = 0.01;
    //// Step 2 Update ////////////
    originalKalmanGain =  originalCovariance /(originalCovariance + originalMeasurementError);
    //// Step 3 Estimate //////////
    estimatedThetaValue = predictedThetaValue + (originalKalmanGain*(originalMeasurement - predictedThetaValue));
    updatedCovariance = (1-originalKalmanGain)*originalCovariance;
    //Serial.println(originalKalmanGain);
    //Serial.print(",");
    //Serial.println(estimatedThetaValue);
    //Serial.print(totalIMUYAW);
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
    alphaQ = 1*((abs(originalTheta-totalIMUYAW)/(abs(originalTheta-totalIMUYAW)+5)));    // lesser the value of the alpha, more priority to IMU
    //filter//
    complimentaryMeasurement1 = originalTheta; 
    complimentaryMeasurement2 = totalIMUYAW;
    if ( abs(originalTheta - totalIMUYAW) >0.1 && abs(originalTheta - totalIMUYAW) <15)      {filterValue = alphaQ; /*Serial.println("Low Slippage")*/    ;}
    else if ( abs(originalTheta - totalIMUYAW) >15 )                                         {filterValue = alpha4; /*Serial.println("Odometry Lost")*/   ;}
    filteredTheta = (1-filterValue)*originalTheta + filterValue*(totalIMUYAW);
  }

void slipDetection(){
    /// condition 1 : when there is a change in pitch  
        singleDifferentiation();
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
  //Serial.println(derivativeDeltaRoll);
  //Serial.print(",");
  //Serial.println(derivativeDeltaRoll);
  //Serial.print(0.1);
  if (derivativeDeltaRoll > (0.3) || derivativeDeltaRoll < (-0.3)){
    //Serial.print(",");
    Serial.print("PATH DISTURBANCE DETECTED");
    saberTooth.stop();
    
    }
}
