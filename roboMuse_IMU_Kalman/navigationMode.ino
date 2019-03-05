float angleFromIMU = 0;
float angleFromIMUYAW =0;
float angleFromIMUPITCH =0;
float angleFromIMUROLL = 0;
float angleFromIMUInRadians = 0;
float angleFromIMUYAWInRadians = 0;
float angleFromIMUPITCHInRadians = 0;
float angleFromIMUROLLInRadians = 0;
int initialCounter = 0;
float initialValue = 0;
float offsetAngleYAW = 0;
int readOnceVariable = 1;
float diffEncIMU = 0.0;
float feedbackFromIMU =0;
float newFeedbackFromIMU =0;
//float totalIMUTheta =30;
float totalIMUTheta = 0;
float deltaIMUTheta =0;
float oldFeedbackFromIMU = 0;
float newRollFromIMU = 0;
float deltaIMUROLLTheta =0;
float oldRollFromIMU =0;
float totalIMURollTheta =0;
////// Kalman Filter Variables ////////

int timeHolder = 0;
int initializationFlag = 1;
double initialThetaValue = 0;
double initialEstimateUncertainity =0;
double predictedThetaValue =0;
double originalMeasurement =0;
double originalMeasurementError =0;
double originalKalmanGain =0;
double originalCovariance = 0;
double estimatedThetaValue = 0;
double updatedCovariance =0;
int counterAddition = 0;

///////////////////////////////////////
////// Weighted Filter Variables //////
float filterSetPoint = 0;
float alpha1 , alpha2, alpha3 , alpha4 =0;
float complimentaryMeasurement1 , complimentaryMeasurement2 =0;
float filterValue =0;
float filteredTheta = 0;
///////////////////////////////////////
void navigationMode() {
  while (abs(x) <= 8000 && interApt == 100) {
    //speedRamp(1,40);
    startIMUReading(0);
    //readOnce();
    //Serial.println(initialValue);
    /*while (!Serial3.available()){
      Serial.println ("waiting");
      }*/
    if (Serial3.available()){
      angleFromIMUYAW=Serial3.parseFloat();
      angleFromIMUPITCH=Serial3.parseFloat();
      angleFromIMUROLL=Serial3.parseFloat();
      //angleFromIMUInRadians = angleFromIMUYAW;
      angleFromIMUYAWInRadians = angleFromIMUYAW * (M_PI/180); 
      angleFromIMUPITCHInRadians = angleFromIMUPITCH * (M_PI/180);
      angleFromIMUROLLInRadians = angleFromIMUROLL * (M_PI/180);
      //Serial.println(angleFromIMUInRadians);
      //Serial.println(offsetAngle);
      //Serial.println(angleFromIMUROLL);
      //Serial.println(angleFromIMUROLL);
      //Serial.println(originalTheta);
      //Serial.println(angleFromIMUYAW);
      readOnce();
      }
    //Serial.println("Im here");
    safeCheck();
    motionType = "s";
    //// try to give a ramp input ////
    //leftMotorSpeed = 10 + rampOutput;
    //rightMotorSpeed = 10 + rampOutput;
    //Serial.println(leftMotorSpeed);
    //Serial.println(rightMotorSpeed);
    //Serial.println(rampOutput);
    leftMotorSpeed = 20;
    rightMotorSpeed = 20;
    odometryCalc();
    errorDifference();
    setpoint = 0;
    
    /// imu angle increment //////
    
    newFeedbackFromIMU = (((angleFromIMUYAWInRadians)*(-1)*(180/M_PI)))+offsetAngleYAW;
    deltaIMUTheta = newFeedbackFromIMU - oldFeedbackFromIMU;
    totalIMUTheta = (totalIMUTheta + deltaIMUTheta);
    oldFeedbackFromIMU = newFeedbackFromIMU;
    //Serial.println (totalIMUTheta);
    
    //// imu angle increment/////


    /// imu roll increment ///

    newRollFromIMU = (angleFromIMUROLLInRadians)*(180/M_PI);
    deltaIMUROLLTheta = newRollFromIMU - oldRollFromIMU;
    totalIMURollTheta = totalIMURollTheta + deltaIMUROLLTheta;
    oldRollFromIMU = newRollFromIMU;
    //Serial.print(",");
    //Serial.println(totalIMURollTheta);
    //Serial.print(-1.0);
    //input = (feedbackFromIMU);
    
//    newKalmanFilter();
    slipDetection();
    weightedFilter();
    //input = originalTheta;
    input = estimatedThetaValue;
    PID_L.Compute(); PID_R.Compute();
    leftMotorSpeed += outputL;
    rightMotorSpeed -= outputR;
    saberTooth.motor(1, leftMotorSpeed);
    saberTooth.motor(2, rightMotorSpeed);
    //Serial.println(newLeftEncoderValue - (newRightEncoderValue)*(-1));
    delay(0);
    //Serial.println(originalTheta);
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
      return rampOutput;
    }
}

void startIMUReading(int detectPin){
  if (detectPin == 1){
    digitalWrite (34,HIGH);
  }
  else if(detectPin == 0){
    digitalWrite (34,LOW);
    }
  }

void readOnce(){
  if (readOnceVariable == 1 ){
    offsetAngleYAW = angleFromIMUYAWInRadians*(180/M_PI);
    readOnceVariable = 0;
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
    originalMeasurement = totalIMUTheta;
    originalMeasurementError = 0.01;
    //// Step 2 Update ////////////
    originalKalmanGain =  originalCovariance /(originalCovariance + originalMeasurementError);
    //// Step 3 Estimate //////////
    estimatedThetaValue = predictedThetaValue + (originalKalmanGain*(originalMeasurement - predictedThetaValue));
    updatedCovariance = (1-originalKalmanGain)*originalCovariance;
    //Serial.println(originalKalmanGain);
    //Serial.print(",");
    //Serial.println(estimatedThetaValue);
    //Serial.print(totalIMUTheta);
    //Serial.println(originalKalmanGain);
    //// Step 4 Predict ///////////
    predictedThetaValue  = estimatedThetaValue;
    originalCovariance = updatedCovariance;
  }*/

void weightedFilter(){
    ///// fuse the readings of both sensors ////
    //setpoint//
    filterSetPoint = 0;
    //sensor priority//
    alpha1 = 0.1; /// lesser the value, more priority to encoders
    alpha2 = 0.2;
    alpha3 = 0.5;
    alpha4 = 0.99;
    //filter//
    complimentaryMeasurement1 = originalTheta; 
    complimentaryMeasurement2 = totalIMUTheta;
    if ( abs(originalTheta - totalIMUTheta) >0.1 && abs(originalTheta - totalIMUTheta) <2 ){filterValue = alpha1;Serial.println("Low Slippage");}
    if ( abs(originalTheta - totalIMUTheta) >2 && abs(originalTheta - totalIMUTheta) <10 ){filterValue = alpha2;Serial.println("Moderate Slippage");}
    if ( abs(originalTheta - totalIMUTheta) >10 && abs(originalTheta - totalIMUTheta) <15 ){filterValue = alpha3;Serial.println("High Slippage");}
    if ( abs(originalTheta - totalIMUTheta) >15 ){filterValue = alpha4;Serial.println("Odometry Lost");}
    filteredTheta = (1-filterValue)*originalTheta + filterValue*(totalIMUTheta);
/*  Serial.print(",");
    Serial.println(originalTheta);
    Serial.print(totalIMUTheta);
    Serial.print(",");
    Serial.print(filteredTheta);*/
  }

void slipDetection(){
    /// condition 1 : when there is a change in pitch 
/*  if (abs(originalTheta - totalIMUTheta)>2){
     originalKalmanGain = 0;
      predictedThetaValue = totalIMUTheta; 
      Serial.println("Switched to IMU Navigation");
    }*/
    /// condition 2 : when there is a difference between the wheels
      
    /// condition 3 : 
  }
