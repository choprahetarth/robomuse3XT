float angleFromIMU = 0;
float angleFromIMUYAW =0;
float angleFromIMUPITCH =0;
float angleFromIMUROLL = 0;
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
//float totalIMUYAW =30;
float totalIMUYAW = 0;
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
    //speedRamp(1,40);
    startIMUReading(0);
    //Serial.println(initialValue);
    if (Serial3.available()){
      angleFromIMUYAW=Serial3.parseFloat();
      angleFromIMUPITCH=Serial3.parseFloat();
      angleFromIMUROLL=Serial3.parseFloat();
      angleFromIMUYAWInRadians = angleFromIMUYAW * (M_PI/180); 
      angleFromIMUPITCHInRadians = angleFromIMUPITCH * (M_PI/180);
      angleFromIMUROLLInRadians = angleFromIMUROLL * (M_PI/180);
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
    leftMotorSpeed = 30;
    rightMotorSpeed = 30;
    odometryCalc();
    errorDifference();
    setpoint = 0;
///////////imu angle increment////////////////////////
    newFeedbackFromIMU = (((angleFromIMUYAWInRadians)*(-1)*(180/M_PI)))+offsetAngleYAW;
    deltaIMUTheta = newFeedbackFromIMU - oldFeedbackFromIMU;
    totalIMUYAW = (totalIMUYAW + deltaIMUTheta);
    oldFeedbackFromIMU = newFeedbackFromIMU;
    //Serial.println (offsetAngleYAW);
    //Serial.println (totalIMUYAW);
//////////////////////////////////////////////////////
//////////////// imu roll increment //////////////////
    newRollFromIMU = (angleFromIMUROLLInRadians)*(180/M_PI);
    deltaIMUROLLTheta = newRollFromIMU - oldRollFromIMU;
    totalIMURollTheta = totalIMURollTheta + deltaIMUROLLTheta;
    oldRollFromIMU = newRollFromIMU;
    //Serial.print(",");
    //Serial.println(deltaIMUROLLTheta);
    //Serial.print(-1.0);
///////////////////////////////////////////////////////    
//  newKalmanFilter();
//  input = (feedbackFromIMU);
    slipDetection();
    //input = originalTheta;
    //input = estimatedThetaValue;
    input = filteredTheta;
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
    ///// fuse the readings of both sensors ////
    //setpoint//
    filterSetPoint = 0;
    //sensor priority//
    alpha4 = 1;// complete priority
    alphaQ = 1*((abs(originalTheta-totalIMUYAW)/(abs(originalTheta-totalIMUYAW)+5)));// lesser the value of the alpha, more priority to IMU
    //filter//
    complimentaryMeasurement1 = originalTheta; 
    complimentaryMeasurement2 = totalIMUYAW;
    if ( abs(originalTheta - totalIMUYAW) >0.1 && abs(originalTheta - totalIMUYAW) <15)      {filterValue = alphaQ; /*Serial.println("Low Slippage")*/    ;}
    else if ( abs(originalTheta - totalIMUYAW) >15 )                                         {filterValue = alpha4; /*Serial.println("Odometry Lost")*/   ;}
    filteredTheta = (1-filterValue)*originalTheta + filterValue*(totalIMUYAW);
    Serial.print(",");
    Serial.println(originalTheta);
    Serial.print(totalIMUYAW);
    Serial.print(",");
    Serial.print(filteredTheta);
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
  timeDiff = timeholder2-timeholder1;
  derivativeDeltaRoll =  deltaIMUROLLTheta / (timeDiff);
  //Serial.print(",");
  //Serial.println(derivativeDeltaRoll);
  //Serial.print(0.1);
  if (derivativeDeltaRoll>0.1 || derivativeDeltaRoll < (-0.1)){
    Serial.println("SlipDetected");
    }
}
