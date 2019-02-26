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
float offsetAngle = 0;
int readOnceVariable = 1;
float diffEncIMU = 0.0;
float feedbackFromIMU =0;
////// Kalman Filter Variables ////////
int timeHolder = 0;
int initializationFlag = 1;
float initialThetaValue = 0;
float initialEstimateUncertainity =0;
float predictedThetaValue =0;
float originalMeasurement =0;
float originalMeasurementError =0;
float originalKalmanGain =0;
float originalCovariance = 0;
float estimatedThetaValue = 0;
float updatedCovariance =0;
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
      angleFromIMUInRadians = angleFromIMUYAW*(PI/180);
      //angleFromIMUYAWInRadians = angleFromIMUYAW*(PI/180);
      //angleFromIMUPITCHInRadians = angleFromIMUPITCH*(PI/180);
      //angleFromIMUROLLInRadians = angleFromIMUROLL*(PI/180);
      //Serial.println(angleFromIMUInRadians);
      //Serial.println(offsetAngle);
      //Serial.println(angleFromIMUROLL);
      //Serial.println(angleFromIMUROLL);
      Serial.println(originalTheta);
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
    leftMotorSpeed = 40;
    rightMotorSpeed = 40;
    odometryCalc();
    errorDifference();
    //Serial.println(y);
    //Serial.println(newLeftEncoderValue - (newRightEncoderValue)*(-1));
    //non_inc_theta = atan((newLeftEncoderValue/100 -(newRightEncoderValue)*(-1)/100)/width);
    //input = non_inc_theta;
    //input = (newLeftEncoderValue - (newRightEncoderValue)*(-1));
    //Serial.println(time);
    //input = feedbackVariable;
    //input = originalTheta;
    setpoint = 0;
    feedbackFromIMU = ((angleFromIMUInRadians)*(-1))+offsetAngle;
    input = (feedbackFromIMU);
    newKalmanFilter();
    //Serial.println(((angleFromIMUInRadians)*(-1))-2.36);
    //Serial.println(input);
    //Serial.print("\t");
    //Serial.println(originalTheta);
    //Serial.print("\t");
    //Serial.println(input - originalTheta);
    PID_L.Compute(); PID_R.Compute();
    //Serial.println(leftMotorSpeed);
    leftMotorSpeed += outputL;
    rightMotorSpeed -= outputR;
    //Serial.println(input);
    //Serial.println(outputL);
    //Serial.println(feedbackVariable);
    //time = 0;
    //Serial.println(leftMotorSpeed);
    //Serial.println("entered bitch");
    //Serial.println("Error");
    //Serial.println(newLeftEncoderValue);
    //Serial.println(newRightEncoderValue);
    //Serial.println(input);
    //Serial.println(outputL);
    //Serial.println(input);
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
    offsetAngle = angleFromIMUInRadians;
    readOnceVariable = 0;
    }
  }

void errorDifference(){
  diffEncIMU = feedbackFromIMU - originalTheta;
  //Serial.println(diffEncIMU);
  }

/*void kalmanFilter(){
    //read the angle //
    //oldThetaEstimate = feedbackFromIMU;
    if (readOnceVariable2 == 1){
     oldThetaEstimate = originalTheta;
     readOnceVariable2 = 0;
    }
    observedThetaValue = feedbackFromIMU;
    //update
    gainValue = (oldPredictionError /(oldPredictionError + rValue));
    newThetaEstimate = oldThetaEstimate + gainValue* (observedThetaValue-oldThetaEstimate);
    newPredictionError = (1-gainValue)* oldPredictionError;
    //Serial.println(newThetaEstimate);
    //Serial.print (",");
    //Serial.print(feedbackFromIMU);
    //Serial.print(",");
    Serial.print(originalTheta);
    //predict
    oldPredictionError = newPredictionError;
    oldThetaEstimate = newThetaEstimate; 
  }*/

void newKalmanFilter(){
  //after 10 miliseconds the function should be initialized
  
  ////Step 0 Initialization ////// runs only once 
    if (initializationFlag == 1 ){
      initialThetaValue = feedbackFromIMU;
      originalCovariance = 0.2;
      /////Prediction step ///////
      predictedThetaValue = initialThetaValue;
      //updatedCovariance = initialEstimateUncertainity;
      initializationFlag = 0;
      } 
    if (!Serial3.available()){
      originalKalmanGain=0;
      predictedThetaValue = originalTheta;
      }
    //// Step 1 Measurement //////// runs like a loop
    originalMeasurement = originalTheta;
    originalMeasurementError = 0.1;
    //// Step 2 Update ////////////
    originalKalmanGain =  originalCovariance /(originalCovariance + originalMeasurementError);
    //// Step 3 Estimate //////////
    estimatedThetaValue = predictedThetaValue + (originalKalmanGain*(originalMeasurement - predictedThetaValue));
    updatedCovariance = (1-originalKalmanGain)*originalCovariance;
    //Serial.println(feedbackFromIMU);
    //Serial.print(" , ");
    //Serial.print(estimatedThetaValue);
    //// Step 4 Predict ///////////
    predictedThetaValue = estimatedThetaValue;
    originalCovariance = updatedCovariance;
  }
