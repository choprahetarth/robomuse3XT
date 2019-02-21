float angleFromIMU = 0;
float angleFromIMUInRadians = 0;
int initialCounter = 0;
float initialValue = 0;
void navigationMode() {
  while (abs(x) <= 10000 && interApt == 100) {
    //speedRamp(1,40);
    startIMUReading(0);
    //readOnce();
    //Serial.println(initialValue);
    while (!Serial3.available()){
      Serial.println ("waiting");
      }
    if (Serial3.available()){
      angleFromIMU=Serial3.parseFloat();
      angleFromIMUInRadians = angleFromIMU*(PI/180);
      //Serial.println(angleFromIMUInRadians);
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
    //Serial.println(y);
    //Serial.println(newLeftEncoderValue - (newRightEncoderValue)*(-1));
    //non_inc_theta = atan((newLeftEncoderValue/100 -(newRightEncoderValue)*(-1)/100)/width);
    //input = non_inc_theta;
    //input = (newLeftEncoderValue - (newRightEncoderValue)*(-1));
    //Serial.println(time);
    //input = feedbackVariable;
    //input = originalTheta;
    setpoint = 0;
    input = ((angleFromIMUInRadians)*(-1)+1.27);
    //Serial.print("\t");
    //Serial.println(input);
    //Serial.print("\t");
    Serial.print(originalTheta);
    PID_L.Compute(); PID_R.Compute();
    //Serial.println(leftMotorSpeed);
    leftMotorSpeed += outputL;
    rightMotorSpeed -= outputR;
    //Serial.println(outputL);
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

/*void readOnce(){
   while (!Serial3.available()){
      Serial.println ("waiting");
      }
  if (Serial3.available()){
    if (initialCounter = 0){
      initialValue=Serial3.parseFloat();
      }
  }}*/
