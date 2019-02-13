void navigationMode() {
  while (abs(x) <= 5000 && interApt == 100) {
    speedRamp(1,40);
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
    //Serial.println(y);
    //Serial.println(newLeftEncoderValue - (newRightEncoderValue)*(-1));
    //non_inc_theta = atan((newLeftEncoderValue/100 -(newRightEncoderValue)*(-1)/100)/width);
    //input = non_inc_theta;
    //input = (newLeftEncoderValue - (newRightEncoderValue)*(-1));
    //Serial.println(time);
    input = feedbackVariable;
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
    delay(200);
    //Serial.println(originalTheta);
  }
  saberTooth.stop();
  delay(500);
  resetCoordinates();
}

void speedRamp(float powerOfFunction, float speedValue){
    while (rampOutput < speedValue){
      ramp = ramp + (pow(dRamp,powerOfFunction));
      rampOutput = ramp*speedValue;
      //Serial.println(rampOutput);
      return rampOutput;
    }
}
