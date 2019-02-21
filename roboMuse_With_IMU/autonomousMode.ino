/////////////////////////////////////////////////
///////////// Here we keep all the variables ////
double sides = 0;
int loops = 0;
int sideIteration = 0;
String keyboardInterrupt;

/////////////////////Keyboard Press Safety //////
void safeCheck() {
  while(Serial.available()) {
     keyboardInterrupt = Serial.readString();
    if (keyboardInterrupt == "a") {
      saberTooth.stop();
      leftMotorSpeed = 0;
      rightMotorSpeed = 0;
      Serial.println("I am here");
      interApt=0;
    }
    break;
  }
}

/////////////////////////////////////////////////
//////////// Here lies the main function ////////
void autonomousMode() {
  while (!Serial.available());
  if (Serial.available()) {
    Serial.println("Please Provide the Total Length Sides");
    sides = Serial.parseFloat();
  }
  while (!Serial.available());
  if (Serial.available()) {
    Serial.println("Please Provide the Total Amount of Loops");
    loops = Serial.parseInt();
  }
  ///////Loop for the Autonomous Navigation /////////
  for (int loopIteration = 0; loopIteration < loops; loopIteration++) {
    for (sideIteration = 0; sideIteration < 4; sideIteration++) {
      resetCoordinates();
      while (sqrt((x*x)+(y*y)) <= sides) {
        //Serial.println("Im here");
        a = 0;
        //safeCheck();
        motionType = "s";
        //Serial.println(newLeftEncoderValue - (newRightEncoderValue)*(-1));
        Serial.println(sqrt((x*x)+(y*y)));
        //Serial.println(y);
        leftMotorSpeed = 30;
        //rightMotorSpeed = 25.84;
        rightMotorSpeed = 30;
        odometryCalc();
        //input = feedbackVariable;
        input = originalTheta;
        //setpoint = orientation_vector_start[sideIteration];
        setpoint = 0;
        //Serial.println(originalThetaIterable);
        //Serial.println(setpoint);
        //setpoint = 0;
        input = originalThetaIterable;
        //Serial.println(originalTheta);
        PID_L.Compute(); PID_R.Compute();
        leftMotorSpeed += outputL;
        rightMotorSpeed -= outputR;
        saberTooth.motor(1, leftMotorSpeed);
        saberTooth.motor(2, rightMotorSpeed);
        delay(10);    
      }
      saberTooth.stop();
      delay(1000);
      Serial.println("I am out");
      
      //90 degree turn
//      while((thetaInRadians*(-1)+ prevTheta) <= orientation_vector[sideIteration]){
    
      while ((sum) <= orientation_vector[sideIteration]) {
//      while ((current_angle) <= orientation_vector[sideIteration){
        motionType = "turn";
        odometryCalc();
        calSpeed(orientation_angle[sideIteration] , 20 , (sum));
        //Serial.println("entered");
        saberTooth.motor(1, -speedTurn-5);
        saberTooth.motor(2, speedTurn+5);
        sum = float (originalTheta) * (-1);
        //Serial.println(sum);
        Serial.println(originalTheta);
      }
      prevTheta += sum;
      sum = 0;
      resetCoordinates();
      Serial.println("this is where i end my journey");
      saberTooth.stop();
      delay(1000);
    }
    prevTheta = 0;
   // sideIteration = 0;
    originalTheta = 0;
   // originalTheta = 0;
  }
  return;}
