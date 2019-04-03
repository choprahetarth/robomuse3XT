int time=0;
float filteredTheta,deltaFilteredTheta;
float lastAngle, clockedVelocity ;
double lastX , lastY;
double position1, position2, approximatedPosition;
void debugMode(){
  while(time<4000  && interApt == 100){
//while(interApt == 100){
    time = millis();
    safeCheck();        ///// press a to exit the loop
    motionType = "d";   ///// so that the loop does not break
    imuRead();          //// to start reading the IMU 
    angleProcessing();  //// to start transmitting the angles 
    leftMotorSpeed   = 50;  
    rightMotorSpeed  = 50;
    odometryCalc();
    velocityApproximation();   /// to find out the relationship between velocity and the battery level
        if (time >3000 && time < 3999){ clockedVelocity  =  centreWheelVelocity;} ///check out the final velocity of the vehicle 
         //newKalmanFilter();
    slipDetection();  
    //yPoseEstimate();
    input = filteredTheta;
    PID_L.Compute(); PID_R.Compute();
    leftMotorSpeed += outputL;
    rightMotorSpeed -= outputR;
    x = x + centreIncremental * cos(filteredTheta + deltaFilteredTheta / 2)*(-1);   ///FILTERED XY///
    y = y + centreIncremental * sin(filteredTheta + deltaFilteredTheta / 2);
    saberTooth.motor(1, leftMotorSpeed);
    saberTooth.motor(2, rightMotorSpeed);
    }
    saberTooth.stop();
    delay(1000);
    //Serial.println("I am out");
    //Serial.println(clockedVelocity);
    readOnceVariable = 1;
    lastAngle = filteredTheta; 
    position1 = poseCalculate(1,4, clockedVelocity); /// only find the final position instead of finding the positions every second
    position2 = poseCalculate(2,4, clockedVelocity);
    approximatedPosition = ((1-0.7932797872)*position1)+((0.7932797872)*position2); /// filter out the final position
    Serial.print(" FINAL POSITION :");
    Serial.print(approximatedPosition,4);
    Serial.print(" POSITION1  ");
    Serial.print(position1,4);
    Serial.print(" POSITION2");
    Serial.print(position2,4);
  }
