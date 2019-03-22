int time=0;
float filteredTheta,deltaFilteredTheta;
float lastAngle, clockedAngularVelocity;
double lastX , lastY;

void debugMode(){
  while(time<4000 && interApt == 100){
    time = millis();
    safeCheck();        ///// press a to exit the loop
    motionType = "d";   ///// so that the loop does not break
    imuRead();          //// to start reading the IMU 
    angleProcessing();  //// to start transmitting the angles 
    leftMotorSpeed   = 40;  
    rightMotorSpeed  = 40;
    odometryCalc();
    velocityApproximation();   /// to find out the relationship between velocity and the battery level
        if (time >2000 && time < 3000){clockedAngularVelocity = velocityCentre;}
         //newKalmanFilter();
    slipDetection();
     Serial.println(filteredTheta);
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
    Serial.println(clockedAngularVelocity);
    /*readOnceVariable = 1;
    lastAngle = filteredTheta;
    lastX = x;
    lastY = y;
    Serial.print("FINAL ANGLE IS : ");
    Serial.println(lastAngle);*/
    
  }
