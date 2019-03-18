int time;
float filteredTheta;
void debugMode(){
  while(time<3000 && interApt == 100){
    time = millis();
    safeCheck();        ///// press a to exit the loop
    motionType = "d";   ///// so that the loop does not break
    imuRead();          //// to start reading the IMU 
    angleProcessing();  //// to start transmitting the angles 
    leftMotorSpeed = 50;  
    rightMotorSpeed = 50;
    odometryCalc();
         //newKalmanFilter();
    slipDetection();
    input = filteredTheta;
    PID_L.Compute(); PID_R.Compute();
    leftMotorSpeed += outputL;
    rightMotorSpeed -= outputR;
    x = x + centreIncremental * cos(filteredTheta + theta / 2)*(-1);   ///FILTERED XY
    y = y + centreIncremental * sin(filteredTheta + theta / 2);
    saberTooth.motor(1, leftMotorSpeed);
    saberTooth.motor(2, rightMotorSpeed);
    Serial.print(",");
    Serial.println(x);
    Serial.print(y);
    }
    saberTooth.stop();
    delay(1000);
    Serial.println("I am out");
   while (abs(filteredTheta) <30){
        odometryCalc;
        calSpeed(30, 30, filteredTheta);
        saberTooth.motor(1, -speedTurn-5);
        saberTooth.motor(2, speedTurn+5);

    }
   
  }
