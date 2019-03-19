int time=0;
float filteredTheta,deltaFilteredTheta;
float lastAngle;
double lastX , lastY;

void debugMode(){
  while(time<3000 && interApt == 100){
    time = millis();
    safeCheck();        ///// press a to exit the loop
    motionType = "d";   ///// so that the loop does not break
    imuRead();          //// to start reading the IMU 
    angleProcessing();  //// to start transmitting the angles 
    leftMotorSpeed = 40;  
    rightMotorSpeed = 40;
    odometryCalc();
         //newKalmanFilter();
         Serial.println(filteredTheta);
    slipDetection();
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
    readOnceVariable = 1;
    lastAngle = filteredTheta;
    lastX = x;
    lastY = y;
   while (time < 10000){
    Serial.println("i am here...");
        
        imuRead();          //// to start reading the IMU 
        angleProcessing();
        odometryCalc();
        slipDetection();
         x =  x + centreIncremental * cos( filteredTheta + deltaFilteredTheta / 2)*(-1);   ///FILTERED XY
         y =  y + centreIncremental * sin( filteredTheta + deltaFilteredTheta / 2);
        calSpeed(-30, 30, filteredTheta);
        saberTooth.motor(1, -speedTurn-5);
        saberTooth.motor(2, speedTurn+5);
        //Serial.println(filteredTheta);
    }
    /*Serial.print(",");
    Serial.println(lastX + x);
    Serial.print(",");
    Serial.print(lastY +y);
    Serial.print(",");
    Serial.print(lastAngle+filteredTheta);*/
   
  }
