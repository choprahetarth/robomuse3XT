////////////////////// IMU READING VARIABLES //////////////////////////////

float angleFromIMU, angleFromIMUYAW, angleFromIMUPITCH, angleFromIMUROLL, angleFromIMUYAWInRadians, angleFromIMUPITCHInRadians, angleFromIMUROLLInRadians;  
int readOnceVariable = 1;
float offsetAngleYAW, offsetAnglePITCH, offsetAngleROLL;
float feedbackFromIMU,newFeedbackFromIMU,oldFeedbackFromIMU, deltaAngle, totalAngle;
float YAW, PITCH, ROLL, incYAW, incPITCH, incROLL;
byte initializer1,initializer2,initializer3;
int count;

////////////////////// IMU READ CODE //////////////////////////////////////

void imuRead(){
  startIMUReading(0);
  if (Serial3.available()){
    /*if(Serial3.peek()=='y'){ Serial3.read();angleFromIMUYAW=Serial3.read();}
    else if(Serial3.peek()=='p'){ Serial3.read();angleFromIMUPITCH=Serial3.read();}
    else if(Serial3.peek()=='r'){ Serial3.read();angleFromIMUROLL=Serial3.read();}
    else{Serial3.read();}
    Serial.println(angleFromIMUYAW);*/
    //Serial.println(Serial3.peek());
    //Serial.println(Serial3.peek());
    //Serial.println(Serial3.peek());
    angleFromIMUYAW=Serial3.parseFloat();
    angleFromIMUPITCH=Serial3.parseFloat();
    angleFromIMUROLL=Serial3.parseFloat();
    angleFromIMUYAWInRadians = angleFromIMUYAW * (M_PI/180); 
    angleFromIMUPITCHInRadians = angleFromIMUPITCH * (M_PI/180);
    angleFromIMUROLLInRadians = angleFromIMUROLL * (M_PI/180);
    readOnce();
    }
  }
  
///////////////////////////////////////////////////////////////////////////
///////////// IMU STARTS TRANSMISSION ONLY WHEN THIS PIN IS HIGH //////////

void startIMUReading(int detectPin){
  if (detectPin == 1){
    digitalWrite (34,HIGH);
  }
  else if(detectPin == 0){
    digitalWrite (34,LOW);
    }
  }

///////////////////////////////////////////////////////////////////////////  
//////////// GET THE INITAL OFFSET VALUES /////////////////////////////////

void readOnce(){
  if (readOnceVariable == 1 ){
    offsetAngleYAW = angleFromIMUYAW;
    offsetAnglePITCH = angleFromIMUPITCH;
    offsetAngleROLL =angleFromIMUROLL;
    //if (count > 100){
      readOnceVariable = 0;
    //  }
    }
  }

//////////////////////////////////////////////////////////////////////////
///////////////// IMU ANGLE PROCESSING ///////////////////////////////////

float imuAngleIncrement(float absoluteAngleValue, float offsetAngle){
    newFeedbackFromIMU = ((absoluteAngleValue)*(-1))+offsetAngle;
    deltaAngle = newFeedbackFromIMU - oldFeedbackFromIMU;
    oldFeedbackFromIMU = newFeedbackFromIMU;
    return deltaAngle;
  }

float imuAngleAbsolute(float angleValue, float offsetAngle){
    totalAngle = angleValue*(-1)+offsetAngle;
    return totalAngle;
  }

void angleProcessing(){
    incYAW = imuAngleIncrement(angleFromIMUYAW, offsetAngleYAW);
    YAW = imuAngleAbsolute(angleFromIMUYAW, offsetAngleYAW);
    incPITCH = imuAngleIncrement(angleFromIMUPITCH, offsetAnglePITCH);
    PITCH = imuAngleAbsolute(incPITCH, offsetAnglePITCH);
    incYAW = imuAngleIncrement(angleFromIMUROLL, offsetAngleROLL);
    ROLL = imuAngleAbsolute(incROLL,offsetAngleROLL);
  }

///////////// SIGNAL RECEPTION FUNCTION ////////////////////////////////

/*const byte numChars = 32;
char receivedChars[numChars];

boolean newData = false;

void recvWithStartEndMarkers(){
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = "p";
  char endMarker = ">";
  char rc;
  char abc;
  while (Serial3.available()>0 && newData == false){
    rc = Serial3.read();
    if (recvInProgress == true){
      if (rc  != endMarker){
          receivedChars[ndx] = rc;
          ndx++;
          if (ndx >= numChars){ndx= numChars -1;}
        }
       else{
          receivedChars[ndx] = '\0';
          recvInProgress =false;
          ndx = 0;
          newData = true;
        }
      }
      else if (rc == startMarker) {
            recvInProgress = true;
            Serial.println("DETECTED");
       }
    }
  }

void showNewData(){
  if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        newData = false;
    }
  }*/
