////////////////////// IMU READING VARIABLES //////////////////////////////

float angleFromIMU, angleFromIMUYAW, angleFromIMUPITCH, angleFromIMUROLL, angleFromIMUYAWInRadians, angleFromIMUPITCHInRadians, angleFromIMUROLLInRadians;  
int readOnceVariable = 1;
float offsetAngleYAW, offsetAnglePITCH, offsetAngleROLL;
float feedbackFromIMU,newFeedbackFromIMU,oldFeedbackFromIMU, deltaAngle, totalAngle;
float YAW, PITCH, ROLL, incYAW, incPITCH, incROLL;
byte initializer1,initializer2,initializer3;
int count;

////////////////////// IMU READ CODE //////////////////////////////////////

/*void imuRead(){
  startIMUReading(0);
  if (Serial3.available()){
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
    //incPITCH = imuAngleIncrement(angleFromIMUPITCH, offsetAnglePITCH);
    //PITCH = imuAngleAbsolute(incPITCH, offsetAnglePITCH);
    //incYAW = imuAngleIncrement(angleFromIMUROLL, offsetAngleROLL);
    //ROLL = imuAngleAbsolute(incROLL,offsetAngleROLL);
  }*/
