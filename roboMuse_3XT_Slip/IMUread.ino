////////////////////// IMU READING VARIABLES //////////////////////////////

float angleFromIMU, angleFromIMUYAW, angleFromIMUPITCH, angleFromIMUROLL, angleFromIMUYAWInRadians, angleFromIMUPITCHInRadians, angleFromIMUROLLInRadians;  
int readOnceVariable = 1;
float offsetAngleYAW, offsetAnglePITCH, offsetAngleROLL;

////////////////////// IMU READ CODE //////////////////////////////////////
void imuRead(){
  startIMUReading(0);
  if (Serial3.available()){
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
    readOnceVariable = 0;
    }
  }

//////////////////////////////////////////////////////////////////////////
///////////////// ANGLE INCREMENTS ///////////////////////////////////////
  
  
