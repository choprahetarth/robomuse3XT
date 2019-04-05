#include <ros.h>
#include <Encoder.h>
#include <Sabertooth.h>
#include <PID_v1.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

double input,outputL, outputR, setpoint, KpL, KiL, KdL, KpR, KiR, KdR;

////// single time setup ///////

Sabertooth saberTooth(128, Serial2);  // Packetized serial mode, Non-LI, 128 bit Addr. (0,0,1,1,1,1)
Encoder enCoder_1(20, 21); // Left hand side enc., +ve value means forward
Encoder enCoder_2(2, 3);  // Right hand side enc., -ve value means forward
PID PID_L(&input, &outputL, &setpoint, KpL, KiL, KdL, P_ON_M, DIRECT);   //// pid variables 
PID PID_R(&input, &outputR, &setpoint, KpR, KiR, KdR, P_ON_M, DIRECT);   //// pid variables

////////////////////////////////

//Factor to get the correct angular velocity
float angular_correct=1.06409;
//Multipying factor for correcting the robot's trajectory(Difference in robot wheels)   
float factor = 1.01638;
//Multiplying factor to provide the right commands to Kangaroo(Angular vel(rad/s) changed to the corrected motor commands (1044.33*radius)) 
float vel_to_cmd=(64.77456*angular_correct); 
int startTime;
long millis1=0;
long oldPositionl=0;
long newPositionl;
long oldPositionr=0;
long newPositionr;

float left_old=0;
float right_old=0;
float left_new=0;
float right_new=0;

float minVal= -15;
float maxVal = 15;





///////////ALL CALCULATIONS //////////////
//////odometry variables ///////////////////////////////////////////////////

double newLeftEncoderValue, newRightEncoderValue, leftEncoderIncrement, rightEncoderIncrement, leftWheelIncrement, rightWheelIncrement, theta, feedbackVariable, centreIncremental;
double x, y, originalTheta, originalThetaIterable, oldLeftEncoderValue, oldRightEncoderValue ;
double width = 518; //in millimeters
float conversionFactorLeft = 0.01, conversionFactorRight= 0.01;

/////////////////////////////////////////////////////////////////////////////
////// filter variables /////////////////////////////////////////////////////

float alpha4, alphaQ =0;
float complimentaryMeasurement1 , complimentaryMeasurement2 =0;
float filterValue =0, gainValue = 0, deltaFilteredTheta;
float newFilteredTheta, oldFilteredTheta, filteredThetaInRadians, filteredTheta;

//////// velocity variables ////////////////////////////////////////////////

int  currentTime, dt;
double velocityLeft, velocityRight, velocityCentre;
double velocityLeftWheel, velocityRightWheel, centreWheelVelocity, centreWheelVelocityAngular;

//////////////////////////////////////////////////////////////////////////////
//////// code to calculate odometry //////////////////////////////////////////

void odometryCalc() {
  newLeftEncoderValue = double(enCoder_1.read());
  newRightEncoderValue = double(enCoder_2.read());
  leftEncoderIncrement = newLeftEncoderValue - oldLeftEncoderValue;
  rightEncoderIncrement = newRightEncoderValue - oldRightEncoderValue;
  leftWheelIncrement = leftEncoderIncrement * conversionFactorLeft; // left side advanced-by-distance
  rightWheelIncrement = rightEncoderIncrement * conversionFactorRight * (-1); // right side advanced-by-distance
  theta = atan((rightWheelIncrement - leftWheelIncrement) / width);
  theta = theta * (180/M_PI);
  feedbackVariable = theta;
  oldLeftEncoderValue = newLeftEncoderValue;
  oldRightEncoderValue = newRightEncoderValue;
  centreIncremental = ((leftWheelIncrement + rightWheelIncrement) / 2);
  x = x + centreIncremental * cos(originalTheta + theta / 2)*(-1);   ///ORIGINAL XY
  y = y + centreIncremental * sin(originalTheta + theta / 2);                
  originalTheta = originalTheta + theta;
  originalThetaIterable = originalThetaIterable + theta;
}

///// code to filter the heading angles /////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////


/*void weightedFilter(){
    //setpoint//
    //filterSetPoint = 0;
    //sensor priority//
    alpha4 = 1;// complete priority
    alphaQ = 1*((abs(originalTheta-YAW)/(abs(originalTheta-YAW)+gainValue)));    // lesser the value of the gain Value, more priority to Encoders
    if(centreWheelVelocity < 0.20){
          gainValue = 5;
      }
    else {
          gainValue = 0.1/centreWheelVelocity;
      }

    //filter//
    complimentaryMeasurement1 = originalTheta; 
    complimentaryMeasurement2 = YAW;
    if ( abs(originalTheta - YAW) >0.1 && abs(originalTheta - YAW) <10)      {filterValue = alphaQ;}
    else if ( abs(originalTheta - YAW) >10 )                                 {filterValue = alpha4;}
    filteredTheta = (1-filterValue)*originalTheta + filterValue*(YAW);
    newFilteredTheta = filteredTheta;
    filteredThetaInRadians = filteredTheta * (M_PI/180);
    deltaFilteredTheta = newFilteredTheta - oldFilteredTheta;
    oldFilteredTheta = newFilteredTheta;
  }*/

  
////// code in case the imu is thrown away ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


/*void plugAndPlay(){
    if (!Serial3.available()){
      filterValue = 0; /// full priority to the encoders
      Serial.println("Switched to Encoder Navigation");
      }
   }*/

//////////////////////////////////////////////////////////////////////////////
////// slip detection combined code //////////////////////////////////////////

/*void slipDetection(){
    /// condition 1 : when there is a change in pitch  
        //singleDifferentiation();x
    /// condition 2 : when there is a difference between the wheels
        weightedFilter();
    /// condition 3 : when the actual odometries dont match 
        // insert the INS Function once it is completed
    /// condition 4 : in case the imu sensor is plugged off and thrown away 
        plugAndPlay();
  }*/
  
//////////////////////////////////////////////////////////////////////////////
//////// code to approximate the velocity ///////////////////////////////////

void velocityApproximation(){
  currentTime = millis();
  //dt = currentTime-startTime;
  dt=30;  //////DETERMINE THIS EXPERIMENTALLY BEFORE TRYING IT OUT YOURSELVES
  velocityLeftWheel = leftWheelIncrement / dt;
  velocityRightWheel = rightWheelIncrement / dt;
  centreWheelVelocity = (velocityLeftWheel + velocityRightWheel)/2;
  centreWheelVelocityAngular = centreWheelVelocity/0.518; //// 0.518 will be the 
  startTime = currentTime;
  }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////











//*************************ROS SETUP*************************//
  ros::NodeHandle  nh;
  
  void left(const std_msgs::Float32& saber_l){
    left_new=float(saber_l.data);
    if(left_new!=left_old){
      left_old=left_new*vel_to_cmd*factor;                           //Left motor provided with extra velocity(smaller diameter)
      saberTooth.motor(1,left_old);          
      }
      if(left_new==0){
        saberTooth.motor(1,0);
      }
    
    }

    void right(const std_msgs::Float32& saber_r){
    right_new=float(saber_r.data);
    if(right_new!=right_old){
      right_old=right_new*vel_to_cmd;                           //Left motor provided with extra velocity(smaller diameter)
      saberTooth.motor(2,right_old);          
      }
      if(right_new==0){
        saberTooth.motor(2,0);
      }
    
    }

  //Motor Command Subscribers
  ros::Subscriber<std_msgs::Float32> l_motor("lmotor_cmd", &left );   //// left wheel command vel subscriber
  ros::Subscriber<std_msgs::Float32> r_motor("rmotor_cmd", &right );  //// right wheel command vel subscriber

//**************************END**************************//

 //Encoder Data Variables
  std_msgs::Int32 l_msg;
  std_msgs::Int32 r_msg;
  std_msgs::Int32 n_theta;
  
  //Encoder Data Publishers
  ros::Publisher left_encoder("lwheel", &l_msg);
  ros::Publisher right_encoder("rwheel", &r_msg);       /// we need the publishers
  ros::Publisher normal_theta("normalTheta", &n_theta); 
  
void setup()
{ 
  startTime = millis();
//**********Sabertooth Setup**********//  
  Serial.begin(115200);   // Arduino to PC Communcation        
  Serial2.begin(9600);    // Baud rate for communication with sabertooth
  //Serial3.begin(115200);  // Serial Communication with the Arduino Uno for I2C with IMU 
  pinMode(34,OUTPUT);     // MEGA-UNO Link
  PID_L.SetOutputLimits(minVal, maxVal);  // [Min,Max] values of output
  PID_L.SetMode(AUTOMATIC);  // Automatic = ON, Manual = OFF
  PID_R.SetOutputLimits(minVal, maxVal);
  PID_R.SetMode(AUTOMATIC);
 

//***********ROS INITIALISATION**************//  
  nh.getHardware()->setBaud(38400);
  
  //Publishers & Subscribers Initialise  
  nh.initNode();
  nh.advertise(left_encoder);
  nh.advertise(right_encoder);
  nh.advertise(normal_theta);
  nh.subscribe(l_motor);
  nh.subscribe(r_motor);
  while (!nh.connected())
  {
    nh.spinOnce();
  }
//*******************************************//
  millis1=millis();
  
}

void loop()
{
  //Condition applied to Publish Data at 10Hz
  if(millis()-millis1>100){
    millis1+=100;    
   
   //////// PUBLISHERS /////////////

    r_msg.data=rightWheelIncrement;     /// right wheel message
    l_msg.data=leftWheelIncrement;      /// left wheel message
    n_theta.data = originalTheta;       /// theta message 
    
    left_encoder.publish( &l_msg );
    right_encoder.publish( &r_msg );
    normal_theta.publish( &n_theta );
  }
  nh.spinOnce();  
}
