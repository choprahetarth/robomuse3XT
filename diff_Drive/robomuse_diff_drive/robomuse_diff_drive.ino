#include <ros.h>
#include <Encoder.h>
#include <Sabertooth.h>
#include <PID_v1.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

int timeStamp;
double input, setpoint;
double KpL = 1.5, KiL = 0.1, KdL = 0, outputL = 0; // Proportional, Integral & Derivative coefficients
double KpR = 1.5, KiR = 0.1, KdR = 0, outputR = 0;  // of respective motors for PID control

////// single time setup ///////

Sabertooth saberTooth(128, Serial2);  // Packetized serial mode, Non-LI, 128 bit Addr. (0,0,1,1,1,1)
Encoder enCoder_1(20, 21); // Left hand side enc., +ve value means forward
Encoder enCoder_2(2, 3);  // Right hand side enc., -ve value means forward
PID PID_L(&input, &outputL, &setpoint, KpL, KiL, KdL, P_ON_M, DIRECT);   //// pid variables 
PID PID_R(&input, &outputR, &setpoint, KpR, KiR, KdR, P_ON_M, DIRECT);   //// pid variables

////////////////////////////////

//Factor to get the correct angular velocity
//float angular_correct=1.06409;  /// factor in order to compensate for the difference in wheel radius
//Multipying factor for correcting the robot's trajectory(Difference in robot wheels)   
float factor = 1.72;   //// 
//Multiplying factor to provide the right commands to Kangaroo(Angular vel(rad/s) changed to the corrected motor commands (1044.33*radius)) 
//float vel_to_cmd=(64.77456*angular_correct)/10; 
float vel_to_cmd = (4.87);
//float vel_to_cmd = (angular_correct);
int startTime;
long millis1=0;
long oldPositionl=0;
long newPositionl;
long oldPositionr=0;
long newPositionr;
int pinDetected;
float left_old=0;
float right_old=0;
float left_new=0;
float right_new=0;
int flag1 =0, flag2 =0;
float minVal= -15;
float maxVal = 15;


///////////ALL CALCULATIONS //////////////
//////odometry variables ///////////////////////////////////////////////////

double newLeftEncoderValue, newRightEncoderValue, leftEncoderIncrement, rightEncoderIncrement, leftWheelIncrement, rightWheelIncrement, theta, feedbackVariable, centreIncremental;
double x, y, originalTheta, originalThetaIterable, oldLeftEncoderValue, oldRightEncoderValue ;
double width = 518; //in millimeters
float conversionFactorLeft = 0.1, conversionFactorRight= 0.1;

/////////////////////////////////////////////////////////////////////////////
////// filter variables /////////////////////////////////////////////////////

float filteredTheta;

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
  originalTheta = (originalTheta + theta);
  
}

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

 void obstacleDetection(){
  pinDetected = digitalRead(32);
  }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





//*************************ROS SETUP*************************//
  ros::NodeHandle  nh;
/////////////LEFT RIGHT MOTORS ARE REVERESED HERE AS COMPARED TO THE ORIGINAL CODE////////////////

  void left(const std_msgs::Float32& saber_l){
    left_new=float(saber_l.data);
    left_new = left_new *vel_to_cmd;
      if (left_new !=0.0){
      PID_R.Compute();PID_L.Compute();
      left_new = left_new - outputL;}
      else {
      left_new = left_new;  
        }
      saberTooth.motor(2,left_new);
      }

    void right(const std_msgs::Float32& saber_r){
    right_new=float(saber_r.data);
    right_new = right_new * vel_to_cmd;
      if (right_new !=0.0){
      PID_R.Compute();PID_L.Compute();
      right_new = right_new + outputR;}
      else {
      right_new = right_new;
       } 
      saberTooth.motor(1,right_new); 
    }
      //if(right_new==0.0){
        //saberTooth.motor(1,0);
      //}
    ///}

    void fil_theta(const std_msgs::Float32& ftheta){
     filteredTheta = float(ftheta.data); 
      }

  //Motor Command Subscribers
  ros::Subscriber<std_msgs::Float32> l_motor("lmotor_cmd", &left );   //// left wheel command vel subscriber
  ros::Subscriber<std_msgs::Float32> r_motor("rmotor_cmd", &right );  //// right wheel command vel subscriber
  ros::Subscriber<std_msgs::Float32> f_theta("filteredTheta", &fil_theta);

//**************************END**************************//

 //Data Variables
  std_msgs::Int32 l_msg;    ////no need to change to float
  std_msgs::Int32 r_msg;    ////no need to change to float
  std_msgs::Float32 n_theta; ///send with float 
  std_msgs::Float32 c_vel;   ///send with float 
  std_msgs::Int16 obs_det;   ///variable for the obstacle flag
  
  //Data Publishers  
  ros::Publisher left_encoder("lwheel", &l_msg);
  ros::Publisher right_encoder("rwheel", &r_msg);       /// we need the publishers
  ros::Publisher normal_theta("normalTheta", &n_theta); 
  ros::Publisher velo_centre("velo_centre", &c_vel);
  ros::Publisher obstacle("obstacle_pin", &obs_det );
  
  
void setup()
{ 
  startTime = millis();
//**********Sabertooth Setup**********//  
  Serial.begin(9600);   // Arduino to PC Communcation        
  Serial2.begin(9600);    // Baud rate for communication with sabertooth
  Serial3.begin(115200);
  pinMode(32,INPUT);     // MEGA-UNO Link for Ultrasonic Sensors
  PID_L.SetOutputLimits(minVal, maxVal);  // [Min,Max] values of output
  PID_L.SetMode(AUTOMATIC);  // Automatic = ON, Manual = OFF
  PID_R.SetOutputLimits(minVal, maxVal);
  PID_R.SetMode(AUTOMATIC);
 

//***********ROS INITIALISATION**************//  
  nh.getHardware()->setBaud(57600);
  
  //Publishers & Subscribers Initialise  
  nh.initNode();
  nh.advertise(left_encoder);
  nh.advertise(right_encoder);
  nh.advertise(normal_theta);
  nh.advertise(velo_centre);
  nh.advertise(obstacle);
  nh.subscribe(l_motor);
  nh.subscribe(r_motor);
  nh.subscribe(f_theta);
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
  if(millis()-millis1>50){
    millis1+=50;    
    timeStamp = millis();
    odometryCalc();
    velocityApproximation();
    obstacleDetection();
    input = filteredTheta;             /// PID input should be of the filteredTheta
    //////// PUBLISHERS /////////////
    r_msg.data=rightWheelIncrement;     /// right wheel message
    l_msg.data=leftWheelIncrement;      /// left wheel message
    n_theta.data = originalTheta;       /// theta message  
    c_vel.data = centreWheelVelocity;   /// centre wheel velocity 
    obs_det.data = pinDetected;
    left_encoder.publish( &l_msg );     /// publishing actions
    right_encoder.publish( &r_msg );
    normal_theta.publish( &n_theta );
    velo_centre.publish( &c_vel );
    obstacle.publish( &obs_det );
  }
  nh.spinOnce();  
}
