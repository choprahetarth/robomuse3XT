#include <ros.h>
#include <Kangaroo.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

// Independent mode channels on Kangaroo are, by default, '1' and '2'.
KangarooSerial  K(Serial2);
KangarooChannel K_left(K, '2');
KangarooChannel K_right(K, '1');

//Factor to get the correct angular velocity
float angular_correct=1.06409;
//Multipying factor for correcting the robot's trajectory(Difference in robot wheels)   
float factor = 1.01638;
//Multiplying factor to provide the right commands to Kangaroo(Angular vel(rad/s) changed to the corrected motor commands (1044.33*radius)) 
float vel_to_cmd=(64.77456*angular_correct); 

long millis1=0;
long oldPositionl=0;
long newPositionl;
long oldPositionr=0;
long newPositionr;

float left_old=0;
float right_old=0;
float left_new=0;
float right_new=0;



//*************************ROS SETUP*************************//
  ros::NodeHandle  nh;
  
  //Callback function for left motor
  void left( const std_msgs::Float32& roboclaw_l){                   //Publishes command to the motor only if the value recieved is different from the previous one      
    left_new=float(roboclaw_l.data);   
      if(left_new!=left_old){
      left_old=left_new*vel_to_cmd*factor;                           //Left motor provided with extra velocity(smaller diameter)
      K_left.s(left_old);           
      }
      if(left_new==0){
        K_left.s(0);
      }
  }
     
  //Callback function for right motor 
  void right( const std_msgs::Float32& roboclaw_r){                  //Publishes command to the motor only if the value recieved is different from the previous one 
    right_new=roboclaw_r.data;
    if(right_new!=right_old){
    right_old=right_new*vel_to_cmd;
    K_right.s(right_old);
    }
    if(right_new==0){
    K_right.s(0);
    }         
  }

  //Motor Command Subscribers
  ros::Subscriber<std_msgs::Float32> l_motor("lmotor_cmd", &left );
  ros::Subscriber<std_msgs::Float32> r_motor("rmotor_cmd", &right );

//**************************END**************************//

  
  //Encoder Data Variables
  std_msgs::Int32 l_msg;
  std_msgs::Int32 r_msg;
  
  //Encoder Data Publishers
  ros::Publisher left_encoder("lwheel", &l_msg);
  ros::Publisher right_encoder("rwheel", &r_msg);


void setup()
{
  //Serial.begin(9600); 
  
//**********Kangaroo Setup**********//  
  Serial2.begin(38400);
  K_left.start();
  K_left.home().wait();  
  K_right.start();
  K_right.home().wait();

//***********ROS INITIALISATION**************//  
  nh.getHardware()->setBaud(38400);
  
  //Publishers & Subscribers Initialise  
  nh.initNode();
  nh.advertise(left_encoder);
  nh.advertise(right_encoder);
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
    newPositionl=(long(K_left.getP().value())/factor);
    newPositionr=long(K_right.getP().value());
    r_msg.data=(newPositionr-oldPositionr);
    l_msg.data=(newPositionl-oldPositionl);
    oldPositionl=newPositionl;
    oldPositionr=newPositionr;
    left_encoder.publish( &l_msg );
    right_encoder.publish( &r_msg );
  }
  nh.spinOnce();  
}
