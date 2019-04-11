#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_state_controller/joint_state_controller.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <ros/console.h>


//Hardware Interface for Robomuse
class MyRobot : public hardware_interface::RobotHW
{
public:

  MyRobot(){
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_left("chassis_to_left_wheel", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_left);

    hardware_interface::JointStateHandle state_handle_right("chassis_to_right_wheel", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_right);

    registerInterface(&jnt_state_interface);

    // connect and register the joint velocity interface
    hardware_interface::JointHandle vel_handle_left(jnt_state_interface.getHandle("chassis_to_left_wheel"), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_left);

    hardware_interface::JointHandle vel_handle_right(jnt_state_interface.getHandle("chassis_to_right_wheel"), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_right);

    registerInterface(&jnt_vel_interface);

  }

  void resetWheelEncoders(){
    if (leftCallbackFlag == 0 && rightCallbackFlag == 0){
      _wheel_angle[0] = 0;
      _wheel_angle[1] = 0;
    }
    leftCallbackFlag=0;
    rightCallbackFlag=0;
  }

//Write function for publishing values to the motor
  void write() {
    double diff_ang_speed_right = cmd[1];
    double diff_ang_speed_left = cmd[0];
    //resetWheelEncoders();
    //leftCallbackFlag=0;
    //rightCallbackFlag=0;
    std_msgs::Float32 left_wheel_vel_msg;
    std_msgs::Float32 right_wheel_vel_msg;
    left_wheel_vel_msg.data = diff_ang_speed_left;
    right_wheel_vel_msg.data = diff_ang_speed_right;
    left_wheel_vel_pub_.publish(left_wheel_vel_msg);
    right_wheel_vel_pub_.publish(right_wheel_vel_msg);



  }

//Read function to read encoder values and convert them in position and velocities.
   void read(const ros::Duration &period) {


     double ang_distance_left = (_wheel_angle[0]*0.0024697248*4.35);
     double ang_distance_right =(_wheel_angle[1]*0.0024697248*4.35);   //0001622*factor
       pos[0] += ang_distance_left;
       vel[0] += ang_distance_left / period.toSec();
       pos[1] += ang_distance_right;
       vel[1] += ang_distance_right / period.toSec();

   }

//ROS Time functions
   ros::Time get_time(){
     prev_update_time = curr_update_time;
     curr_update_time = ros::Time::now();
     return curr_update_time;
   }

   ros::Duration get_period(){
     return curr_update_time - prev_update_time;
   }

   /*pos[0]=0;
   pos[1]=0;
   vel[0]=0;
   vel[1]=0;
   eff[0]=0;
   eff[1]=0;
   */

private:
  ros::NodeHandle nh;

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;

  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
  double _wheel_angle[2];
  int leftCallbackFlag = 0;
  int rightCallbackFlag = 0;


  //double ang_distance_left_old=0;
  //double ang_distance_right_old=0;


  ros::Time curr_update_time, prev_update_time;


  void leftWheelAngleCallback(const std_msgs::Int32& msg) {
    _wheel_angle[0] = msg.data;
    leftCallbackFlag = 1;
  }

  void rightWheelAngleCallback(const std_msgs::Int32& msg) {
    _wheel_angle[1] = msg.data;
    rightCallbackFlag = 1;
  }

  ros::Publisher left_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("lmotor_cmd", 20);     /// changed thequeue size from 10 to 20
  ros::Publisher right_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("rmotor_cmd", 20);	/// changed the queue size from 10 to 20
  ros::Subscriber left_wheel_angle_sub_ = nh.subscribe("lwheel", 20, &MyRobot::leftWheelAngleCallback, this);		/// changed the queue size from  10 to 20
  ros::Subscriber right_wheel_angle_sub_ = nh.subscribe("rwheel", 20, &MyRobot::rightWheelAngleCallback, this);		/// chanegd the queue size from  10 to 20

};



int main(int argc, char **argv)
{
  //Setup the robomuse_hwinterface_node
  ros::init(argc, argv, "robomuse_hw");

  //Setup the control manager update loop
  MyRobot robot;
  controller_manager::ControllerManager cm(&robot);  //Controller manager would update the robot HW interface objects(read as well as write)
//Spinner thread, not Real Time   (for controller manager API, controller ROS API and other's)
  ros::AsyncSpinner spinner(10);
  spinner.start();


  //Control loop
  ros::Time prev_time=ros::Time::now();
  //ros::Rate rate(10.0); //Updation rate 10Hz
  ros::Rate rate(20.0); //Updation rate 20Hz		UPDATED IN  ARUDINO

  while(ros::ok())
  {
    const ros::Time   time = ros::Time::now();
    const ros::Duration period = time-prev_time;

//Control Thread, which is Real Time
    robot.read(period);
    cm.update(time, period);
    robot.write();
    robot.resetWheelEncoders();

    rate.sleep();

  }
  return 0;

}


//The Spinner thread and Control Thread talk to each other
