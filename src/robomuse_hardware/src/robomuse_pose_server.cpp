#include "ros/ros.h"
#include "robomuse_hardware/time_determine.h"

int time_initial = 0;
float linear_vel = 0.25;                                               //m/s
float linear_to_ticks = 6161.47;                                       //conversion factor
float ticks_acc = 1000;
float ticks_deacc = 2000;


bool position(robomuse_hardware::time_determine::Request  &req,
         robomuse_hardware::time_determine::Response &res)
{
  if(req.position<0.5){
    	linear_vel = 0.1;
  }
  //ROS_INFO("request: x=%ld", (long int)req.position);
  float total_ticks = req.position*linear_to_ticks;
  float ticks_vel = linear_vel*linear_to_ticks;                            //ticks/s
  float time_acc = ticks_vel/ticks_acc;
  float distance_acc = (ticks_acc*time_acc*time_acc)/2;                    //Ticks covered to reach the velocity
  float time_deacc = ticks_acc/ticks_deacc;
  float distance_deacc = (ticks_deacc*time_deacc*time_deacc)/2;            //Ticks covered to reach zero velocity
  float ticks_required = total_ticks - distance_acc - distance_deacc;
  float time_ticks_required = ticks_required/ticks_vel;
  float time_total = time_ticks_required + time_acc + time_deacc;
  res.total_time = time_total;
  ROS_INFO("request: pos x=[%f]", req.position);
  ROS_INFO("sending back response time:[%f]", res.total_time);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robomuse_pose_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("pose_x_time", position);
  ROS_INFO("Ready to find the time required for the pos.");
  ros::spin();

  return 0;
}
