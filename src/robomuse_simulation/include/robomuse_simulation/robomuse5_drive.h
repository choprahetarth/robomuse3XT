
#ifndef ROBOMUSE5_DRIVE_H_
#define ROBOMUSE5_DRIVE_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define LINEAR_VELOCITY  0.7
#define ANGULAR_VELOCITY 1.5

#define GET_RM5_DIRECTION 0
#define RM5_DRIVE_FORWARD 1
#define RM5_RIGHT_TURN    2
#define RM5_LEFT_TURN     3

class Robomuse5Drive
{
 public:
  Robomuse5Drive();
  ~Robomuse5Drive();
  bool init();
  bool controlLoop();

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters

  // ROS Time

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;

  // Variables
  double escape_range_;
  double check_forward_dist_;
  double check_side_dist_;

  double scan_data_[3] = {0.0, 0.0, 0.0};

  double rm5_pose_;
  double prev_rm5_pose_;

  // Function prototypes
  void updatecommandVelocity(double linear, double angular);
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
};
#endif // ROBOMUSE5_DRIVE_H_
