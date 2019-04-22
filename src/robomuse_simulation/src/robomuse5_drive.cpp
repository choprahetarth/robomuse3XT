
#include "robomuse_simulation/robomuse5_drive.h"

Robomuse5Drive::Robomuse5Drive()
  : nh_priv_("~")
{
  //Init gazebo ros robomuse5 node
  ROS_INFO("RoboMuse5 Simulation Node Init");
  ROS_ASSERT(init());
}

Robomuse5Drive::~Robomuse5Drive()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Robomuse5Drive::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escape_range_       = 30.0 * DEG2RAD;
  check_forward_dist_ = 0.7;
  check_side_dist_    = 0.6;

  rm5_pose_ = 0.0;
  prev_rm5_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Robomuse5Drive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Robomuse5Drive::odomMsgCallBack, this);

  return true;
}

void Robomuse5Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);

	rm5_pose_ = atan2(siny, cosy);
}

void Robomuse5Drive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[3] = {0, 30, 330};

  for (int num = 0; num < 3; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void Robomuse5Drive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Robomuse5Drive::controlLoop()
{
  static uint8_t robomuse5_state_num = 0;

  switch(robomuse5_state_num)
  {
    case GET_RM5_DIRECTION:
      if (scan_data_[CENTER] > check_forward_dist_)
      {
        if (scan_data_[LEFT] < check_side_dist_)
        {
          prev_rm5_pose_ = rm5_pose_;
          robomuse5_state_num = RM5_RIGHT_TURN;
        }
        else if (scan_data_[RIGHT] < check_side_dist_)
        {
          prev_rm5_pose_ = rm5_pose_;
          robomuse5_state_num = RM5_LEFT_TURN;
        }
        else
        {
          robomuse5_state_num = RM5_DRIVE_FORWARD;
        }
      }

      if (scan_data_[CENTER] < check_forward_dist_)
      {
        prev_rm5_pose_ = rm5_pose_;
        robomuse5_state_num = RM5_RIGHT_TURN;
      }
      break;

    case RM5_DRIVE_FORWARD:
      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
      robomuse5_state_num = GET_RM5_DIRECTION;
      break;

    case RM5_RIGHT_TURN:
      if (fabs(prev_rm5_pose_ - rm5_pose_) >= escape_range_)
        robomuse5_state_num = GET_RM5_DIRECTION;
      else
        updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
      break;

    case RM5_LEFT_TURN:
      if (fabs(prev_rm5_pose_ - rm5_pose_) >= escape_range_)
        robomuse5_state_num = GET_RM5_DIRECTION;
      else
        updatecommandVelocity(0.0, ANGULAR_VELOCITY);
      break;

    default:
      robomuse5_state_num = GET_RM5_DIRECTION;
      break;
  }

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "robomuse5_drive");
  Robomuse5Drive robomuse5_drive;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    robomuse5_drive.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
