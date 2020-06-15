/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include "piyabot_gazebo/piyabot_drive.h"

//using namespace std;

PiyabotDrive::PiyabotDrive()
  : nh_priv_("~")
{
  //Init gazebo ros piyabot node
  ROS_INFO("Piyabot Simulation Node Init");
  ROS_ASSERT(init());
}

PiyabotDrive::~PiyabotDrive()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool PiyabotDrive::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");
  std::string scan_topic_name = nh_.param<std::string>("scan_topic_name", "");
  std::string odom_topic_name = nh_.param<std::string>("odom_topic_name", "");
  std::string bumper_topic_name = nh_.param<std::string>("bumper_topic_name", "");

  std::string sonar_front_topic_name = nh_.param<std::string>("sonar_front_topic_name", "");
  std::string sonar_left_topic_name = nh_.param<std::string>("sonar_left_topic_name", "");
  std::string sonar_right_topic_name = nh_.param<std::string>("sonar_right_topic_name", "");
  std::string sonar_back_topic_name = nh_.param<std::string>("sonar_back_topic_name", "");

  double linear_vel = nh_.param<double>("linear_vel", 0);
  double angular_vel = nh_.param<double>("angular_vel", 0);

  check_forward_dist_ = nh_.param<double>("check_forward_dist", 0);
  check_side_dist_ = nh_.param<double>("check_side_dist", 0);
  double escape_degree = nh_.param<double>("escape_degree_range", 0);




  ROS_INFO("%s",cmd_vel_topic_name.c_str());
  ROS_INFO("%s",scan_topic_name.c_str());
  ROS_INFO("%s",odom_topic_name.c_str());
  ROS_INFO("%s",bumper_topic_name.c_str());
  ROS_INFO("%s",sonar_front_topic_name.c_str());
  ROS_INFO("%s",sonar_left_topic_name.c_str());
  ROS_INFO("%s",sonar_right_topic_name.c_str());
  ROS_INFO("%s",sonar_back_topic_name.c_str());

  ROS_INFO("linear velocity: %f",linear_vel);
  ROS_INFO("angular velocity: %f",angular_vel);

  ROS_INFO("check_forward_dist: %f",check_forward_dist_);
  ROS_INFO("check_side_dist: %f",check_side_dist_);
  ROS_INFO("escape_degree_range: %f",escape_degree);

  // initialize variables
  escape_range_       = escape_degree * DEG2RAD;
  LINEAR_VELOCITY = linear_vel;
  ANGULAR_VELOCITY = angular_vel;


  bot_pose_ = 0.0;
  prev_bot_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe(scan_topic_name, 10, &PiyabotDrive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe(odom_topic_name, 10, &PiyabotDrive::odomMsgCallBack, this);
  
  sonar_front_sub_ = nh_.subscribe(sonar_front_topic_name, 100, &PiyabotDrive::sonarFrontCallback, this);
  sonar_left_sub_ = nh_.subscribe(sonar_left_topic_name, 100, &PiyabotDrive::sonarLeftCallback, this);
  sonar_right_sub_ = nh_.subscribe(sonar_right_topic_name, 100, &PiyabotDrive::sonarRightCallback, this);
  sonar_back_sub_ = nh_.subscribe(sonar_back_topic_name, 100, &PiyabotDrive::sonarBackCallback, this);

  
  begin = ros::Time::now();
  bumper_sensor_sub_ = nh_.subscribe(bumper_topic_name, 1000, &PiyabotDrive::bumperCallback, this);
  
  return true;
}


void PiyabotDrive::sonarFrontCallback(const sensor_msgs::Range::ConstPtr &msg)
{
  //ROS_INFO("sonar front %f",msg->range);
  sonar_data_[CENTER] = msg->range;

}
void PiyabotDrive::sonarLeftCallback(const sensor_msgs::Range::ConstPtr &msg)
{
  //ROS_INFO("sonar left %f",msg->range);
  sonar_data_[LEFT] = msg->range;
}
void PiyabotDrive::sonarRightCallback(const sensor_msgs::Range::ConstPtr &msg)
{
  //ROS_INFO("sonar right %f",msg->range);
  sonar_data_[RIGHT] = msg->range;
}
void PiyabotDrive::sonarBackCallback(const sensor_msgs::Range::ConstPtr &msg)
{
  //ROS_INFO("sonar back %f",msg->range);
  sonar_data_[BACK] = msg->range;
}


void PiyabotDrive::bumperCallback(const gazebo_msgs::ContactsStateConstPtr &cs)
{

  if( cs->states.size() > 0 )
  { 
      // ROS_INFO("The bumper ContactState message size is: %d", a);
      ROS_INFO("Robot hit!!!");
      end = ros::Time::now();

      geometry_msgs::Twist cmd_vel;

      cmd_vel.linear.x  = 0;
      cmd_vel.angular.z = 0;

      cmd_vel_pub_.publish(cmd_vel);

      ros::Duration diff=end-begin;
      std::cout<<"Total time run: "<<diff<<std::endl;

      bumper_sensor_sub_.shutdown();

      piyabot_state_num = BOT_DRIVE_STOP;
      //double z = cs->states[0].total_wrench.force.z; //here is for you question 1).
  }

}

void PiyabotDrive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	bot_pose_ = atan2(siny, cosy);
}

void PiyabotDrive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
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

void PiyabotDrive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool PiyabotDrive::controlLoop()
{
  

  switch(piyabot_state_num)
  {
    case GET_BOT_DIRECTION:
      if (scan_data_[CENTER] > check_forward_dist_)
      {

        if (scan_data_[LEFT] < check_side_dist_ 
          && scan_data_[RIGHT] < check_side_dist_
         )
         {
           if (scan_data_[RIGHT] > scan_data_[LEFT] )
           {
              prev_bot_pose_ = bot_pose_;
              piyabot_state_num = BOT_RIGHT_TURN;
           }else{
              prev_bot_pose_ = bot_pose_;
              piyabot_state_num = BOT_LEFT_TURN;
           }

         }else if (scan_data_[RIGHT] < check_side_dist_)
        {
          prev_bot_pose_ = bot_pose_;
          piyabot_state_num = BOT_LEFT_TURN;
        }
        else if (scan_data_[LEFT] < check_side_dist_)
        {
          prev_bot_pose_ = bot_pose_;
          piyabot_state_num = BOT_RIGHT_TURN;
        }
        else
        {
          piyabot_state_num = BOT_DRIVE_FORWARD;
          prev_bot_pose_ = bot_pose_;
        }

        
      }

      if (scan_data_[CENTER] < check_forward_dist_)
      {

        if (scan_data_[LEFT] < check_side_dist_ 
          && scan_data_[RIGHT] < check_side_dist_
         )
        {
          piyabot_state_num = BOT_DRIVE_BACKWARD;
          prev_bot_pose_ = bot_pose_;

        }else if (scan_data_[LEFT] < check_side_dist_)
        {
          prev_bot_pose_ = bot_pose_;
          piyabot_state_num = BOT_RIGHT_TURN;
        }
        else if (scan_data_[RIGHT] < check_side_dist_)
        {
          prev_bot_pose_ = bot_pose_;
          piyabot_state_num = BOT_LEFT_TURN;
        }
        else
        {
           piyabot_state_num = BOT_DRIVE_BACKWARD;

        }


      }


      break;

    case BOT_DRIVE_FORWARD:
      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
      piyabot_state_num = GET_BOT_DIRECTION;
      break;

    case BOT_DRIVE_BACKWARD:
      updatecommandVelocity(-LINEAR_VELOCITY, 0.0);

      //piyabot_state_num = GET_BOT_DIRECTION;
      if (scan_data_[CENTER] > check_forward_dist_)
      {
        if (scan_data_[LEFT] > check_side_dist_)
        {
          prev_bot_pose_ = bot_pose_;
          piyabot_state_num = BOT_LEFT_TURN;
        }
        else if (scan_data_[RIGHT] > check_side_dist_)
        {
          prev_bot_pose_ = bot_pose_;
          piyabot_state_num = BOT_RIGHT_TURN;
        }
        else
        {
          piyabot_state_num = BOT_DRIVE_FORWARD;
        }

      }

      break;

    case BOT_RIGHT_TURN:
      if (fabs(prev_bot_pose_ - bot_pose_) >= escape_range_)
        piyabot_state_num = GET_BOT_DIRECTION;
      else
        updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
      break;

    case BOT_LEFT_TURN:
      if (fabs(prev_bot_pose_ - bot_pose_) >= escape_range_)
        piyabot_state_num = GET_BOT_DIRECTION;
      else
        updatecommandVelocity(0.0, ANGULAR_VELOCITY);
      break;

    case BOT_DRIVE_STOP:

    break;
    default:
      piyabot_state_num = GET_BOT_DIRECTION;
      break;
  }

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "piyabot_drive");
  PiyabotDrive piyabot_drive;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    piyabot_drive.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();

    if(piyabot_drive.getRobotState() == BOT_DRIVE_STOP)
    {
      break;
    }
  }

  return 0;
}
