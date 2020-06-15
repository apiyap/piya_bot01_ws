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

#ifndef PIYABOT_DRIVE_H_
#define PIYABOT_DRIVE_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <gazebo/gazebo.hh>
#include <sensor_msgs/Range.h>
#include <gazebo_msgs/ContactState.h>
#include <gazebo_msgs/ContactsState.h>

#include <iostream>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2
#define BACK   3

//#define LINEAR_VELOCITY  0.8
//#define ANGULAR_VELOCITY 1.5

#define GET_BOT_DIRECTION 0
#define BOT_DRIVE_FORWARD 1
#define BOT_RIGHT_TURN    2
#define BOT_LEFT_TURN     3
#define BOT_DRIVE_BACKWARD 4
#define BOT_DRIVE_STOP 10

class PiyabotDrive
{
 public:
  PiyabotDrive();
  ~PiyabotDrive();
  bool init();
  bool controlLoop();

  uint8_t getRobotState(){
    return piyabot_state_num;
  };
 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters
  ros::Time begin;
  ros::Time end;
  // ROS Time

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber bumper_sensor_sub_;
  ros::Subscriber sonar_front_sub_;
  ros::Subscriber sonar_left_sub_;
  ros::Subscriber sonar_right_sub_;
  ros::Subscriber sonar_back_sub_;

  //Configuration
  double LINEAR_VELOCITY = 0.8;
  double ANGULAR_VELOCITY = 1.5;
  
  // Variables
  double escape_range_;
  double check_forward_dist_;
  double check_side_dist_;

  double scan_data_[3] = {0.0, 0.0, 0.0};
  double sonar_data_[4] = {2.0, 2.0, 2.0, 2.0};

  double bot_pose_;
  double prev_bot_pose_;

  uint8_t piyabot_state_num = 0;

  // Function prototypes
  void updatecommandVelocity(double linear, double angular);
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
  void bumperCallback(const gazebo_msgs::ContactsStateConstPtr &cs);
  void sonarFrontCallback(const sensor_msgs::Range::ConstPtr &msg);
  void sonarLeftCallback(const sensor_msgs::Range::ConstPtr &msg);
  void sonarRightCallback(const sensor_msgs::Range::ConstPtr &msg);
  void sonarBackCallback(const sensor_msgs::Range::ConstPtr &msg);

};
#endif // PIYABOT_DRIVE_H_
