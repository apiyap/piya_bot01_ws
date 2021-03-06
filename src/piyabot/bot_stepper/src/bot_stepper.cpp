#include<bot_stepper/bot_stepper.h>



PiyaBotDrive::PiyaBotDrive()
: nh_priv_("~")
{
  //Init fake turtlebot node
  bool init_result = init();
  ROS_ASSERT(init_result);
}

PiyaBotDrive::~PiyaBotDrive()
{

}





/*******************************************************************************
* Init function
*******************************************************************************/
bool PiyaBotDrive::init()
{
  // initialize ROS parameter

  std::string robot_model = nh_.param<std::string>("piyabot_model", "");

  if (!robot_model.compare("delta"))
  {
    wheel_seperation_ = 0.160;
    turning_radius_   = 0.080;
    robot_radius_     = 0.105;
  }
  else 
  {
    wheel_seperation_ = 0.287;
    turning_radius_   = 0.1435;
    robot_radius_     = 0.220;
  }

  nh_.param("wheel_left_joint_name", joint_states_name_[LEFT],  std::string("left_wheel_hinge"));
  nh_.param("wheel_right_joint_name", joint_states_name_[RIGHT],  std::string("right_wheel_hinge"));
  nh_.param("joint_states_frame", joint_states_.header.frame_id, std::string("base_joint"));
  nh_.param("odom_frame", odom_.header.frame_id, std::string("odom"));
  nh_.param("base_frame", odom_.child_frame_id, std::string("base_footprint"));

  std::string imu_topic_name;

  nh_.param("imu_topic_name", imu_topic_name, std::string("imu"));



  // initialize variables
  wheel_speed_cmd_[LEFT]  = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;
  goal_linear_velocity_   = 0.0;
  goal_angular_velocity_  = 0.0;
  cmd_vel_timeout_        = 1.0;
  last_position_[LEFT]    = 0.0;
  last_position_[RIGHT]   = 0.0;
  last_velocity_[LEFT]    = 0.0;
  last_velocity_[RIGHT]   = 0.0;

  double pcov[36] = { 0.1,   0,   0,   0,   0, 0,
                        0, 0.1,   0,   0,   0, 0,
                        0,   0, 1e6,   0,   0, 0,
                        0,   0,   0, 1e6,   0, 0,
                        0,   0,   0,   0, 1e6, 0,
                        0,   0,   0,   0,   0, 0.2};
  memcpy(&(odom_.pose.covariance),pcov,sizeof(double)*36);
  memcpy(&(odom_.twist.covariance),pcov,sizeof(double)*36);

  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;

  odom_vel_[0] = 0.0;
  odom_vel_[1] = 0.0;
  odom_vel_[2] = 0.0;

  joint_states_.name.push_back(joint_states_name_[LEFT]);
  joint_states_.name.push_back(joint_states_name_[RIGHT]);
  joint_states_.position.resize(2,0.0);
  joint_states_.velocity.resize(2,0.0);
  joint_states_.effort.resize(2,0.0);

  // initialize publishers
  joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
  odom_pub_         = nh_.advertise<nav_msgs::Odometry>("odom", 100);
  motor_cmd_pub_ = nh_.advertise<piyabot_msgs::MotorCommand>("motor_cmd", 100);

  // initialize subscribers
  cmd_vel_sub_  = nh_.subscribe("cmd_vel", 100,  &PiyaBotDrive::commandVelocityCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic_name, 10,  &PiyaBotDrive::imuCallback, this);



  prev_update_time_ = ros::Time::now();

  return true;
}

void PiyaBotDrive::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // imu_msg.orientation.x = msg->orientation.x;
    // imu_msg.orientation.y = msg->orientation.y; 
    // imu_msg.orientation.z = msg->orientation.z;
    // imu_msg.orientation.w = msg->orientation.w;
    // imu_msg.angular_velocity.x = msg->angular_velocity.x;
    // imu_msg.angular_velocity.y = msg->angular_velocity.y;
    // imu_msg.angular_velocity.z = msg->angular_velocity.z;
    // imu_msg.linear_acceleration.x = msg->linear_acceleration.x;
    // imu_msg.linear_acceleration.y = msg->linear_acceleration.y;
    // imu_msg.linear_acceleration.z = msg->linear_acceleration.z;
    orientation[0] = msg->orientation.x;
    orientation[1] = msg->orientation.y;
    orientation[2] = msg->orientation.z;
    orientation[3] = msg->orientation.w;
}

/*******************************************************************************
* Callback function for cmd_vel msg
*******************************************************************************/
void PiyaBotDrive::commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg)
{
  last_cmd_vel_time_ = ros::Time::now();

  goal_linear_velocity_  = cmd_vel_msg->linear.x;
  goal_angular_velocity_ = cmd_vel_msg->angular.z;

  wheel_speed_cmd_[LEFT]  = goal_linear_velocity_ - (goal_angular_velocity_ * wheel_seperation_ / 2);
  wheel_speed_cmd_[RIGHT] = goal_linear_velocity_ + (goal_angular_velocity_ * wheel_seperation_ / 2);
}

/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool PiyaBotDrive::updateOdometry(ros::Duration diff_time)
{
  double wheel_l, wheel_r; // rotation value of wheel [rad]
  double delta_s, delta_theta, theta;
  double v[2], w[2];
  static double last_theta = 0.0;

  wheel_l = wheel_r     = 0.0;
  delta_s = delta_theta = 0.0;

  v[LEFT]  = wheel_speed_cmd_[LEFT];
  w[LEFT]  = v[LEFT] / WHEEL_RADIUS;  // w = v / r
  v[RIGHT] = wheel_speed_cmd_[RIGHT];
  w[RIGHT] = v[RIGHT] / WHEEL_RADIUS;

  last_velocity_[LEFT]  = w[LEFT];
  last_velocity_[RIGHT] = w[RIGHT];

  wheel_l = w[LEFT]  * diff_time.toSec();
  wheel_r = w[RIGHT] * diff_time.toSec();

  if(isnan(wheel_l))
  {
    wheel_l = 0.0;
  }

  if(isnan(wheel_r))
  {
    wheel_r = 0.0;
  }

  last_position_[LEFT]  += wheel_l;
  last_position_[RIGHT] += wheel_r;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  //delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / wheel_seperation_;
//<----
  //orientation = sensors.getOrientation();
  theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);
  delta_theta = theta - last_theta;
//<---

  // compute odometric pose
  odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[2] += delta_theta;

  // compute odometric instantaneouse velocity
  odom_vel_[0] = delta_s / diff_time.toSec();     // v
  odom_vel_[1] = 0.0;
  odom_vel_[2] = delta_theta / diff_time.toSec(); // w

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose_[2]);

  // We should update the twist of the odometry
  odom_.twist.twist.linear.x  = odom_vel_[0];
  odom_.twist.twist.angular.z = odom_vel_[2];

  last_theta = theta;

  return true;
}

/*******************************************************************************
* Calculate the joint states
*******************************************************************************/
void PiyaBotDrive::updateJoint(void)
{
  joint_states_.position[LEFT]  = last_position_[LEFT];
  joint_states_.position[RIGHT] = last_position_[RIGHT];
  joint_states_.velocity[LEFT]  = last_velocity_[LEFT];
  joint_states_.velocity[RIGHT] = last_velocity_[RIGHT];
}

/*******************************************************************************
* Calculate the TF
*******************************************************************************/
void PiyaBotDrive::updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom_.header;
  odom_tf.child_frame_id = odom_.child_frame_id;
  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;
}

/*******************************************************************************
* Update function
*******************************************************************************/
bool PiyaBotDrive::update()
{
  ros::Time time_now = ros::Time::now();
  ros::Duration step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  // zero-ing after timeout
  if((time_now - last_cmd_vel_time_).toSec() > cmd_vel_timeout_)
  {
    wheel_speed_cmd_[LEFT]  = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
  }

// motor command
  motor_cmd_.mode = piyabot_msgs::MotorCommand::SPEED;
  motor_cmd_.left_motor = (int)(wheel_speed_cmd_[LEFT] * MAX_SPEED_STEP_VALUE);
  motor_cmd_.right_motor = (int)(wheel_speed_cmd_[RIGHT] * MAX_SPEED_STEP_VALUE);
  motor_cmd_pub_.publish(motor_cmd_);


  // odom
  updateOdometry(step_time);
  odom_.header.stamp = time_now;
  odom_pub_.publish(odom_);

  // joint_states
  updateJoint();
  joint_states_.header.stamp = time_now;
  joint_states_pub_.publish(joint_states_);

  // tf
  geometry_msgs::TransformStamped odom_tf;
  updateTF(odom_tf);
  tf_broadcaster_.sendTransform(odom_tf);

  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "piyabot_stepper_drive_node");

  PiyaBotDrive bot_drive;

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    bot_drive.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

