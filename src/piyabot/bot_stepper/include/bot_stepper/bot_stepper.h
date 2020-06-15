#ifndef PIYABOT_STEPPER_DRIVE_H_
#define PIYABOT_STEPPER_DRIVE_H_

#include <math.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/BatteryState.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <piyabot_msgs/SensorState.h>
#include <piyabot_msgs/Sound.h>
#include <piyabot_msgs/VersionInfo.h>
#include <piyabot_msgs/MotorCommand.h>
#include <piyabot_msgs/MotorConfig.h>

#include <string>
#include <sstream>



#define WHEEL_RADIUS                    0.0325     // meter

#define LEFT                            0
#define RIGHT                           1

// #define MAX_LINEAR_VELOCITY             0.22   // m/s
// #define MAX_ANGULAR_VELOCITY            2.84   // rad/s

#define MAX_SPEED_STEP_VALUE                10000


class PiyaBotDrive
{
    public:
        PiyaBotDrive();
        ~PiyaBotDrive();
        bool init();
        bool update();

    private:
        // ROS NodeHandle
        ros::NodeHandle nh_;
        ros::NodeHandle nh_priv_;


        // ROS Parameters
        // (TODO)

        // ROS Time
        ros::Time last_cmd_vel_time_;
        ros::Time prev_update_time_;

        // ROS Topic Publishers
        ros::Publisher joint_states_pub_;
        ros::Publisher odom_pub_;

        ros::Publisher motor_cmd_pub_;


        // ROS Topic Subscribers
        ros::Subscriber cmd_vel_sub_;
        ros::Subscriber imu_sub_;


        sensor_msgs::JointState joint_states_;
        nav_msgs::Odometry odom_;
        tf::TransformBroadcaster tf_broadcaster_;

        piyabot_msgs::MotorCommand motor_cmd_;
        piyabot_msgs::MotorConfig  motor_config_;

        double wheel_speed_cmd_[2];
        double goal_linear_velocity_;
        double goal_angular_velocity_;
        double cmd_vel_timeout_;

        float  odom_pose_[3];
        float  odom_vel_[3];
        double pose_cov_[36];

        std::string joint_states_name_[2];

        double last_position_[2];
        double last_velocity_[2];

        double wheel_seperation_;
        double turning_radius_;
        double robot_radius_;

        // Function prototypes
        void commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg);
        bool updateOdometry(ros::Duration diff_time);
        void updateJoint(void);
        void updateTF(geometry_msgs::TransformStamped& odom_tf);

        //sensor_msgs::Imu imu_msg;
        float orientation[4]= {0.0, 0.0, 0.0, 0.0};
        void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
        


};

#endif