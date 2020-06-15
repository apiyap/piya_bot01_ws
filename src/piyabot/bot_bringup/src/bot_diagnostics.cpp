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

/* Authors: Taehoon Lim (Darby) */

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <piyabot_msgs/SensorState.h>
#include <piyabot_msgs/VersionInfo.h>
#include <string>
#include <sstream>

#define SOFTWARE_VERSION "0.0.1"
#define HARDWARE_VERSION "2020.05.19"
#define FIRMWARE_VERSION_MAJOR_NUMBER 0
#define FIRMWARE_VERSION_MINOR_NUMBER 0

ros::Publisher _version_info_pub;
ros::Publisher _diagnostics_pub;

diagnostic_msgs::DiagnosticStatus imu_state;
diagnostic_msgs::DiagnosticStatus motor_state;
diagnostic_msgs::DiagnosticStatus LDS_state;
diagnostic_msgs::DiagnosticStatus battery_state;
diagnostic_msgs::DiagnosticStatus button_state;
diagnostic_msgs::DiagnosticStatus led_state;
diagnostic_msgs::DiagnosticStatus sonar_state;


typedef struct
{
  int major_number;
  int minor_number;
  int patch_number;
}VERSION;

void split(std::string data, std::string separator, std::string* temp)
{
	int cnt = 0;
  std::string copy = data;
  
	while(true)
	{
		std::size_t index = copy.find(separator);

    if (index != std::string::npos)
    {
      temp[cnt] = copy.substr(0, index);

      copy = copy.substr(index+1, copy.length());
    }
    else
    {
      temp[cnt] = copy.substr(0, copy.length());
      break;
    }
    
		++cnt;
	}
}

void setDiagnosisMsg(diagnostic_msgs::DiagnosticStatus *diag, uint8_t level, std::string name, std::string message, std::string hardware_id)
{
  diag->level = level;
  diag->name  = name;
  diag->message = message;
  diag->hardware_id = hardware_id;
}

void setLedDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&led_state, level, "LED", message, "Mode");
}

void setSonarDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&sonar_state, level, "Sonar", message, "Value");
}


void setIMUDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&imu_state, level, "IMU Sensor", message, "MPU9250");
}

void setMotorDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&motor_state, level, "Actuator", message, "Stepper motor");
}

void setBatteryDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&battery_state, level, "Power System", message, "Battery");
}

void setLDSDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&LDS_state, level, "Lidar Sensor", message, "HLS-LFCD-LDS");
}

void setButtonDiagnosis(uint8_t level, std::string message)
{
  setDiagnosisMsg(&button_state, level, "Analog Button", message, "Button");
}

void imuMsgCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
  setIMUDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

void LDSMsgCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  setLDSDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
}

void sensorStateMsgCallback(const piyabot_msgs::SensorState::ConstPtr &msg)
{
  if (msg->battery > 11.0)
    setBatteryDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Good Condition");
  else
    setBatteryDiagnosis(diagnostic_msgs::DiagnosticStatus::WARN, "Charge!!! Charge!!!");

  if (msg->button == piyabot_msgs::SensorState::BUTTON0)
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "BUTTON 0 IS PUSHED");
  else if (msg->button == piyabot_msgs::SensorState::BUTTON1)
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "BUTTON 1 IS PUSHED");
  else
    setButtonDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Pushed Nothing");

  setMotorDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Steeper Motor");

  std::ostringstream oss;
  oss.str(std::string());
  motor_state.values.clear();

  diagnostic_msgs::KeyValue torque;
    torque.key = "Motor torque";
    torque.value = (msg->torque)? "ON" : "OFF";

  motor_state.values.push_back(torque);

  diagnostic_msgs::KeyValue left_speed;
  left_speed.key="left_speed";
  left_speed.value = std::to_string(msg->left_speed);
  motor_state.values.push_back(left_speed);

  diagnostic_msgs::KeyValue right_speed;
  right_speed.key="right_speed";
  right_speed.value = std::to_string(msg->right_speed);
  motor_state.values.push_back(right_speed);

  diagnostic_msgs::KeyValue left_encoder;
  left_encoder.key="left_encoder";
  left_encoder.value = std::to_string(msg->left_encoder);
  motor_state.values.push_back(left_encoder);

  diagnostic_msgs::KeyValue right_encoder;
  right_encoder.key="right_encoder";
  right_encoder.value = std::to_string(msg->right_encoder);
  motor_state.values.push_back(right_encoder);


  oss.str(std::string());
  oss <<"led:"<< msg->led;
  setLedDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, oss.str());

  // setSonarDiagnosis(diagnostic_msgs::DiagnosticStatus::OK, "Sonars sensor");
  
  // sonar_state.values.clear();

  // diagnostic_msgs::KeyValue sonar_front;
  // sonar_front.key="front";
  // sonar_front.value = std::to_string(msg->sonar_front);
  // sonar_state.values.push_back(sonar_front);

  // diagnostic_msgs::KeyValue sonar_left;
  // sonar_left.key="left";
  // sonar_left.value = std::to_string(msg->sonar_left);
  // sonar_state.values.push_back(sonar_left);

  // diagnostic_msgs::KeyValue sonar_right;
  // sonar_right.key="right";
  // sonar_right.value = std::to_string(msg->sonar_right);
  // sonar_state.values.push_back(sonar_right);

  // diagnostic_msgs::KeyValue sonar_back;
  // sonar_back.key="back";
  // sonar_back.value = std::to_string(msg->sonar_back);
  // sonar_state.values.push_back(sonar_back);

}

void firmwareVersionMsgCallback(const piyabot_msgs::VersionInfo::ConstPtr &msg)
{
  static bool check_version = false;
  std::string get_version[3];

  split(msg->firmware, ".", get_version);

  VERSION firmware_version; 
  firmware_version.major_number = std::stoi(get_version[0]);
  firmware_version.minor_number = std::stoi(get_version[1]);
  firmware_version.patch_number = std::stoi(get_version[2]);

  if (check_version == false)
  {
    if (firmware_version.major_number == FIRMWARE_VERSION_MAJOR_NUMBER)
    {
      if (firmware_version.minor_number > FIRMWARE_VERSION_MINOR_NUMBER)
      {
        ROS_WARN("This firmware(v%s) isn't compatible with this software (v%s)", msg->firmware.data(), SOFTWARE_VERSION);
        ROS_WARN("You can find how to update its in `FAQ` section(turtlebot3.robotis.com)");
      }
    }
    else
    {
      ROS_WARN("This firmware(v%s) isn't compatible with this software (v%s)", msg->firmware.data(), SOFTWARE_VERSION);
      ROS_WARN("You can find how to update its in `FAQ` section(turtlebot3.robotis.com)");
    }

    check_version = true;
  }
  
  piyabot_msgs::VersionInfo version;

  version.software = SOFTWARE_VERSION;
  version.hardware = HARDWARE_VERSION;
  version.firmware = msg->firmware;

  _version_info_pub.publish(version);
}

void msgPub()
{
  diagnostic_msgs::DiagnosticArray diagnostics;

  diagnostics.header.stamp = ros::Time::now();

  diagnostics.status.clear();
  diagnostics.status.push_back(imu_state);
  diagnostics.status.push_back(motor_state);
  diagnostics.status.push_back(LDS_state);
  diagnostics.status.push_back(battery_state);
  diagnostics.status.push_back(button_state);
  diagnostics.status.push_back(led_state);
  diagnostics.status.push_back(sonar_state);

  _diagnostics_pub.publish(diagnostics);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bot_diagnostic");
  ros::NodeHandle nh;

  _diagnostics_pub  = nh.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);
  _version_info_pub = nh.advertise<piyabot_msgs::VersionInfo>("version_info", 10);

  ros::Subscriber imu         = nh.subscribe("imu", 10, imuMsgCallback);
  ros::Subscriber lds         = nh.subscribe("scan", 10, LDSMsgCallback);
  ros::Subscriber tb3_sensor  = nh.subscribe("sensor_state", 10, sensorStateMsgCallback);
  ros::Subscriber version     = nh.subscribe("firmware_version", 10, firmwareVersionMsgCallback);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    msgPub();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
