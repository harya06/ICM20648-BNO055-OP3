/*******************************************************************************
 * Copyright 2017 ROBOTIS CO., LTD.
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

/* Author: Kayman Jung */

#ifndef MOVE_FORWART_H_
#define MOVE_FORWART_H_

#include <iostream>
#include <ctime>
#include <fstream>
#include <sstream>
#include <tf/tf.h>
#include <cmath>
#include <ctime>
#include <Eigen/Dense>
#include <ros/package.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>

#include "op3_action_module_msgs/IsRunning.h"
#include "robotis_controller_msgs/SensorYPR.h"
#include "robotis_controller_msgs/SyncWriteItem.h"
#include "robotis_controller_msgs/JointCtrlModule.h"
#include "robotis_controller_msgs/SetJointModule.h"

#include "op3_demo/op_demo.h"
#include "op3_demo/move_forwart_ball_tracker.h"
#include "op3_demo/move_forwart_ball_follower.h"
#include "robotis_math/robotis_linear_algebra.h"

namespace robotis_op
{

  class MoveForwart : public OPDemo
  {
  public:
    enum Stand_Status
    {
      Stand = 0,
      Fallen_Forward = 1,
      Fallen_Behind = 2,
    };

    enum Robot_Status
    {
      Waited = 0,
      TrackingAndFollowing = 1,
      ReadyToKick = 2,
      ReadyToCeremony = 3,
      ReadyToGetup = 4,
    };

    MoveForwart();
    ~MoveForwart();

    void setDemoEnable();
    void setDemoDisable();

  protected:
    const double FALL_FORWARD_LIMIT;
    const double FALL_BACK_LIMIT;
    const int SPIN_RATE;
    const bool DEBUG_PRINT;

    bool start_bf = false;
    double offset_yaw = 0.0;

    void processThread();
    void callbackThread();
    void trackingThread();

    void setBodyModuleToDemo(const std::string &body_module, bool with_head_control = true);
    void setModuleToDemo(const std::string &module_name);
    void callServiceSettingModule(const robotis_controller_msgs::JointCtrlModule &modules);
    void parseJointNameFromYaml(const std::string &path);
    bool getJointNameFromID(const int &id, std::string &joint_name);
    bool getIDFromJointName(const std::string &joint_name, int &id);
    int getJointCount();
    bool isHeadJoint(const int &id);
    void buttonHandlerCallback(const std_msgs::String::ConstPtr &msg);
    void demoCommandCallback(const std_msgs::String::ConstPtr &msg);
    void imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void sensorYPRCallback(const robotis_controller_msgs::SensorYPR::ConstPtr &msg);
    void gamecontrollerCallback(const std_msgs::Int32::ConstPtr &msg);
    void komunikasimoveforwartCallback(const std_msgs::Int32::ConstPtr &msg); // komunikasi
    void rpyCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);     // imu bno
    void rawGyroCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg); // imu bno

    void initialSoccerMode();
    void readySoccerMode();
    void setSoccerMode();
    void startSoccerMode();
    void waitSoccerMode(); // komunikasi
    void stopSoccerMode();
    std::string waktu();

    void process();
    void handleKick(int ball_position);
    void handleKick();
    bool handleFallen(int fallen_status);

    void playMotion(int motion_index);
    void setRGBLED(int blue, int green, int red);
    bool isActionRunning();

    void sendDebugTopic(const std::string &msgs);

    void updateIsEnabled(int isEnabled);

    MoveForwartBallTracker move_forwart_ball_tracker_;
    MoveForwartBallFollower move_forwart_ball_follower_;

    ros::Publisher module_control_pub_;
    ros::Publisher is_enable_pub_;
    ros::Publisher motion_index_pub_;
    ros::Publisher rgb_led_pub_;
    ros::Subscriber buttuon_sub_;
    ros::Subscriber demo_command_sub_;
    ros::Subscriber imu_data_sub_;
    ros::Subscriber ypr_data_sub_;
    ros::Subscriber comp_data_sub_;
    ros::Subscriber game_controller_sub_;
    ros::Subscriber komunikasimoveforwart_sub_; // komunikasi
    ros::Subscriber rpy_sub_;                   // imu bno
    ros::Subscriber gyro_sub_;
    ros::Subscriber rpy_bno_sub_;

    ros::Publisher reply_pub_;

    ros::Publisher test_pub_;

    ros::ServiceClient is_running_client_;
    ros::ServiceClient set_joint_module_client_;

    ros::Time start_time_;
    ros::Time last_log_time_;
    ros::Time start_time_bno_;
    ros::Time last_log_time_bno_;
    ros::Time start_time_gyro_;
    ros::Time last_log_time_gyro_;

    std::map<int, std::string> id_joint_table_;
    std::map<std::string, int> joint_id_table_;

    bool is_grass_;
    int wait_count_;
    bool on_following_ball_;
    bool on_tracking_ball_;
    bool restart_soccer_;
    bool start_following_;
    bool stop_following_;
    bool stop_fallen_check_;
    bool start_ready_;
    bool strategi_satu_;
    int robot_status_;
    int tracking_status_;
    int stand_state_;
    double present_pitch_;
    float comp_;
    int game_;
    int komunikasi_moveforwart;         // komunikasi
    int desired_komunikasi_moveforwart; // komunikasi
    bool log_initialized_;
    double log_duration_sec_; // lama waktu log
    std::ofstream imu_log_file_;
    bool log_initialized_bno_;
    double log_duration_sec_bno_; // lama waktu log
    std::ofstream imu_log_file_bno_;
    bool start_time_bno_initialized_;
    bool log_initialized_gyro_;
    double log_duration_sec_gyro_; // lama waktu log
    std::ofstream imu_log_file_gyro_;
    bool start_time_gyro_initialized_;
    double current_body_roll_;
    double current_body_pitch_;
    boost::mutex imu_mutex_;
  };

} // namespace robotis_op
#endif // MOVE_FORWART_H_
