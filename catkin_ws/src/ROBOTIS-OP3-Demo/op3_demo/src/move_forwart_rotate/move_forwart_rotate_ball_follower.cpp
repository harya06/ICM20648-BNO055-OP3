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

#include "op3_demo/move_forwart_rotate_ball_follower.h"
#include <iostream>

namespace robotis_op
{

  MoveForwartRotateBallFollower::MoveForwartRotateBallFollower()
      : nh_(ros::this_node::getName()),
        FOV_WIDTH(35.2 * M_PI / 180),
        FOV_HEIGHT(21.6 * M_PI / 180),
        count_not_found_(0),
        count_to_kick_(0),
        on_tracking_(false),
        first_kick_(0),
        approach_ball_position_(NotFound),
        kick_motion_index_(83),
        CAMERA_HEIGHT(0.46),
        NOT_FOUND_THRESHOLD(100),
        MAX_FB_STEP(55.0 * 0.001), // 40
        MAX_RL_TURN(20.0 * M_PI / 180),
        IN_PLACE_FB_STEP(0.0 * 0.001), // 3
        MIN_FB_STEP(12.0 * 0.001),     // 5
        MIN_RL_TURN(5.0 * M_PI / 180),
        UNIT_FB_STEP(5.0 * 0.001), // 2
        UNIT_RL_TURN(0.5 * M_PI / 180),
        SPOT_FB_OFFSET(0.0 * 0.001),
        SPOT_RL_OFFSET(0.45 * 0.001), // 0.45
        SPOT_ANGLE_OFFSET(0.0),
        hip_pitch_offset_(7.0),
        current_pan_(-10),
        current_tilt_(-10),
        current_x_move_(0.005),
        current_r_angle_(0),
        curr_period_time_(0.6),
        accum_period_time_(0.0),
        imu(0),
        arah(0),
        komunikasi(0),
        // FALL_FORWARD_LIMIT(60), //bangun
        // FALL_BACK_LIMIT(-60), //bangun
        DEBUG_PRINT(false)
  {
    current_joint_states_sub_ = nh_.subscribe("/robotis/goal_joint_states", 10, &MoveForwartRotateBallFollower::currentJointStatesCallback,
                                              this);
    chatter_sub_ = nh_.subscribe("/chatter", 10, &MoveForwartRotateBallFollower::chatterCallback, this); // kompas
    ypr_data_sub_ = nh_.subscribe("/sensor_ypr", 1, &MoveForwartRotateBallFollower::sensorYPRCallback, this);
    // imu_data_sub_ = nh_.subscribe("/robotis/open_cr/imu", 1, &MoveForwartRotateBallFollower::imuDataCallback, this); //bangun
    komunikasi_sub_ = nh_.subscribe("/receive", 10, &MoveForwartRotateBallFollower::komunikasiCallback, this); // komunikasi
    set_walking_command_pub_ = nh_.advertise<std_msgs::String>("/robotis/walking/command", 0);
    set_walking_param_pub_ = nh_.advertise<op3_walking_module_msgs::WalkingParam>("/robotis/walking/set_params", 0);
    get_walking_param_client_ = nh_.serviceClient<op3_walking_module_msgs::GetWalkingParam>(
        "/robotis/walking/get_params");
    reply_pub_ = nh_.advertise<std_msgs::Int32>("/reply", 10); // balasan komunikasi

    prev_time_ = ros::Time::now();
  }

  MoveForwartRotateBallFollower::~MoveForwartRotateBallFollower()
  {
  }

  void MoveForwartRotateBallFollower::muterkanan()
  {
    // ROS_INFO ("muterr kanan GOFRONTTTTTTTTTTTTTTTTTTTTT");
    current_walking_param_.z_move_amplitude = 0.065;
    // current_walking_param_.hip_pitch_offset = 12* M_PI / 180;
    setWalkingParam(-0.001, -0.020, 0.08, true); //-0.001, -0.015, 0.06
    // ROS_INFO("Arah IMU kanann GOFRONTT: %d", arah);

    set_walking_param_pub_.publish(current_walking_param_);
  }

  void MoveForwartRotateBallFollower::muterkiri()
  {
    // ROS_INFO ("muterr kiri GOFRONTTTTTTTTTTTTTTTTTTTTT");
    current_walking_param_.z_move_amplitude = 0.065;
    // current_walking_param_.hip_pitch_offset = 12* M_PI / 180;
    // ROS_INFO("Arah IMU kirii GOFRONTT: %d", arah);
    setWalkingParam(-0.001, 0.021, -0.08, true); //-0.001, 0.015, -0.05

    set_walking_param_pub_.publish(current_walking_param_);
  }

  void MoveForwartRotateBallFollower::majuReady()
  {
    current_walking_param_.x_move_amplitude = 20.0 * 0.001;
    current_walking_param_.y_move_amplitude = 0;
    current_walking_param_.angle_move_amplitude = 0;
    // current_walking_param_.z_move_amplitude = 0.06;
    // current_walking_param_.hip_pitch_offset = 12* M_PI / 180;

    set_walking_param_pub_.publish(current_walking_param_);
  }

  void MoveForwartRotateBallFollower::jalanTempat()
  {
    // current_walking_param_.balance_enable = balance;
    current_walking_param_.z_move_amplitude = 0.065;
    current_walking_param_.angle_move_amplitude = 0.0; // dikit
    current_walking_param_.x_move_amplitude = 0.000;   // di bikin -
    current_walking_param_.y_move_amplitude = 0.00;    // kanan -, kiri +

    set_walking_param_pub_.publish(current_walking_param_);
  }

  void MoveForwartRotateBallFollower::startSet()
  {
    //  approach_ball_position_ = NotFound;
    count_to_kick_ = 0;
    //  accum_ball_position_ = 0;
    ROS_INFO("Stop Ball following");

    setWalkingCommand("stop");
  }

  void MoveForwartRotateBallFollower::startFollowing()
  {

    on_tracking_ = true;
    ROS_INFO("Start Ball following");

    setWalkingCommand("start");

    bool result = getWalkingParam();
    if (result == true)
    {
      hip_pitch_offset_ = current_walking_param_.hip_pitch_offset;
      curr_period_time_ = current_walking_param_.period_time;

      // jalan maju lurus
      // setWalkingParam(0.02, 0.0, 0.0, true);
      // usleep(20000000);

      // //jalan mutar
      setWalkingParam(-0.002, -0.018, 0.06, true);
      usleep(31500000);

      // jalan maju serong kanan
      // setWalkingParam(0.02, 0.0, -0.03, true);
      // usleep(15000000);

      setWalkingCommand("stop");
    }
    else
    {
      hip_pitch_offset_ = current_walking_param_.hip_pitch_offset;
      curr_period_time_ = current_walking_param_.period_time;
    }
  }

  void MoveForwartRotateBallFollower::stopFollowing()
  {
    on_tracking_ = false;
    //  approach_ball_position_ = NotFound;
    count_to_kick_ = 0;
    //  accum_ball_position_ = 0;
    ROS_INFO("Stop Ball following");

    setWalkingCommand("stop");
    // CompassInit();
  }

  void MoveForwartRotateBallFollower::chatterCallback(const std_msgs::String::ConstPtr &msg) // kompas
  {
    // ROS_INFO("data kompas MoveForwartRotateBallFollower: %s", msg->data.c_str());
    int mag = std::atoi(msg->data.c_str());
    imu = mag;
  }

  void MoveForwartRotateBallFollower::sensorYPRCallback(const robotis_controller_msgs::SensorYPR::ConstPtr &msg)
  {
    // ROS_INFO("Roll : %f", (360 - (msg->yaw<0?180+(180+(msg->yaw/1800*180)):(msg->yaw/1800*180))));
    int goal = float(360 - (msg->yaw < 0 ? 180 + (180 + (msg->yaw / 1800 * 180)) : (msg->yaw / 1800 * 180)));

    if (!start_bf)
    {
      offset_yaw = float(360 - (msg->yaw < 0 ? 180 + (180 + (msg->yaw / 1800 * 180)) : (msg->yaw / 1800 * 180)));
      start_bf = true;
    }
    arah = goal - offset_yaw;
    if (arah < 0)
    {
      arah = 360 + arah;
    }
    ROS_INFO("Arah IMU GOFRONTT: %d", arah);
    ROS_INFO("FFFFFFFFFFFFFFFFFFFFFFFFFFFFBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB : %6.4f", current_pan_);
  }

  void MoveForwartRotateBallFollower::komunikasiCallback(const std_msgs::Int32::ConstPtr &msg)
  {
    ROS_INFO("Balasan Komunikasi BFFFFFFF: %d", msg->data);
    komunikasi = msg->data;
  }

  void MoveForwartRotateBallFollower::currentJointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
  {
    double pan, tilt;
    int get_count = 0;

    for (int ix = 0; ix < msg->name.size(); ix++)
    {
      if (msg->name[ix] == "head_pan")
      {
        pan = msg->position[ix];
        get_count += 1;
      }
      else if (msg->name[ix] == "head_tilt")
      {
        tilt = msg->position[ix];
        get_count += 1;
      }

      if (get_count == 2)
        break;
    }

    // check variation
    current_pan_ = pan;
    current_tilt_ = tilt;
  }

  void MoveForwartRotateBallFollower::calcFootstep(double target_distance, double target_angle, double delta_time,
                                                   double &fb_move, double &rl_angle)
  {
    // clac fb
    double next_movement = current_x_move_;
    if (target_distance < 0)
      target_distance = 0.0;

    double fb_goal = fmin(target_distance * 0.1, MAX_FB_STEP);
    accum_period_time_ += delta_time;
    if (accum_period_time_ > (curr_period_time_ / 4))
    {
      accum_period_time_ = 0.0;
      if ((target_distance * 0.1 / 2) < current_x_move_)
        next_movement -= UNIT_FB_STEP;
      else
        next_movement += UNIT_FB_STEP;
    }
    fb_goal = fmin(next_movement, fb_goal);
    fb_move = fmax(fb_goal, MIN_FB_STEP);
    ROS_INFO_COND(DEBUG_PRINT, "distance to ball : %6.4f, fb : %6.4f, delta : %6.6f", target_distance, fb_move,
                  delta_time);

    ROS_INFO_COND(DEBUG_PRINT, "==============================================");

    // calc rl angle
    double rl_goal = 0.0;
    if (fabs(target_angle) * 180 / M_PI > 5.0)
    {
      double rl_offset = fabs(target_angle) * 0.2;
      rl_goal = fmin(rl_offset, MAX_RL_TURN);
      rl_goal = fmax(rl_goal, MIN_RL_TURN);
      rl_angle = fmin(fabs(current_r_angle_) + UNIT_RL_TURN, rl_goal);

      if (target_angle < 0)
        rl_angle *= (-1);
    }
  }

  void MoveForwartRotateBallFollower::calcwaitFootstep(double target_distance, double target_angle, double delta_time,
                                                       double &fb_move, double &rl_angle)
  {
    // calc rl angle
    double rl_goal = 0.0;
    if (fabs(target_angle) * 180 / M_PI > 5.0)
    {
      double rl_offset = fabs(target_angle) * 0.2;
      rl_goal = fmin(rl_offset, MAX_RL_TURN);
      rl_goal = fmax(rl_goal, MIN_RL_TURN);
      rl_angle = fmin(fabs(current_r_angle_) + UNIT_RL_TURN, rl_goal);

      if (target_angle < 0)
        rl_angle *= (-1);
    }
  }

  // x_angle : ball position (pan), y_angle : ball position (tilt), ball_size : angle of ball radius
  bool MoveForwartRotateBallFollower::processFollowing(double x_angle, double y_angle, double ball_size)
  {
    ros::Time curr_time = ros::Time::now();
    ros::Duration dur = curr_time - prev_time_;
    double delta_time = dur.nsec * 0.000000001 + dur.sec;
    prev_time_ = curr_time;

    count_not_found_ = 0;
    //  int ball_position_sum = 0;

    // check of getting head joints angle
    if (current_tilt_ == -10 && current_pan_ == -10)
    {
      ROS_ERROR("Failed to get current angle of head joints.");
      setWalkingCommand("stop");

      on_tracking_ = false;
      approach_ball_position_ = NotFound;
      return false;
    }

    ROS_INFO_COND(DEBUG_PRINT, "   ============== Head | Ball ==============   ");
    ROS_INFO_STREAM_COND(DEBUG_PRINT,
                         "== Head Pan : " << (current_pan_ * 180 / M_PI) << " | Ball X : " << (x_angle * 180 / M_PI));
    ROS_INFO_STREAM_COND(DEBUG_PRINT,
                         "== Head Tilt : " << (current_tilt_ * 180 / M_PI) << " | Ball Y : " << (y_angle * 180 / M_PI));

    approach_ball_position_ = OutOfRange;

    double distance_to_ball = CAMERA_HEIGHT * tan(M_PI * 0.5 + current_tilt_ - hip_pitch_offset_ - ball_size);

    double ball_y_angle = (current_tilt_ + y_angle) * 180 / M_PI; // 180
    double ball_x_angle = (current_pan_ + x_angle) * 180 / M_PI;  // 180
    if (distance_to_ball < 0)
    {
      distance_to_ball *= (-1);
    }

    double distance_to_kick = 0.14; // 0.16

    if (distance_to_ball <= 0.7) // balasan komunikasi
    {
      std_msgs::Int32 reply;
      reply.data = 4;
      reply_pub_.publish(reply);
      ROS_INFO("KIRMMMMMMMMMMMMMMMMMMMMMM");
    }

    if ((distance_to_ball < distance_to_kick) && (fabs(ball_x_angle) < 25.0)) // contoh: kalo ini  < 25 ////syarat mundur yg dibawah >= 25
    {
      count_to_kick_ += 1;

      ROS_INFO_STREAM_COND(DEBUG_PRINT,
                           "head pan : " << (current_pan_ * 180 / M_PI) << " | ball pan : " << (x_angle * 180 / M_PI));
      ROS_INFO_STREAM_COND(DEBUG_PRINT,
                           "head tilt : " << (current_tilt_ * 180 / M_PI) << " | ball tilt : " << (y_angle * 180 / M_PI));
      ROS_INFO_STREAM_COND(DEBUG_PRINT, "foot to kick : " << ball_x_angle);

      ROS_INFO("In range [%d | %f]", count_to_kick_, ball_x_angle);

      // ball queue
      //    if(ball_position_queue_.size() >= 5)
      //      ball_position_queue_.erase(ball_position_queue_.begin());

      //    ball_position_queue_.push_back((ball_x_angle > 0) ? 1 : -1);

      //  COMPASS ARAH LURUS
      //     if (first_kick_ == 0) {
      int tengah = 0;
      int limitKanan = tengah + 10; // 10
      int limitKiri = tengah - 10;  // 350
      int behind_ = tengah + 180;   // 180

      if (limitKiri < 0)
        limitKiri = limitKiri + 360;
      if (limitKanan > 360)
        limitKanan = limitKanan - 360;
      if (behind_ > 360)
        behind_ = behind_ - 360;

      if (count_to_kick_ > 5 && (arah >= limitKiri || arah <= limitKanan)) // 5
      {
        setWalkingCommand("stop");
        on_tracking_ = false;
        //        first_kick_ = false;       // buat dia menggiring taruh bool ini disini

        // check direction of the ball
        //      accum_ball_position_ = std::accumulate(ball_position_queue_.begin(), ball_position_queue_.end(), 0);

        //      if (accum_ball_position_ > 0)
        if (ball_x_angle > 0)
        {
          ROS_INFO_COND(DEBUG_PRINT, "Ready to kick : left"); // left
          ROS_INFO("TENDANGGGGGGGGGGGGGGGG LEFT ");
          approach_ball_position_ = OnLeft;
        }
        else
        {
          ROS_INFO_COND(DEBUG_PRINT, "Ready to kick : right"); // right
          ROS_INFO("TENDANGGGGGGGGGGGGGGGG RIGHT ");
          approach_ball_position_ = OnRight;
        }

        return true;
      }

      else if (count_to_kick_ > 2 && arah < limitKiri && arah >= behind_)
      {
        setWalkingParam(-0.001, 0.021, -0.08, true);
        // imuDataCallback(); //bangun
        return true;
      }

      else if (count_to_kick_ > 3 && arah > limitKanan && arah < behind_)
      {
        setWalkingParam(-0.001, -0.020, 0.08, true);
        // imuDataCallback(); //bangun
        return true;
      }

      //     }

      else if (count_to_kick_ > 1)
      {
        // ROS_INFO("COUNTTTTT11111111111111");  //1
        // if (ball_x_angle > 0)
        // accum_ball_position_ += 1;
        // else
        // accum_ball_position_ -= 1;

        // send message
        setWalkingParam(IN_PLACE_FB_STEP, 0, 0);
        return false;
      }
    }

    else
    {
      // ROS_INFO("COUNTTTTTTTTTTTTTTTT0");
      count_to_kick_ = 0;
      //    accum_ball_position_ = NotFound;
    }

    double fb_move = 0.0, rl_angle = 0.0;
    double distance_to_walk = distance_to_ball - distance_to_kick;
    double distance_to_backward = -0.008;
    calcFootstep(distance_to_walk, current_pan_, delta_time, fb_move, rl_angle);

    if (distance_to_ball <= 0.22 && ball_x_angle <= -25.0)
    {
      setWalkingParam(0, 0, -0.12, true);
      // imuDataCallback(); //bangun
      return true;
    }
    else if (distance_to_ball <= 0.22 && ball_x_angle >= 25.0)
    {
      setWalkingParam(0, 0, 0.12, true);
      // imuDataCallback(); //bangun
      return true;
    }
    else if (distance_to_ball <= 0.07 && (fabs(ball_x_angle) >= 25.0))
    {
      setWalkingParam(-0.005, 0, 0);
      // imuDataCallback(); //bangun
      return false;
    }
    else
    {
      setWalkingParam(fb_move, 0, rl_angle);
      if (fb_move >= 0.03 && fb_move <= 0.039)
      {
        current_walking_param_.hip_pitch_offset = 14.5 * M_PI / 180;
        current_walking_param_.z_move_amplitude = 0.085;
        return true;
      }
      else if (fb_move >= 0.04)
      {
        current_walking_param_.hip_pitch_offset = 12.5 * M_PI / 180;
        current_walking_param_.z_move_amplitude = 0.085;
        return true;
      }
      else if (fb_move < 0.03)
      {
        current_walking_param_.hip_pitch_offset = 18.5 * M_PI / 180;
        current_walking_param_.z_move_amplitude = 0.065;
        return false;
      }
    }

    // ROS_INFO("FFFFFFFFFFFFFFFFFFFFFFFFFFFFBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB : %6.4f", fb_move);
    //  send message
    //  send message
    //  ROS_INFO("MASUKKK PROCESS FOLLOWING");
    //  for debug
    ROS_INFO("distance to ball : %6.4f, fb : %6.4f, delta : %6.6f", distance_to_ball, fb_move, delta_time);
    // ROS_INFO("In range [%d | %f]", count_to_kick_, ball_x_angle);
    return false;
  }

  bool MoveForwartRotateBallFollower::processwaitFollowing(double x_angle, double y_angle, double ball_size)
  {
    ros::Time curr_time = ros::Time::now();
    ros::Duration dur = curr_time - prev_time_;
    double delta_time = dur.nsec * 0.000000001 + dur.sec;
    prev_time_ = curr_time;

    count_not_found_ = 0;
    //  int ball_position_sum = 0;

    // check of getting head joints angle
    if (current_tilt_ == -10 && current_pan_ == -10)
    {
      ROS_ERROR("Failed to get current angle of head joints.");
      setWalkingCommand("stop");

      on_tracking_ = false;
      approach_ball_position_ = NotFound;
      return false;
    }

    ROS_INFO_COND(DEBUG_PRINT, "   ============== Head | Ball ==============   ");
    ROS_INFO_STREAM_COND(DEBUG_PRINT,
                         "== Head Pan : " << (current_pan_ * 180 / M_PI) << " | Ball X : " << (x_angle * 180 / M_PI));
    ROS_INFO_STREAM_COND(DEBUG_PRINT,
                         "== Head Tilt : " << (current_tilt_ * 180 / M_PI) << " | Ball Y : " << (y_angle * 180 / M_PI));

    approach_ball_position_ = OutOfRange;

    double distance_to_ball = CAMERA_HEIGHT * tan(M_PI * 0.5 + current_tilt_ - hip_pitch_offset_ - ball_size);

    double ball_y_angle = (current_tilt_ + y_angle) * 180 / M_PI;
    double ball_x_angle = (current_pan_ + x_angle) * 180 / M_PI;

    if (distance_to_ball < 0)
      distance_to_ball *= (-1);

    // double distance_to_kick = 0.25;
    double distance_to_kick = 0.22;

    // check whether ball is correct position.
    if ((distance_to_ball < distance_to_kick) && (fabs(ball_x_angle) < 25.0))
    {
      count_to_kick_ += 1;

      ROS_INFO_STREAM_COND(DEBUG_PRINT,
                           "head pan : " << (current_pan_ * 180 / M_PI) << " | ball pan : " << (x_angle * 180 / M_PI));
      ROS_INFO_STREAM_COND(DEBUG_PRINT,
                           "head tilt : " << (current_tilt_ * 180 / M_PI) << " | ball tilt : " << (y_angle * 180 / M_PI));
      ROS_INFO_STREAM_COND(DEBUG_PRINT, "foot to kick : " << ball_x_angle);

      ROS_INFO("In range [%d | %d]", count_to_kick_, ball_x_angle);

      // ball queue
      //    if(ball_position_queue_.size() >= 5)
      //      ball_position_queue_.erase(ball_position_queue_.begin());

      //    ball_position_queue_.push_back((ball_x_angle > 0) ? 1 : -1);

      if (count_to_kick_ > 20)
      {
        setWalkingCommand("stop");
        on_tracking_ = false;

        // check direction of the ball
        //      accum_ball_position_ = std::accumulate(ball_position_queue_.begin(), ball_position_queue_.end(), 0);

        //      if (accum_ball_position_ > 0)
        if (ball_x_angle > 0)
        {
          ROS_INFO_COND(DEBUG_PRINT, "Ready to kick : left"); // left
          approach_ball_position_ = OnLeft;
        }
        else
        {
          ROS_INFO_COND(DEBUG_PRINT, "Ready to kick : right"); // right
          approach_ball_position_ = OnRight;
        }

        return true;
      }
      else if (count_to_kick_ > 15)
      {
        //      if (ball_x_angle > 0)
        //        accum_ball_position_ += 1;
        //      else
        //        accum_ball_position_ -= 1;

        // send message
        setWalkingParam(IN_PLACE_FB_STEP, 0, 0);

        return false;
      }
    }
    else
    {
      count_to_kick_ = 0;
      //    accum_ball_position_ = NotFound;
    }

    double fb_move = 0.0, rl_angle = 0.0;
    double distance_to_walk = distance_to_ball - distance_to_kick;
    ROS_INFO("MASUKKK PROCESS WAIT FOLLOWING");
    calcwaitFootstep(distance_to_walk, current_pan_, delta_time, fb_move, rl_angle);
    ROS_INFO("MASUKKK PROCESS WAIT FOLLOWING");
    // send message
    setWalkingParam(fb_move, 0, rl_angle);
    ROS_INFO("MASUKKK PROCESS WAIT FOLLOWING");
    // for debug
    // ROS_INFO("distance to ball : %6.4f, fb : %6.4f, delta : %6.6f", distance_to_ball, fb_move, delta_time);
    // ROS_INFO("In range [%d | %d]", count_to_kick_, ball_x_angle);
    return false;
  }

  void MoveForwartRotateBallFollower::decideBallPositin(double x_angle, double y_angle)
  {
    // check of getting head joints angle
    if (current_tilt_ == -10 && current_pan_ == -10)
    {
      approach_ball_position_ = NotFound;
      return;
    }

    double ball_x_angle = current_pan_ + x_angle;

    if (ball_x_angle > 0)
      approach_ball_position_ = OnLeft;
    else
      approach_ball_position_ = OnRight;
  }

  void MoveForwartRotateBallFollower::firstKick()
  {

    first_kick_ += 1;

    if (first_kick_ > 0)
    {
      std_msgs::Int32 reply; // balasan komunikasi
      reply.data = 41;
      reply_pub_.publish(reply);
    }
  }

  void MoveForwartRotateBallFollower::sendZero()
  {
    std_msgs::Int32 reply; // balasan komunikasi
    reply.data = 0;
    reply_pub_.publish(reply);
  }

  void MoveForwartRotateBallFollower::waitFollowing()
  {
    count_not_found_++;

    if (count_not_found_ > NOT_FOUND_THRESHOLD * 0.5)
    {
      current_walking_param_.hip_pitch_offset = 18.5 * M_PI / 180;
      current_walking_param_.z_move_amplitude = 0.065;

      int tengah = 0;
      int limitKanan = tengah + 10; // 10
      int limitKiri = tengah - 10;  // 350
      int behind_ = tengah + 180;   // 180

      if (limitKiri < 0)
        limitKiri = limitKiri + 360;
      if (limitKanan > 360)
        limitKanan = limitKanan - 360;
      if (behind_ > 360)
        behind_ = behind_ - 360;

      if (arah < limitKiri && arah >= behind_)
      {
        ROS_INFO("Arah IMU kanann GOFRONTT: %d", arah);
        setWalkingParam(0.000, 0.0, -0.08, true);
      }
      else if (arah > limitKanan && arah < behind_)
      {
        ROS_INFO("Arah IMU kanann GOFRONTT: %d", arah);
        ROS_INFO("Arah IMU kanann GOFRONTT: %d", arah);
        setWalkingParam(0.00, 0.00, 0.08, true);
      }
      else if (arah >= (limitKiri + 9) || arah <= limitKanan)
      { // arah >= 359 atau arah <= 10
        ROS_INFO("Arah IMU kanann GOFRONTT: %d", arah);
        setWalkingParam(0.02, 0.00, 0.0, true);
      }

      ros::Duration(1.5).sleep();

      std_msgs::Int32 reply; // balasan komunikasi
      reply.data = 0;
      reply_pub_.publish(reply);
      // setWalkingParam(0.02, 0.0, 0.0);
    }
  }

  void MoveForwartRotateBallFollower::waitFollowingLurus()
  {
    count_not_found_++;

    if (count_not_found_ > NOT_FOUND_THRESHOLD * 0.5)
    {
      current_walking_param_.hip_pitch_offset = 18.5 * M_PI / 180;
      current_walking_param_.z_move_amplitude = 0.065;

      setWalkingParam(0.02, 0.0, 0.0, true);

      ros::Duration(1.5).sleep();

      std_msgs::Int32 reply;
      reply.data = 0;
      reply_pub_.publish(reply);
    }
  }

  void MoveForwartRotateBallFollower::waitFollowingMuter()
  {
    count_not_found_++;

    if (count_not_found_ > NOT_FOUND_THRESHOLD * 0.5)
    {
      current_walking_param_.hip_pitch_offset = 18.5 * M_PI / 180;
      current_walking_param_.z_move_amplitude = 0.065;

      setWalkingParam(0.0, 0.0, 0.09, true);

      ros::Duration(1.5).sleep();

      std_msgs::Int32 reply;
      reply.data = 0;
      reply_pub_.publish(reply);
    }
  }
  // void MoveForwartRotateBallFollower::waitprocessFollowing()
  // {
  // //  // count_not_found_++;
  // //   // std_msgs::Int32 reply; //balasan komunikasi
  // //   // reply.data = 0;
  // //   // reply_pub_.publish(reply);
  // //   double rl_angle = 0.0;
  // //   calcFootstep(0.0, current_pan_, 0.6, 0.0, rl_angle);
  // //   setWalkingParam(0.0, 0.0, rl_angle);
  // //   // if (count_not_found_ > NOT_FOUND_THRESHOLD * 0.5)
  // //   // {

  // //   // }
  //   count_not_found_++;

  //   if (count_not_found_ > NOT_FOUND_THRESHOLD * 0.5)
  //   {
  //     std_msgs::Int32 reply; //balasan komunikasi
  //     reply.data = 0;
  //     reply_pub_.publish(reply);
  //     double rl_angle = 0.0;
  //     calcFootstep(0.0, current_pan_, 0.6, 0.0, rl_angle);
  //     setWalkingParam(0.0, 0.0, 0.0);
  //   }
  // }

  void MoveForwartRotateBallFollower::setWalkingCommand(const std::string &command)
  {
    // get param
    if (command == "start")
    {
      getWalkingParam();
      setWalkingParam(IN_PLACE_FB_STEP, 0, 0, true);
      current_walking_param_.hip_pitch_offset = 18.5 * M_PI / 180;
      current_walking_param_.z_move_amplitude = 0.065;
      usleep(1500000);
    }

    std_msgs::String _command_msg;
    _command_msg.data = command;
    set_walking_command_pub_.publish(_command_msg);

    ROS_INFO_STREAM_COND(DEBUG_PRINT, "Send Walking command : " << command);
  }

  // void MoveForwartRotateBallFollower::imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg)  //bangun
  // {
  //   Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
  //   Eigen::MatrixXd rpy_orientation = robotis_framework::convertQuaternionToRPY(orientation);
  //   rpy_orientation *= (180 / M_PI);

  //   ROS_INFO_COND(DEBUG_PRINT, "Roll : %3.2f, Pitch : %2.2f", rpy_orientation.coeff(0, 0), rpy_orientation.coeff(1, 0));

  //   double pitch = rpy_orientation.coeff(1, 0);

  //   double alpha = 0.4;
  //   if (present_pitch_ == 0)
  //     present_pitch_ = pitch;
  //   else
  //     present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

  //   if (present_pitch_ > FALL_FORWARD_LIMIT && present_pitch_ < FALL_BACK_LIMIT)
  //       {
  //         setWalkingCommand("stop");
  //         on_tracking_ = false;
  //         approach_ball_position_ = NotFound;
  //         return false;
  //       }
  // }

  void MoveForwartRotateBallFollower::setWalkingParam(double x_move, double y_move, double rotation_angle, bool balance)
  {
    current_walking_param_.balance_enable = balance;
    current_walking_param_.x_move_amplitude = x_move + SPOT_FB_OFFSET;
    current_walking_param_.y_move_amplitude = y_move + SPOT_RL_OFFSET;
    current_walking_param_.angle_move_amplitude = rotation_angle + SPOT_ANGLE_OFFSET;

    set_walking_param_pub_.publish(current_walking_param_);

    current_x_move_ = x_move;
    current_r_angle_ = rotation_angle;
  }

  bool MoveForwartRotateBallFollower::getWalkingParam()
  {
    op3_walking_module_msgs::GetWalkingParam walking_param_msg;

    if (get_walking_param_client_.call(walking_param_msg))
    {
      current_walking_param_ = walking_param_msg.response.parameters;

      // update ui
      ROS_INFO_COND(DEBUG_PRINT, "Get walking parameters");

      return true;
    }
    else
    {
      ROS_ERROR("Fail to get walking parameters.");

      return false;
    }
  }

} // namespace robotis_op
