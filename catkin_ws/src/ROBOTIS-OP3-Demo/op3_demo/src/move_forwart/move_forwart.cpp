/*******************************************************************************
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

#include "op3_demo/move_forwart.h"

namespace robotis_op
{

  MoveForwart::MoveForwart()
      : FALL_FORWARD_LIMIT(60),
        FALL_BACK_LIMIT(-60),
        SPIN_RATE(30),
        DEBUG_PRINT(false),
        wait_count_(0),
        on_following_ball_(false),
        on_tracking_ball_(false),
        restart_soccer_(false),
        start_following_(false),
        stop_following_(false),
        stop_fallen_check_(false),
        robot_status_(Waited),
        stand_state_(Stand),
        tracking_status_(MoveForwartBallTracker::Waiting),
        present_pitch_(0),
        comp_(0),
        komunikasi_moveforwart(0),         // komunikasi
        desired_komunikasi_moveforwart(0), // komunikasi
        log_initialized_(false),
        log_duration_sec_(61.0), // lama waktu log open cr
        log_initialized_bno_(false),
        log_duration_sec_bno_(61.0), // lama waktu log bno
        start_time_bno_initialized_(false),
        log_initialized_gyro_(false),
        log_duration_sec_gyro_(61.0), // lama waktu log gyro
        start_time_gyro_initialized_(false)
  {
    // COMPASS

    // init ros
    enable_ = false;

    ros::NodeHandle nh(ros::this_node::getName());

    std::string default_path = ros::package::getPath("op3_gui_demo") + "/config/gui_config.yaml";
    std::string path = nh.param<std::string>("demo_config", default_path);
    parseJointNameFromYaml(path);

    boost::thread queue_thread = boost::thread(boost::bind(&MoveForwart::callbackThread, this));
    boost::thread process_thread = boost::thread(boost::bind(&MoveForwart::processThread, this));
    boost::thread tracking_thread = boost::thread(boost::bind(&MoveForwart::trackingThread, this));

    is_grass_ = nh.param<bool>("grass_demo", false);
  }

  MoveForwart::~MoveForwart()
  {
  }

  void MoveForwart::setDemoEnable()
  {
    enable_ = true;
    startSoccerMode();
  }

  void MoveForwart::setDemoDisable()
  {
    // handle disable procedure
    move_forwart_ball_tracker_.stopTracking();
    move_forwart_ball_follower_.stopFollowing();

    enable_ = false;
    wait_count_ = 0;
    on_following_ball_ = false;
    on_tracking_ball_ = false;
    restart_soccer_ = false;
    start_following_ = false;
    stop_following_ = false;
    stop_fallen_check_ = false;
    start_ready_ = (false);
    strategi_satu_ = (false);

    tracking_status_ = MoveForwartBallTracker::Waiting;
  }

  void MoveForwart::process()
  {
    if (enable_ == false)
      return;

    // check to start
    if (start_following_ == true)
    {
      move_forwart_ball_tracker_.startTracking();
      move_forwart_ball_follower_.startFollowing();
      start_following_ = false;

      wait_count_ = 0 * SPIN_RATE;
    }

    // check to stop
    if (stop_following_ == true)
    {
      move_forwart_ball_tracker_.stopTracking();
      move_forwart_ball_follower_.stopFollowing();
      stop_following_ = false;

      wait_count_ = 0;
    }

    if (wait_count_ <= 0)
    {
      // ball following

      if (on_following_ball_ == true && (komunikasi_moveforwart == 0 || komunikasi_moveforwart == 4 || komunikasi_moveforwart == 10))
      {
        switch (tracking_status_)
        {
        case MoveForwartBallTracker::Found:
          move_forwart_ball_follower_.processFollowing(move_forwart_ball_tracker_.getPanOfBall(), move_forwart_ball_tracker_.getTiltOfBall(), 0.0);
          break;

        case MoveForwartBallTracker::NotFound:
          move_forwart_ball_follower_.waitFollowing();
          break;

        default:
          break;
        }
      }

      else if (on_following_ball_ == true && (komunikasi_moveforwart == 42))
      {
        switch (tracking_status_)
        {
        case MoveForwartBallTracker::Found:
          move_forwart_ball_follower_.waitFollowingLurus();
          break;

        case MoveForwartBallTracker::NotFound:
          move_forwart_ball_follower_.waitFollowingLurus();
          break;

        default:
          break;
        }
      }

      else if (on_following_ball_ == true && (komunikasi_moveforwart == 43))
      {
        switch (tracking_status_)
        {
        case MoveForwartBallTracker::Found:
          move_forwart_ball_follower_.waitFollowingMuter();
          break;

        case MoveForwartBallTracker::NotFound:
          move_forwart_ball_follower_.waitFollowingMuter();
          break;

        default:
          break;
        }
      }

      // check fallen states
      switch (stand_state_)
      {
      case Stand:
      {
        // check restart soccer
        if (restart_soccer_ == true)
        {
          restart_soccer_ = false;
          move_forwart_ball_follower_.firstKick();
          startSoccerMode();
          break;
        }

        // check states for kick
        //      int ball_position = move_forwart_ball_follower_.getBallPosition();
        bool in_range = move_forwart_ball_follower_.isBallInRange();

        if (in_range == true)
        {
          move_forwart_ball_follower_.stopFollowing();
          handleKick();
        }
        break;
      }
        // fallen state : Fallen_Forward, Fallen_Behind
      default:
      {
        move_forwart_ball_follower_.stopFollowing();
        handleFallen(stand_state_);
        break;
      }
      }
    }

    else
    {
      wait_count_ -= 1;
    }
  }

  void MoveForwart::processThread()
  {
    bool result = false;

    // set node loop rate
    ros::Rate loop_rate(SPIN_RATE);

    move_forwart_ball_tracker_.startTracking();

    // node loop
    while (ros::ok())
    {
      if (enable_ == true)
      {
        if ((komunikasi_moveforwart == 0 || komunikasi_moveforwart == 4 || komunikasi_moveforwart == 10 || komunikasi_moveforwart == 42 || komunikasi_moveforwart == 43) && desired_komunikasi_moveforwart != komunikasi_moveforwart)
        {
          startSoccerMode();
          desired_komunikasi_moveforwart = komunikasi_moveforwart;
        }
        else if (komunikasi_moveforwart != 0 && komunikasi_moveforwart != 4 && komunikasi_moveforwart != 10 && komunikasi_moveforwart != 42 && komunikasi_moveforwart != 43)
        {
          on_following_ball_ = false;
          desired_komunikasi_moveforwart = 5;
          switch (tracking_status_)
          {
          case MoveForwartBallTracker::Found:
            move_forwart_ball_follower_.processwaitFollowing(move_forwart_ball_tracker_.getPanOfBall(), move_forwart_ball_tracker_.getTiltOfBall(), 0.0);
            break;

          case MoveForwartBallTracker::NotFound:
            move_forwart_ball_follower_.waitFollowing();
            break;

          default:
            break;
          }
        }
        process();
      }
      // relax to fit output rate
      loop_rate.sleep();
    }
  }

  void MoveForwart::callbackThread()
  {
    ros::NodeHandle nh(ros::this_node::getName());

    // subscriber & publisher
    reply_pub_ = nh.advertise<std_msgs::Int32>("/reply", 10); // balasan komunikasi
    module_control_pub_ = nh.advertise<robotis_controller_msgs::JointCtrlModule>("/robotis/set_joint_ctrl_modules", 0);
    motion_index_pub_ = nh.advertise<std_msgs::Int32>("/robotis/action/page_num", 0);
    rgb_led_pub_ = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);

    buttuon_sub_ = nh.subscribe("/robotis/open_cr/button", 1, &MoveForwart::buttonHandlerCallback, this);
    demo_command_sub_ = nh.subscribe("/robotis/demo_command", 1, &MoveForwart::demoCommandCallback, this);
    imu_data_sub_ = nh.subscribe("/robotis/open_cr/imu", 1, &MoveForwart::imuDataCallback, this);
    ypr_data_sub_ = nh.subscribe("sensor_ypr", 1, &MoveForwart::sensorYPRCallback, this);
    komunikasimoveforwart_sub_ = nh.subscribe("/receive", 10, &MoveForwart::komunikasimoveforwartCallback, this); // komunikasi
    // rpy_sub_ = nh.subscribe("/bno055/rpy", 10, &MoveForwart::rpyCallback, this);                                  // bno
    rpy_bno_sub_ = nh.subscribe("/robotis/rpy_bno", 10, &MoveForwart::rpyCallback, this);   // bno
    gyro_sub_ = nh.subscribe("/robotis/raw_gyro", 10, &MoveForwart::rawGyroCallback, this); // bno

    is_running_client_ = nh.serviceClient<op3_action_module_msgs::IsRunning>("/robotis/action/is_running");
    set_joint_module_client_ = nh.serviceClient<robotis_controller_msgs::SetJointModule>("/robotis/set_present_joint_ctrl_modules");

    test_pub_ = nh.advertise<std_msgs::String>("/debug_text", 0);

    while (nh.ok())
    {
      ros::spinOnce();

      usleep(1000);
    }
  }

  void MoveForwart::sensorYPRCallback(const robotis_controller_msgs::SensorYPR::ConstPtr &msg)
  {
    ROS_INFO("Roll : %3.2f, Pitch : %2.2f", msg->yaw, msg->pitch);
  }

  void MoveForwart::trackingThread()
  {

    // set node loop rate
    ros::Rate loop_rate(SPIN_RATE);

    move_forwart_ball_tracker_.startTracking();

    // node loop
    while (ros::ok())
    {
      if (enable_ == true && on_tracking_ball_ == true)
      {
        // ball tracking
        int tracking_status;

        tracking_status = move_forwart_ball_tracker_.processTracking();

        // set led
        switch (tracking_status)
        {
        case MoveForwartBallTracker::Found:
          if (tracking_status_ != tracking_status)
            setRGBLED(0x1F, 0x1F, 0x1F);
          break;

        case MoveForwartBallTracker::NotFound:
          if (tracking_status_ != tracking_status)
            setRGBLED(0, 0, 0);
          break;

        default:
          break;
        }

        if (tracking_status != tracking_status_)
          tracking_status_ = tracking_status;
      }
      // relax to fit output rate
      loop_rate.sleep();
    }
  }

  void MoveForwart::setBodyModuleToDemo(const std::string &body_module, bool with_head_control)
  {
    robotis_controller_msgs::JointCtrlModule control_msg;

    std::string head_module = "head_control_module";
    std::map<int, std::string>::iterator joint_iter;

    for (joint_iter = id_joint_table_.begin(); joint_iter != id_joint_table_.end(); ++joint_iter)
    {
      // check whether joint name contains "head"
      if (joint_iter->second.find("head") != std::string::npos)
      {
        if (with_head_control == true)
        {
          control_msg.joint_name.push_back(joint_iter->second);
          control_msg.module_name.push_back(head_module);
        }
        else
          continue;
      }
      else
      {
        control_msg.joint_name.push_back(joint_iter->second);
        control_msg.module_name.push_back(body_module);
      }
    }

    // no control
    if (control_msg.joint_name.size() == 0)
      return;

    callServiceSettingModule(control_msg);
    // std::cout << "enable module of body : " << body_module << std::endl;
  }

  void MoveForwart::setModuleToDemo(const std::string &module_name)
  {
    if (enable_ == false)
      return;

    robotis_controller_msgs::JointCtrlModule control_msg;
    std::map<int, std::string>::iterator joint_iter;

    for (joint_iter = id_joint_table_.begin(); joint_iter != id_joint_table_.end(); ++joint_iter)
    {
      control_msg.joint_name.push_back(joint_iter->second);
      control_msg.module_name.push_back(module_name);
    }

    // no control
    if (control_msg.joint_name.size() == 0)
      return;

    callServiceSettingModule(control_msg);
    // std::cout << "enable module : " << module_name << std::endl;
  }

  void MoveForwart::callServiceSettingModule(const robotis_controller_msgs::JointCtrlModule &modules)
  {
    robotis_controller_msgs::SetJointModule set_joint_srv;
    set_joint_srv.request.joint_name = modules.joint_name;
    set_joint_srv.request.module_name = modules.module_name;

    if (set_joint_module_client_.call(set_joint_srv) == false)
    {
      ROS_ERROR("Failed to set module");
      return;
    }

    return;
  }

  void MoveForwart::parseJointNameFromYaml(const std::string &path)
  {
    YAML::Node doc;
    try
    {
      // load yaml
      doc = YAML::LoadFile(path.c_str());
    }
    catch (const std::exception &e)
    {
      ROS_ERROR("Fail to load id_joint table yaml.");
      return;
    }

    // parse id_joint table
    YAML::Node _id_sub_node = doc["id_joint"];
    for (YAML::iterator _it = _id_sub_node.begin(); _it != _id_sub_node.end(); ++_it)
    {
      int _id;
      std::string _joint_name;

      _id = _it->first.as<int>();
      _joint_name = _it->second.as<std::string>();

      id_joint_table_[_id] = _joint_name;
      joint_id_table_[_joint_name] = _id;
    }
  }

  // joint id -> joint name
  bool MoveForwart::getJointNameFromID(const int &id, std::string &joint_name)
  {
    std::map<int, std::string>::iterator _iter;

    _iter = id_joint_table_.find(id);
    if (_iter == id_joint_table_.end())
      return false;

    joint_name = _iter->second;
    return true;
  }

  // joint name -> joint id
  bool MoveForwart::getIDFromJointName(const std::string &joint_name, int &id)
  {
    std::map<std::string, int>::iterator _iter;

    _iter = joint_id_table_.find(joint_name);
    if (_iter == joint_id_table_.end())
      return false;

    id = _iter->second;
    return true;
  }

  int MoveForwart::getJointCount()
  {
    return joint_id_table_.size();
  }

  bool MoveForwart::isHeadJoint(const int &id)
  {
    std::map<std::string, int>::iterator _iter;

    for (_iter = joint_id_table_.begin(); _iter != joint_id_table_.end(); ++_iter)
    {
      if (_iter->first.find("head") != std::string::npos)
        return true;
    }

    return false;
  }

  void MoveForwart::komunikasimoveforwartCallback(const std_msgs::Int32::ConstPtr &msg) // komunikasi
  {
    ROS_INFO("Balasan Komunikasi Soccerr: %d", msg->data);
    komunikasi_moveforwart = msg->data;
  }

  void MoveForwart::buttonHandlerCallback(const std_msgs::String::ConstPtr &msg)
  {
    move_forwart_ball_follower_.sendZero();

    if (enable_ == false)
      return;

    if (msg->data == "start")
    {
      if (on_following_ball_ == true)
        stopSoccerMode();
      else
        startSoccerMode();
    }
  }

  void MoveForwart::demoCommandCallback(const std_msgs::String::ConstPtr &msg)
  {
    if (enable_ == false)
      return;

    if (msg->data == "start")
    {
      if (on_following_ball_ == true)
        stopSoccerMode();
      else
        startSoccerMode();
    }
    else if (msg->data == "stop")
    {
      stopSoccerMode();
    }
  }

  std::string MoveForwart::waktu()
  {
    time_t now = time(NULL);
    tm *localTime = localtime(&now);

    const char *hari[] = {
        "Minggu", "Senin", "Selasa", "Rabu",
        "Kamis", "Jumat", "Sabtu"};

    char buffer[64];
    snprintf(buffer, sizeof(buffer), "[%s_%02d-%02d-%04d_%02d-%02d-%02d]",
             hari[localTime->tm_wday],
             localTime->tm_mday,
             localTime->tm_mon + 1,     // bulan dimulai dari 0
             localTime->tm_year + 1900, // tahun dimulai dari 1900
             localTime->tm_hour,
             localTime->tm_min,
             localTime->tm_sec);

    return std::string(buffer);
  }

  void MoveForwart::rawGyroCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
  {
    if (!start_time_gyro_initialized_)
    {
      return;
    }

    // Simpan data dari pesan ke variabel member class
    double raw_gyro_x_ = msg->vector.x;
    double raw_gyro_y_ = msg->vector.y;
    double raw_gyro_z_ = msg->vector.z;

    system("mkdir -p /home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_demo/src/LOG_ROS/imu_balance_lurus");

    if (!log_initialized_gyro_)
    {
      last_log_time_gyro_ = last_log_time_;
      std::stringstream filename;
      filename << "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_demo/src/LOG_ROS/imu_balance_lurus/walking_log_lurus_" << waktu() << ".txt";
      imu_log_file_gyro_.open(filename.str().c_str());

      if (!imu_log_file_gyro_.is_open())
      {
        ROS_ERROR("Gagal membuka file log: %s", filename.str().c_str());
        return;
      }

      imu_log_file_gyro_ << "Time                        , raw_gyro_x_, raw_gyro_y_, raw_gyro_z_\n";
      last_log_time_gyro_ = start_time_gyro_;
      log_initialized_gyro_ = true;

      ROS_INFO("Logging dimulai. File: %s", filename.str().c_str());
    }

    ros::Time current_time = ros::Time::now();
    double time_now = (current_time - start_time_gyro_).toSec(); // Mengacu ke waktu imuDataCallback
    double time_since_last_log = (current_time - last_log_time_gyro_).toSec();

    if (log_initialized_gyro_)
    {
      if (time_since_last_log >= 0.5)
      {
        imu_log_file_gyro_ << waktu() << ", " << raw_gyro_x_ << ", " << raw_gyro_y_ << ", " << raw_gyro_z_ << "\n";
        imu_log_file_gyro_.flush();
        last_log_time_gyro_ = current_time;
      }

      if (time_now > log_duration_sec_gyro_)
      {
        imu_log_file_gyro_.close();
        log_initialized_gyro_ = false;
        ROS_INFO("Logging selesai. File disimpan.");
      }
    }
  }

  void MoveForwart::rpyCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg)
  {
    if (!start_time_bno_initialized_)
    {
      return;
    }

    // const double DEG_TO_RAD = M_PI / 180.0;

    // float roll_bno = msg->vector.x * DEG_TO_RAD;
    // float pitch_bno = msg->vector.y * DEG_TO_RAD;
    // float yaw_bno = msg->vector.z * DEG_TO_RAD;

    float roll_bno = msg->vector.x;
    float pitch_bno = msg->vector.y;
    float yaw_bno = msg->vector.z;

    ROS_INFO("Roll BNO = %.2f, Pitch BNO = %.2f, Yaw BNO = %.2f", roll_bno, pitch_bno, yaw_bno);

    system("mkdir -p /home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_demo/src/LOG_ROS/imu_bno_lurus/");

    if (!log_initialized_bno_)
    {
      last_log_time_bno_ = last_log_time_;
      std::stringstream filename;
      filename << "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_demo/src/LOG_ROS/imu_bno_lurus/log_lurus_" << waktu() << ".txt";
      imu_log_file_bno_.open(filename.str().c_str());

      if (!imu_log_file_bno_.is_open())
      {
        ROS_ERROR("Gagal membuka file log: %s", filename.str().c_str());
        return;
      }

      imu_log_file_bno_ << "Time                        , Roll, Pitch, Yaw\n";
      last_log_time_bno_ = start_time_bno_;
      log_initialized_bno_ = true;

      ROS_INFO("Logging dimulai. File: %s", filename.str().c_str());
    }

    ros::Time current_time = ros::Time::now();
    double time_now = (current_time - start_time_bno_).toSec(); // Mengacu ke waktu imuDataCallback
    double time_since_last_log = (current_time - last_log_time_bno_).toSec();

    if (log_initialized_bno_)
    {
      if (time_since_last_log >= 0.5)
      {
        imu_log_file_bno_ << waktu() << ", " << roll_bno << ", " << pitch_bno << ", " << yaw_bno << "\n";
        imu_log_file_bno_.flush();
        last_log_time_bno_ = current_time;
      }

      if (time_now > log_duration_sec_bno_)
      {
        imu_log_file_bno_.close();
        log_initialized_bno_ = false;
        ROS_INFO("Logging selesai. File disimpan.");
      }
    }
  }

  // check fallen states
  void MoveForwart::imuDataCallback(const sensor_msgs::Imu::ConstPtr &msg)
  {
    if (enable_ == false)
      return;

    if (stop_fallen_check_ == true)
      return;

    Eigen::Quaterniond orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::MatrixXd rpy_orientation = robotis_framework::convertQuaternionToRPY(orientation);
    rpy_orientation *= (180 / M_PI);

    start_time_bno_initialized_ = true;
    start_time_gyro_initialized_ = true;

    // ROS_INFO_COND(DEBUG_PRINT, "Roll : %3.2f, Pitch : %2.2f", rpy_orientation.coeff(0, 0), rpy_orientation.coeff(1, 0));

    double roll = rpy_orientation.coeff(0, 0);
    double pitch = rpy_orientation.coeff(1, 0);
    double yaw = rpy_orientation.coeff(2, 0);

    ROS_INFO("Roll : %3.2f, Pitch : %2.2f, Yaw : %1.2f", roll, pitch, yaw);

    system("mkdir -p /home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_demo/src/LOG_ROS/imu_open_cr_lurus/");

    // Inisialisasi logging
    if (!log_initialized_)
    {
      last_log_time_ = ros::Time::now();
      start_time_bno_ = last_log_time_;
      start_time_gyro_ = start_time_bno_;

      std::string package_path = ros::package::getPath("op3_demo");

      std::stringstream filename;
      filename << "/home/robotis/catkin_ws/src/ROBOTIS-OP3-Demo/op3_demo/src/LOG_ROS/imu_open_cr_lurus/log_lurus_" << waktu() << ".txt"; // unik tiap waktu dijalankan
      imu_log_file_.open(filename.str().c_str());

      if (!imu_log_file_.is_open())
      {
        ROS_ERROR("Gagal membuka file log: %s", filename.str().c_str());
        return;
      }

      imu_log_file_ << "Time                        , Roll, Pitch, Yaw\n";
      start_time_ = ros::Time::now();
      last_log_time_ = start_time_;
      log_initialized_ = true;
      ROS_INFO("Logging dimulai. File: %s", filename.str().c_str());
    }

    ros::Time current_time = ros::Time::now();
    double time_now = (current_time - start_time_).toSec();
    double time_since_last_log = (current_time - last_log_time_).toSec();

    // ROS_INFO("Time Now: %.2f | Since Last Log: %.2f", time_now, time_since_last_log);

    if (log_initialized_)
    {
      if (time_since_last_log >= 0.5) // interval
      {
        imu_log_file_ << waktu() << ", " << roll << ", " << pitch << ", " << yaw << "\n";
        imu_log_file_.flush();
        ROS_INFO("LOGGED: %.2f, %.2f, %.2f", roll, pitch, yaw);
        last_log_time_ = current_time;
      }

      if (time_now > log_duration_sec_)
      {
        imu_log_file_.close();
        log_initialized_ = false;
        start_time_bno_initialized_ = false;
        start_time_gyro_initialized_ = false;
        ROS_INFO("Logging selesai. File disimpan.");
      }
    }

    // double alpha = 0.4;
    // if (present_pitch_ == 0)
    //   present_pitch_ = pitch;
    // else
    //   present_pitch_ = present_pitch_ * (1 - alpha) + pitch * alpha;

    // if (present_pitch_ > FALL_FORWARD_LIMIT)
    //   stand_state_ = Fallen_Forward;
    // else if (present_pitch_ < FALL_BACK_LIMIT)
    //   stand_state_ = Fallen_Behind;
    // else
    //   stand_state_ = Stand;
  }

  void MoveForwart::startSoccerMode()
  {
    setBodyModuleToDemo("walking_module");
    on_following_ball_ = true;
    on_tracking_ball_ = false;
    start_following_ = true;
    ROS_INFO("Start Soccer Demo");
  }

  void MoveForwart::stopSoccerMode()
  {
    ROS_INFO("Stop Soccer Demo");
    on_following_ball_ = false;
    on_tracking_ball_ = false;
    stop_following_ = true;
  }

  void MoveForwart::handleKick(int ball_position)
  {
    usleep(100);

    // change to motion module
    setModuleToDemo("action_module");

    if (handleFallen(stand_state_) == true || enable_ == false)
      return;

    // kick motion
    switch (ball_position)
    {
    case robotis_op::MoveForwartBallFollower::OnRight:
      std::cout << "Kick Motion [R]: " << ball_position << std::endl;
      playMotion(is_grass_ ? RightKick + ForGrass : RightKick);
      break;

    case robotis_op::MoveForwartBallFollower::OnLeft:
      std::cout << "Kick Motion [L]: " << ball_position << std::endl;
      playMotion(is_grass_ ? LeftKick + ForGrass : LeftKick);
      break;

    default:
      break;
    }

    on_following_ball_ = false;
    restart_soccer_ = true;
    tracking_status_ = MoveForwartBallTracker::NotFound;
    move_forwart_ball_follower_.clearBallPosition();

    usleep(100); // 2detik

    if (handleFallen(stand_state_) == true)
      return;

    // ceremony
    // playMotion(Ceremony);
  }

  void MoveForwart::handleKick()
  {
    usleep(100);

    // change to motion module
    setModuleToDemo("action_module");

    if (handleFallen(stand_state_) == true || enable_ == false)
      return;

    // kick motion
    move_forwart_ball_follower_.decideBallPositin(move_forwart_ball_tracker_.getPanOfBall(), move_forwart_ball_tracker_.getTiltOfBall());
    int ball_position = move_forwart_ball_follower_.getBallPosition();
    if (ball_position == MoveForwartBallFollower::NotFound || ball_position == MoveForwartBallFollower::OutOfRange)
    {
      on_following_ball_ = false;
      restart_soccer_ = true;
      tracking_status_ = MoveForwartBallTracker::NotFound;
      move_forwart_ball_follower_.clearBallPosition();
      return;
    }

    switch (ball_position)
    {
    case robotis_op::MoveForwartBallFollower::OnRight:
      std::cout << "Kick Motion [R]: " << ball_position << std::endl;
      sendDebugTopic("Kick the ball using Right foot");
      playMotion(is_grass_ ? RightKick + ForGrass : RightKick);
      break;

    case robotis_op::MoveForwartBallFollower::OnLeft:
      std::cout << "Kick Motion [L]: " << ball_position << std::endl;
      sendDebugTopic("Kick the ball using Left foot");
      playMotion(is_grass_ ? LeftKick + ForGrass : LeftKick);
      break;

    default:
      break;
    }

    on_following_ball_ = false;
    restart_soccer_ = true;
    tracking_status_ = MoveForwartBallTracker::NotFound;
    move_forwart_ball_follower_.clearBallPosition();

    usleep(100); // 2detik

    if (handleFallen(stand_state_) == true)
      return;

    // ceremony
    // playMotion(Ceremony);
  }

  bool MoveForwart::handleFallen(int fallen_status)
  {
    if (fallen_status == Stand)
    {
      return false;
    }

    // change to motion module
    setModuleToDemo("action_module");

    // getup motion
    switch (fallen_status)
    {
    case Fallen_Forward:
      std::cout << "Getup Motion [F]: " << std::endl;
      playMotion(is_grass_ ? GetUpFront + ForGrass : GetUpFront);
      break;

    case Fallen_Behind:
      std::cout << "Getup Motion [B]: " << std::endl;
      playMotion(is_grass_ ? GetUpBack + ForGrass : GetUpBack);
      break;

    default:
      break;
    }

    while (isActionRunning() == true)
      usleep(100); // 100

    usleep(100);

    if (on_following_ball_ == true || komunikasi_moveforwart == 0 || komunikasi_moveforwart == 4 || komunikasi_moveforwart == 1 || komunikasi_moveforwart == 3 || komunikasi_moveforwart == 2)
      restart_soccer_ = true;

    // reset state
    on_following_ball_ = false;

    return true;
  }

  void MoveForwart::playMotion(int motion_index)
  {
    std_msgs::Int32 motion_msg;
    motion_msg.data = motion_index;

    motion_index_pub_.publish(motion_msg);
  }

  void MoveForwart::setRGBLED(int blue, int green, int red)
  {
    int led_full_unit = 0x1F;
    int led_value = (blue & led_full_unit) << 10 | (green & led_full_unit) << 5 | (red & led_full_unit);
    robotis_controller_msgs::SyncWriteItem syncwrite_msg;
    syncwrite_msg.item_name = "LED_RGB";
    syncwrite_msg.joint_name.push_back("open-cr");
    syncwrite_msg.value.push_back(led_value);

    rgb_led_pub_.publish(syncwrite_msg);
  }

  // check running of action
  bool MoveForwart::isActionRunning()
  {
    op3_action_module_msgs::IsRunning is_running_srv;

    if (is_running_client_.call(is_running_srv) == false)
    {
      ROS_ERROR("Failed to get action status");
      return true;
    }
    else
    {
      if (is_running_srv.response.is_running == true)
      {
        return true;
      }
    }

    return false;
  }

  void MoveForwart::sendDebugTopic(const std::string &msgs)
  {
    std_msgs::String debug_msg;
    debug_msg.data = msgs;

    test_pub_.publish(debug_msg);
  }
} // namespace robotis_op
