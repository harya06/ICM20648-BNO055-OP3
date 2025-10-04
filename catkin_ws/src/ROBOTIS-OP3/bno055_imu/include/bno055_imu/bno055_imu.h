#ifndef BNO055_IMU_H_
#define BNO055_IMU_H_

// C++ standard
#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <regex>
#include <thread>
#include <chrono>
#include <cstdlib>

#include <serial/serial.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

class BNO055_IMU
{
public:
    BNO055_IMU(const std::string &port, uint32_t baudrate);
    void readAndPublish();
    static std::string find_bno055_port();

private:
    void parseLine(const std::string &line, float &roll, float &pitch, float &yaw);

    serial::Serial serial_port_;
    ros::NodeHandle nh_;
    ros::Publisher rpy_pub_;
};

#endif // BNO055_IMU_H_
