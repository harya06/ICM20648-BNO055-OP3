#include "bno055_imu/bno055_imu.h"

BNO055_IMU::BNO055_IMU(const std::string &port, uint32_t baudrate)
{
    try
    {
        serial_port_.setPort(port);
        serial_port_.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_port_.setTimeout(to);
        serial_port_.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Gagal membuka port serial: " << e.what());
        ros::shutdown();
    }

    if (serial_port_.isOpen())
        ROS_INFO("Port serial berhasil dibuka.");
    else
    {
        ROS_ERROR("Gagal membuka port serial.");
        ros::shutdown();
    }

    rpy_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("bno055/rpy", 10);
}

std::string BNO055_IMU::find_bno055_port()
{
    namespace fs = std::experimental::filesystem;
    std::string dir = "/dev/serial/by-id/";

    ROS_INFO("Scanning USB serial ports...");

    std::vector<std::string> ports_to_check;

    // Kumpulkan semua path yang punya 'USB'
    for (const auto &entry : fs::directory_iterator(dir))
    {
        std::string path = entry.path().string();
        if (path.find("USB") != std::string::npos || path.find("usb") != std::string::npos)
        {
            try
            {
                std::string real_path = fs::canonical(entry).string();
                ports_to_check.push_back(real_path);
            }
            catch (const std::exception &e)
            {
                ROS_WARN_STREAM("Gagal resolve path " << path << ": " << e.what());
                continue;
            }
        }
    }

    // Coba deteksi di semua port
    for (int attempt = 0; attempt < 3; ++attempt)
    {
        for (const auto &real_path : ports_to_check)
        {
            ROS_INFO_STREAM("Testing port: " << real_path);

            try
            {
                serial::Serial test_port(real_path, 115200, serial::Timeout::simpleTimeout(1000));
                if (test_port.isOpen())
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    test_port.flushInput();
                    test_port.flushOutput();
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));

                    for (int i = 0; i < 3; ++i)
                    {
                        std::string line = test_port.readline(1024, "\n");
                        ROS_INFO_STREAM("DEBUG: " << line);

                        if (line.find("Roll") != std::string::npos)
                        {
                            test_port.close();
                            ROS_INFO_STREAM("Detected BNO055 on port: " << real_path);
                            return real_path;
                        }
                    }
                }
            }
            catch (const std::exception &e)
            {
                ROS_WARN_STREAM("Exception on port " << real_path << ": " << e.what());
                continue;
            }
        }

        ROS_WARN("Retrying port detection...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    ROS_ERROR("BNO055 not detected on any port.");
    return "";
}

void BNO055_IMU::parseLine(const std::string &line, float &roll, float &pitch, float &yaw)
{
    sscanf(line.c_str(), "Roll = %f, Pitch = %f, Yaw = %f", &roll, &pitch, &yaw);
}

void BNO055_IMU::readAndPublish()
{
    if (serial_port_.available())
    {
        std::string line = serial_port_.readline(1024, "\n");
        float roll = 0, pitch = 0, yaw = 0;
        parseLine(line, roll, pitch, yaw);

        geometry_msgs::Vector3Stamped rpy_msg;
        rpy_msg.header.stamp = ros::Time::now();
        rpy_msg.header.frame_id = "imu_link";
        rpy_msg.vector.x = roll;
        rpy_msg.vector.y = pitch;
        rpy_msg.vector.z = yaw;

        rpy_pub_.publish(rpy_msg);

        ROS_INFO("Roll = %.2f, Pitch = %.2f, Yaw = %.2f", roll, pitch, yaw);
    }
}