#include "bno055_imu/bno055_imu.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bno055_serial_node");

    std::string port = "/dev/bno055";
    if (!fs::exists(port))
    {
        ROS_WARN("Symlink /dev/bno055 tidak ditemukan, fallback ke pencarian otomatis...");
        port = BNO055_IMU::find_bno055_port();
    }

    if (port.empty())
    {
        ROS_ERROR("BNO055 tidak terdeteksi di port manapun.");
        return -1;
    }

    try
    {
        BNO055_IMU handler(port, 115200);

        ros::Rate loop_rate(100); // 100 Hz
        while (ros::ok())
        {
            handler.readAndPublish();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    catch (const std::exception &e)
    {
        ROS_FATAL_STREAM("Gagal inisialisasi port serial: " << e.what());
        return 1;
    }

    return 0;
}
