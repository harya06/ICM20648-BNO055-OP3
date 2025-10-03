#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include <signal.h>

#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>
#include <time.h> // log
#include <pthread.h>
#include <termios.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <errno.h>
#include <fcntl.h>
#include <sstream>
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <unistd.h>
#include <arpa/inet.h>
#include <thread>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <thread>

int revenger;
int lastReceivedNumber = 0;
int previousRevenger = 0;
bool shouldSend = false; 
// std::mutex revengerMutex;


void replyCallback(const std_msgs::Int32::ConstPtr& msg)
{
    ROS_INFO("Balasan Komunikasi: %d", msg->data);
    revenger = msg->data;
    // std::lock_guard<std::mutex> lock(revengerMutex);
    if (revenger != previousRevenger)
    {
        previousRevenger = revenger;
        shouldSend = false;
    }
}

void receiveInt(int clientSocket, ros::Publisher& receive_pub)
{
    while (ros::ok())
    {
        int receivedInt;
        int bytesRead = recv(clientSocket, &receivedInt, sizeof(receivedInt), 0);
        if (bytesRead <= 0)
        {
            std::cerr << "Server terputus.\n";
            break;
        }

        lastReceivedNumber = receivedInt;

        std_msgs::Int32 receive;
        receive.data = lastReceivedNumber;
        receive_pub.publish(receive);
        std::cout << "Diterima: " << receivedInt << std::endl;
    }
}

void sendInt(int clientSocket)
{ 
    while (ros::ok())
    {
        // std::lock_guard<std::mutex> lock(revengerMutex);
        ros::spinOnce();
        if (shouldSend == false)
        {
            int value = revenger;
            // std::cout << "Mengirim ke server: " << value << std::endl;
            send(clientSocket, &value, sizeof(value), 0);
            shouldSend = true;   
        }



        usleep(100000);  // Menghindari busy-waiting
    }
}

void sigintHandler(int sig)
{
    ROS_INFO("Ctrl+C ditekan. Keluar...");
    ros::shutdown();  // Memberhentikan ROS
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_cpp_pubsub");
    ros::NodeHandle nh;
    ros::Publisher receive_pub = nh.advertise<std_msgs::Int32>("/receive", 10);
    ros::Subscriber reply_sub = nh.subscribe("/reply", 10, replyCallback);

    int clientSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (clientSocket == -1)
    {
        std::cerr << "Error membuat socket\n";
        return -1;
    }

    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(5555);
    inet_pton(AF_INET, "192.168.0.95", &serverAddr.sin_addr);

    if (connect(clientSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == -1)
    {
        std::cerr << "Error menghubungkan ke server\n";
        close(clientSocket);
        return -1;
    }

    signal(SIGINT, sigintHandler);
// Thread untuk mengirim data ke server
    std::thread sendThread(sendInt, clientSocket);

    // Thread untuk menerima data dari server
    std::thread receiveThread(receiveInt, clientSocket, std::ref(receive_pub));


    // Loop utama untuk mengubah nilai yang akan dikirim
    while (ros::ok())
    {
        // replyCallback();
        // int value = revenger;
        // if (value != lastReceivedNumber)
        // {
        //     shouldSend = false;
        // }
        // // if (revenger != lastReceivedNumber)
        // // {
        // //     shouldSend = false;
        // // }
        // ros::spinOnce();
    }

    // Bergabung dengan thread penerima dan pengirim
    receiveThread.join();
    // sendThread.join();

    // Menutup socket klien
    close(clientSocket);

    return 0;
}


