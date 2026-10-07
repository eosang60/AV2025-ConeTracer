// ----- motor_control_node.cpp -----
#include <ros/ros.h>
#include <fs_msgs/ControlCommand.h>
#include <std_msgs/Bool.h>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <cmath>

#include "lidar_cone_detector/stop_cone_detect.h"

class MotorControl
{
public:
    MotorControl(ros::NodeHandle &nh) : nh_(nh)
    {
        // --- 파라미터 로드 ---
        nh_.param("/motor_ctr_node/AccelNorm", AccelNorm, false);
        nh_.param("/motor_ctr_node/SteeringNorm", SteeringNorm, false);

        if (AccelNorm)
            nh_.param("/motor_ctr_node/AccelNormValue", AccelNormValue, 1.0);
        if (SteeringNorm)
            nh_.param("/motor_ctr_node/SteeringNormValue", SteeringNormValue, 50.0);

        nh_.param("/motor_ctr_node/SubMotor", SubMotor, true);
        nh_.param("/motor_ctr_node/SubBrakeFlag", SubBrakeFlag, false);

        nh_.param<std::string>("/motor_ctr_node/BLDC_HOST", BLDC_HOST, "192.168.3.2");
        nh_.param<std::string>("/motor_ctr_node/AGNI_HOST", AGNI_HOST, "192.168.2.2");
        nh_.param("/motor_ctr_node/PORT", PORT, 7000);

        nh_.param("/motor_ctr_node/MinAccel", MinAccel, 3.3);
        nh_.param("/motor_ctr_node/MaxAccel", MaxAccel, 3.5);

        // --- 소켓 생성/연결 ---
        BLDC_SOCKET = createAndConnectSocket(BLDC_HOST.c_str(), PORT);
        AGNI_SOCKET = createAndConnectSocket(AGNI_HOST.c_str(), PORT);

        // --- 토픽 구독 ---
        if (SubMotor)
        {
            nh_.param<std::string>("/motor_ctr_node/SubMotorTopic", SubMotorTopic, "/fsds/control_command");
            sub_motor_ = nh_.subscribe(SubMotorTopic, 1, &MotorControl::SendMotorCmd, this);
        }
        if (SubBrakeFlag)
        {
            nh_.param<std::string>("/motor_ctr_node/SubBrakeFlagTopic", SubBrakeFlagTopic, "/brake_flag");
            sub_brake_flag_ = nh_.subscribe(SubBrakeFlagTopic, 1, &MotorControl::SendBrakeFlag, this);
        }

        ros::NodeHandle nh_global;
        estop_srv_ = nh_global.advertiseService("/e_stop",
                                                &MotorControl::handleEStop, this);
        ROS_INFO("E-STOP service ready at /e_stop");
    }

    int createAndConnectSocket(const char *host, int port)
    {
        int sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0)
        {
            ROS_ERROR("Cannot create socket");
            throw std::runtime_error("Socket creation failed");
        }
        sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        inet_pton(AF_INET, host, &server_addr.sin_addr);
        if (connect(sock, (sockaddr *)&server_addr, sizeof(server_addr)) < 0)
        {
            ROS_ERROR("Cannot connect to %s:%d", host, port);
            close(sock);
            throw std::runtime_error("Socket connection failed");
        }
        return sock;
    }

    void SendMotorCmd(const fs_msgs::ControlCommand::ConstPtr &msg)
    {
        ROS_INFO("----");
        double accel = MinAccel + (MaxAccel - MinAccel) * msg->throttle;
        if (AccelNorm)
            accel *= AccelNormValue;

        double steering = msg->steering * 180.0;
        if (SteeringNorm)
            steering = (steering / 180.0) * SteeringNormValue;

        char buf[64];
        snprintf(buf, sizeof(buf), "%.2f %.2f", accel, steering);

        sendAll(BLDC_SOCKET, buf);
        sendAll(AGNI_SOCKET, buf);
        ROS_INFO_STREAM("SEND MSG: " << buf);
    }

    void
    SendBrakeFlag(const std_msgs::Bool::ConstPtr &msg)
    {
        std::string data;
        if (msg->data)
            data = "E-STOP";
        else
            data = std::to_string(msg->data);
        sendAll(AGNI_SOCKET, data.c_str());
        sendAll(BLDC_SOCKET, data.c_str());
    }

    bool handleEStop(lidar_cone_detector::stop_cone_detect::Request &req,
                     lidar_cone_detector::stop_cone_detect::Response &res)
    {
        ROS_INFO("e-stop-handler");
        try
        {
            if (req.detected)
            {
                sendAll(BLDC_SOCKET, "E-STOP");
                sendAll(AGNI_SOCKET, "E-STOP");
                res.success = true;
                ROS_WARN("[MotorControl] E-STOP activated");
            }
            else
            {
                sendAll(BLDC_SOCKET, "DRIVING");
                sendAll(AGNI_SOCKET, "DRIVING");
                res.success = true;
                ROS_INFO("[MotorControl] E-STOP released");
            }
        }
        catch (const std::exception &e)
        {
            res.success = false;
            ROS_ERROR("[MotorControl] E-STOP handler error: %s", e.what());
        }
        return true;
    }

    void sendAll(int sock, const char *data)
    {
        size_t total_sent = 0;
        size_t len = strlen(data);
        while (total_sent < len)
        {
            ssize_t sent = send(sock, data + total_sent, len - total_sent, 0);
            if (sent < 0)
            {
                ROS_WARN("Socket send error");
                break;
            }
            total_sent += sent;
        }
        send(sock, "\n", 1, 0);
    }

    ~MotorControl()
    {
        close(BLDC_SOCKET);
        close(AGNI_SOCKET);
    }

private:
    ros::NodeHandle nh_;
    bool AccelNorm, SteeringNorm, SubMotor, SubBrakeFlag;
    double AccelNormValue = 1.0, SteeringNormValue = 50.0;
    std::string BLDC_HOST, AGNI_HOST, SubMotorTopic, SubBrakeFlagTopic;
    int PORT;
    double MinAccel = 3.3, MaxAccel = 3.5;
    int BLDC_SOCKET, AGNI_SOCKET;

    // ★ 추가: 서비스 서버 & 상태
    ros::ServiceServer estop_srv_;

    ros::Subscriber sub_motor_;
    ros::Subscriber sub_brake_flag_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_ctr_node");
    ros::NodeHandle nh("~");
    try
    {
        MotorControl mc(nh);
        ROS_INFO("---");
        ros::spin();
    }
    catch (const std::exception &ex)
    {
        ROS_ERROR("MotorControl exception: %s", ex.what());
    }
    return 0;
}
