#ifndef _TURTLE_GUARDIAN_NAVIGATOR_H_
#define _TURTLE_GUARDIAN_NAVIGATOR_H_

#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtleGuardianNavigator : public rclcpp::Node
{
public:
    TurtleGuardianNavigator();

private:
    void moveTurtle();
    void poseCallback(const turtlesim::msg::Pose::SharedPtr poseMsg);
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPublisher;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_turtlePoseSubscriber;
    geometry_msgs::msg::Twist m_vel;
    turtlesim::msg::Pose::SharedPtr m_pose;
    std::thread m_moveThread;
    double m_minX{0.5};
    double m_maxX{10.0};
    double m_linearVel{1.0};
};

#endif