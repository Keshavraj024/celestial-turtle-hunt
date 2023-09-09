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
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPublisher;    /**< Publishes the command veloctiy */
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_turtlePoseSubscriber; /**< Subscribes to the master turtle's pose*/
    void moveTurtle();
    void poseCallback(const turtlesim::msg::Pose::SharedPtr poseMsg);
    geometry_msgs::msg::Twist m_vel;
    turtlesim::msg::Pose::SharedPtr m_pose;
    // turtlesim::msg::Pose m_pose;
    std::thread m_moveThread;
    
};

#endif