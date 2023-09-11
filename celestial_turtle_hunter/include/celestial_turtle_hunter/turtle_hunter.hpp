#ifndef _CELESTIAL_TURTLE_HUNTER_H_
#define _CELESTIAL_TURTLE_HUNTER_H_

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "std_msgs/msg/bool.hpp"
#include "celestial_turtle_lib/utils.hpp"

class TurtleHunter : public rclcpp::Node
{
public:
    TurtleHunter();

private:
    void spawnTurle();
    void shutdownRequestCallback(const std_msgs::msg::Bool::SharedPtr shouldTerminate);
    std::thread m_killThread;
    std::thread m_spawnThread;
    double m_spawnPositionX;
    double m_spawnPositionY;
    double m_spawnOrientation;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_shutdownRequestSubscriber;
};

#endif