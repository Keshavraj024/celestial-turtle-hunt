#ifndef __TURTLE_GUARDIAN_H__
#define __TURTLE_GURADIAN_H__

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/kill.hpp"
#include "thread"

class TurtleGuardian : public rclcpp::Node
{
public:
    TurtleGuardian();

private:
    void killTurtle(const std::string &turlteName);
    void spawnTurle();
    std::thread m_killThread;
    std::thread m_spawnThread;


};

#endif