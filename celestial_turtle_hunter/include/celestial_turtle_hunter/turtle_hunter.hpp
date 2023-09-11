#ifndef _CELESTIAL_TURTLE_HUNTER_H_
#define _CELESTIAL_TURTLE_HUNTER_H_

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "celestial_turtle_lib/utils.hpp"

class TurtleHunter : public rclcpp::Node
{
public:
    TurtleHunter();

private:
    void spawnTurle();
    std::thread m_killThread;
    std::thread m_spawnThread;
    double m_spawnPositionX;
    double m_spawnPositionY;
    double m_spawnOrientation;
};

#endif