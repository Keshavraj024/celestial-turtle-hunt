#ifndef _CELESTIAL_TURTLE_LIB__UTILS_HPP_
#define _CELESTIAL_TURTLE_LIB__UTILS_HPP_

#include <random>
#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/kill.hpp"

namespace celestial_turtle_lib
{
    template <typename T>
    T GenerateRandomNumber(T min, T max)
    {
        // Seed the random number generator
        std::random_device rd;
        std::mt19937 gen(rd());

        // Define the distribution for generating numbers between min and max
        std::uniform_real_distribution<T> distribution(min, max);

        // Generate a random number between min and max
        T randomNum = distribution(gen);

        return randomNum;
    }

    void offTrailLine(rclcpp::Node *node, const std::string &turtleName);
    void killTurtle(rclcpp::Node *node, const std::string &turlteName);

} /* namespace celestial_turtle_lib */

#endif /* CELESTIAL_TURTLE_LIB__UTILS_HPP_ */