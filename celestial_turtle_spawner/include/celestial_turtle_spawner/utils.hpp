#ifndef CELESTIAL_TURTLE_SPAWNER__UTILS_HPP_
#define CELESTIAL_TURTLE_SPAWNER__UTILS_HPP_
#include <iostream>
#include <random>
#include "turtlesim/srv/set_pen.hpp"
#include "chrono"
#include "rclcpp/rclcpp.hpp"

namespace celestial_turtle_spawner
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

    void offTrailLine(const std::shared_ptr<rclcpp::Node> &node, const std::string &turtleName);

} /* namespace celestial_turtle_spawner */

#endif /* CELESTIAL_TURTLE_SPAWNER__UTILS_HPP_ */