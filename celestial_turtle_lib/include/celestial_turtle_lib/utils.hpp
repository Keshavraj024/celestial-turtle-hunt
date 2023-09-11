#ifndef _CELESTIAL_TURTLE_LIB__UTILS_HPP_
#define _CELESTIAL_TURTLE_LIB__UTILS_HPP_

#include <random>
#include "chrono"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/kill.hpp"

/**
 * @namespace celestial_turtle_lib
 * @brief Contains utility functions for working with celestial turtles.
 */
namespace celestial_turtle_lib
{
    /**
     * @brief Generates a random number between the given range [min, max].
     * @tparam T The type of number to generate (e.g., int, double).
     * @param min The minimum value of the range.
     * @param max The maximum value of the range.
     * @return A random number between min and max.
     */
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

    /**
     * @brief Turns off the trail line for a specified turtle.
     * @param node A pointer to the ROS2 node.
     * @param turtleName The name of the turtle to turn off the trail line for.
     */
    void offTrailLine(rclcpp::Node *node, const std::string &turtleName);

    /**
     * @brief Kills a specified turtle.
     * @param node A pointer to the ROS2 node.
     * @param turlteName The name of the turtle to kill.
     */
    void killTurtle(rclcpp::Node *node, const std::string &turtleName);

} /* namespace celestial_turtle_lib */

#endif /* CELESTIAL_TURTLE_LIB__UTILS_HPP_ */
