/**
 * @file celestial_turtle_spawner/turtle_spawner.hpp
 * @brief Spawns a turtle
 */
#ifndef CELESTIAL_TURTLE_SPAWNER__TURTLE_MOVER_HPP_
#define CELESTIAL_TURTLE_SPAWNER__TURTLE_MOVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "celestial_turtle_interface/msg/turtles.hpp"
#include "celestial_turtle_interface/msg/turtle.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace celestial_turtle_spawner
{
    class TurtleMover : public rclcpp::Node
    {
    public:
        TurtleMover();

    private:
        void TimerCallBack();

        /**
         * @brief Callback function to get the information about the alive turtles
         * @param alive_turtles list of alive turtles
         */
        void CallbackAliveTurtles(const celestial_turtle_interface::msg::Turtles::SharedPtr alive_turtles);

        std::vector<celestial_turtle_interface::msg::Turtle> m_aliveTurtles;                               /**< List of alive turtles*/
        rclcpp::TimerBase::SharedPtr m_timer;                                                              /**< Send the velocity command*/
        rclcpp::Subscription<celestial_turtle_interface::msg::Turtles>::SharedPtr m_aliveTurleSubscriber;  /**< Subscribes to the alive turtles information*/
        std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> m_cmdVelPublishers; /**< Publishes the command veloctiy */
    };

} /* namespace celestial_turtle_spawner */
#endif /* CELESTIAL_TURTLE_SPAWNER__TURTLE_MOVER_HPP_ */