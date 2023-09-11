/**
 * @file celestial_turtle_spawner/turtle_spawner.hpp
 * @brief Defines the TurtleMover class for controlling turtle movement.
 */
#ifndef CELESTIAL_TURTLE_SPAWNER__TURTLE_MOVER_HPP_
#define CELESTIAL_TURTLE_SPAWNER__TURTLE_MOVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "celestial_turtle_interface/msg/turtles.hpp"
#include "celestial_turtle_interface/msg/turtle.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

namespace celestial_turtle_spawner
{
    /**
     * @class TurtleMover
     * @brief Controls turtle movement and interacts with the celestial turtle environment.
     */
    class TurtleMover : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructor for the TurtleMover class.
         */
        TurtleMover();

    private:
        /**
         * @brief Callback for receiving shutdown requests.
         * @param shouldTerminate A boolean message indicating whether to terminate.
         */
        void shutdownRequestCallback(const std_msgs::msg::Bool::SharedPtr shouldTerminate);

        /**
         * @brief Timer callback for controlling turtle movement.
         */
        void TimerCallBack();

        /**
         * @brief Callback function to receive information about alive turtles.
         * @param alive_turtles A list of alive turtles.
         */
        void CallbackAliveTurtles(const celestial_turtle_interface::msg::Turtles::SharedPtr alive_turtles);

        std::vector<celestial_turtle_interface::msg::Turtle> m_aliveTurtles;                               /**< List of alive turtles */
        rclcpp::TimerBase::SharedPtr m_timer;                                                              /**< Timer for sending velocity commands */
        rclcpp::Subscription<celestial_turtle_interface::msg::Turtles>::SharedPtr m_aliveTurleSubscriber;  /**< Subscriber for alive turtles information */
        std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> m_cmdVelPublishers; /**< Publishers for command velocity */
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_shutdownRequestSubscriber;                  /**< Subscriber for shutdown requests */
    };

} /* namespace celestial_turtle_spawner */
#endif /* CELESTIAL_TURTLE_SPAWNER__TURTLE_MOVER_HPP_ */
