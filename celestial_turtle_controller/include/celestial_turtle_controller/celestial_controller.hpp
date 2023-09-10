/**
 * @file celestial_turtle_spawner/turtle_spawner.hpp
 * @brief Spawns a turtle
 */
#ifndef CELESTIAL_CONTROLLER_HPP_
#define CELESTIAL_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "celestial_turtle_interface/msg/turtles.hpp"
#include "celestial_turtle_interface/msg/turtle.hpp"
#include "turtlesim/msg/pose.hpp"
#include "unordered_map"

namespace celestial_turtle_controller
{
    class TurtleController : public rclcpp::Node
    {
    public:
        TurtleController();

    private:
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_poseSubscriber;
        void processTurtlePose(const celestial_turtle_interface::msg::Turtle &turtle, const turtlesim::msg::Pose::SharedPtr pose_msg);
        std::unordered_map<std::string, rclcpp::Subscription<turtlesim::msg::Pose>> m_aliveTurtleSubscribers;
        void callbackAliveTurtles(const celestial_turtle_interface::msg::Turtles::SharedPtr alive_turtles);
        rclcpp::Subscription<celestial_turtle_interface::msg::Turtles>::SharedPtr m_aliveTurleSubscriber; /**< Subscribes to the alive turtles information*/
    };

} /* namespace celestial_turtle_controller */
#endif /* CELESTIAL_CONTROLLER_HPP_ */