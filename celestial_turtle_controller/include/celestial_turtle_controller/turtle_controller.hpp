#ifndef _TURTLE_CONTROLLER_HPP_
#define _TURTLE_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "celestial_turtle_interface/msg/turtles.hpp"
#include "celestial_turtle_interface/msg/turtle.hpp"
#include "turtlesim/msg/pose.hpp"
#include "unordered_map"
#include "celestial_turtle_lib/utils.hpp"

namespace celestial_turtle_controller
{
    class TurtleController : public rclcpp::Node
    {
    public:
        TurtleController();

    private:
        std::vector<celestial_turtle_interface::msg::Turtle> m_aliveTurtles;
        std::vector<celestial_turtle_interface::msg::Turtle> m_prevAliveTurtles;
        std::vector<rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr> m_poseSubscribers;
        celestial_turtle_interface::msg::Turtles stillAliveTurtle;
        rclcpp::Publisher<celestial_turtle_interface::msg::Turtles>::SharedPtr m_aliveTurtlePublisher;
        void guardianTurtlePoseCallback(const turtlesim::msg::Pose::SharedPtr pose);
        turtlesim::msg::Pose::SharedPtr m_guardianTurtlePose;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_guardianTurtlePoseSubscriber;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_poseSubscriber;
        void processTurtlePose(const celestial_turtle_interface::msg::Turtle &turtle, const turtlesim::msg::Pose::SharedPtr pose_msg);
        std::unordered_map<std::string, rclcpp::Subscription<turtlesim::msg::Pose>> m_aliveTurtleSubscribers;
        void callbackAliveTurtles(const celestial_turtle_interface::msg::Turtles::SharedPtr alive_turtles);
        rclcpp::Subscription<celestial_turtle_interface::msg::Turtles>::SharedPtr m_aliveTurleSubscriber; 
    };

} /* namespace celestial_turtle_controller */
#endif /* _TURTLE_CONTROLLER_HPP_ */