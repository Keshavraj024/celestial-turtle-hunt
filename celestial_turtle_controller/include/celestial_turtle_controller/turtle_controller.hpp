#ifndef _TURTLE_CONTROLLER_HPP_
#define _TURTLE_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "std_msgs/msg/bool.hpp"
#include "celestial_turtle_interface/msg/turtles.hpp"
#include "celestial_turtle_interface/msg/turtle.hpp"
#include "turtlesim/msg/pose.hpp"
#include "celestial_turtle_lib/utils.hpp"
#include "celestial_turtle_interface/srv/kill_turtle.hpp"

namespace celestial_turtle_controller
{
    class TurtleController : public rclcpp::Node
    {
    public:
        TurtleController();

    private:
        void killTurtle(const std::string &turtleName);
        void hunterPoseCallback(const turtlesim::msg::Pose::SharedPtr pose);
        void processTurtlePose(const celestial_turtle_interface::msg::Turtle &turtle, const turtlesim::msg::Pose::SharedPtr pose_msg);
        void callbackAliveTurtles(const celestial_turtle_interface::msg::Turtles::SharedPtr alive_turtles);
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_stopNode;
        std::vector<celestial_turtle_interface::msg::Turtle> m_aliveTurtles;
        std::vector<rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr> m_turtlePoseSubscribers;
        std::vector<std::thread> m_killThreads;
        turtlesim::msg::Pose::SharedPtr m_hunterPose;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_hunterPoseSubscriber;
        rclcpp::Subscription<celestial_turtle_interface::msg::Turtles>::SharedPtr m_aliveTurleSubscriber;
    };

} /* namespace celestial_turtle_controller */
#endif /* _TURTLE_CONTROLLER_HPP_ */