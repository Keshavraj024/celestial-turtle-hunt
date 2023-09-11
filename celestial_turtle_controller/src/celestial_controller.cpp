#include "celestial_turtle_controller/celestial_controller.hpp"
#include "turtlesim/srv/kill.hpp"

namespace celestial_turtle_controller
{
    TurtleController::TurtleController() : Node("turtle_controller")
    {
        m_aliveTurtlePublisher = this->create_publisher<celestial_turtle_interface::msg::Turtles>("alive_turtles", 10);
        m_aliveTurleSubscriber = this->create_subscription<celestial_turtle_interface::msg::Turtles>("alive_turtles",
                                                                                                     10, std::bind(&TurtleController::callbackAliveTurtles, this, std::placeholders::_1));

        m_guardianTurtlePoseSubscriber = this->create_subscription<turtlesim::msg::Pose>("/TurtleGuardian/pose",
                                                                                         10, std::bind(&TurtleController::guardianTurtlePoseCallback, this, std::placeholders::_1));
    }

    void TurtleController::guardianTurtlePoseCallback(const turtlesim::msg::Pose::SharedPtr pose)
    {
        m_guardianTurtlePose = pose;
    }

    void TurtleController::callbackAliveTurtles(const celestial_turtle_interface::msg::Turtles::SharedPtr alive_turtles)
    {
        m_aliveTurtles = alive_turtles->turtles;
        if (m_aliveTurtles.empty() || m_aliveTurtles == m_prevAliveTurtles)
        {
            return;
        }
        std::vector<rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr> poseSubscribers;
        for (const auto &turtle : m_aliveTurtles)
        {
            std::string topic_name = "/" + turtle.name + "/pose";

            // Create a subscription for each turtle's pose
            auto poseSubscriber = create_subscription<turtlesim::msg::Pose>(
                topic_name, 10, [this, turtle](const turtlesim::msg::Pose::SharedPtr pose_msg)
                { processTurtlePose(turtle, pose_msg); });
            poseSubscribers.push_back(poseSubscriber);
        }
        m_poseSubscribers = poseSubscribers;
        m_prevAliveTurtles = m_aliveTurtles;
    }

    void TurtleController::processTurtlePose(const celestial_turtle_interface::msg::Turtle &turtle, const turtlesim::msg::Pose::SharedPtr pose_msg)
    {
        // RCLCPP_INFO(get_logger(), "Turtle name: %s", turtle.name.c_str());
        // RCLCPP_INFO(get_logger(), "Turtle Pose Y: %.2f", std::abs(pose_msg->y - m_guardianTurtlePose->y));
        // RCLCPP_INFO(get_logger(), "Turtle Pose X: %.2f", std::abs(pose_msg->x - m_guardianTurtlePose->x));

        if (pose_msg->y <= 0)
        {
            RCLCPP_INFO(get_logger(), "Turtle %s reached the ground. Terminating the Game", turtle.name.c_str());
            rclcpp::shutdown();
        }
        else if (std::abs(pose_msg->y - m_guardianTurtlePose->y) <= 1.0 && std::abs(pose_msg->x - m_guardianTurtlePose->x) <= 1.0)
        {
            celestial_turtle_lib::killTurtle(this, turtle.name);
            auto turtleToRemove = std::remove_if(m_aliveTurtles.begin(), m_aliveTurtles.end(), [turtle](const auto &turtleElement)
                                                 { return turtleElement.name == turtle.name; });
            m_aliveTurtles.erase(turtleToRemove);
            stillAliveTurtle.turtles = m_aliveTurtles;
            m_aliveTurtlePublisher->publish(stillAliveTurtle);
        }
    }

}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<celestial_turtle_controller::TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
