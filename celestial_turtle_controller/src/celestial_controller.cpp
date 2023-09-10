#include "celestial_turtle_controller/celestial_controller.hpp"

namespace celestial_turtle_controller
{
    TurtleController::TurtleController() : Node("turtle_controller")
    {
        m_aliveTurleSubscriber = this->create_subscription<celestial_turtle_interface::msg::Turtles>("alive_turtles",
                                                                                                     10, std::bind(&TurtleController::callbackAliveTurtles, this, std::placeholders::_1));
    }

    void TurtleController::callbackAliveTurtles(const celestial_turtle_interface::msg::Turtles::SharedPtr alive_turtles)
    {
        auto aliveTurtles = alive_turtles->turtles;
        if (aliveTurtles.empty())
        {
            return;
        }
        turtlesim::msg::Pose poseMsg;
        for (const auto &turtle : aliveTurtles)
        {
            std::string topic_name = "/" + turtle.name + "/pose";

            // Create a subscription for each turtle's pose
            m_poseSubscriber = create_subscription<turtlesim::msg::Pose>(
                topic_name, 10, [this, turtle](const turtlesim::msg::Pose::SharedPtr pose_msg)
                { processTurtlePose(turtle, pose_msg); });
        }
    }

    void TurtleController::processTurtlePose(const celestial_turtle_interface::msg::Turtle &turtle, const turtlesim::msg::Pose::SharedPtr pose_msg)
    {
        RCLCPP_INFO(get_logger(), "Turtle name: %s", turtle.name.c_str());
        RCLCPP_INFO(get_logger(), "Turtle Pose Y: %.2f", pose_msg->y);

        if (pose_msg->y <= 0)
        {
            RCLCPP_INFO(get_logger(), "Turtle %s reached the ground", turtle.name.c_str());
            rclcpp::shutdown();
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
