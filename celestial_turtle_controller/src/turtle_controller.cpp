#include "celestial_turtle_controller/turtle_controller.hpp"
#include "turtlesim/srv/kill.hpp"

namespace celestial_turtle_controller
{
    TurtleController::TurtleController() : Node("turtle_controller_node")
    {
        m_shutdownRequest = this->create_publisher<std_msgs::msg::Bool>("kill_nodes", 10);
        m_aliveTurleSubscriber = this->create_subscription<celestial_turtle_interface::msg::Turtles>("alive_turtles",
                                                                                                     10, std::bind(&TurtleController::callbackAliveTurtles, this, std::placeholders::_1));

        m_hunterPoseSubscriber = this->create_subscription<turtlesim::msg::Pose>("/turtleHunter/pose",
                                                                                 10, std::bind(&TurtleController::hunterPoseCallback, this, std::placeholders::_1));
    }

    void TurtleController::hunterPoseCallback(const turtlesim::msg::Pose::SharedPtr pose)
    {
        m_hunterPose = pose;
    }

    void TurtleController::callbackAliveTurtles(const celestial_turtle_interface::msg::Turtles::SharedPtr alive_turtles)
    {
        m_aliveTurtles = alive_turtles->turtles;
        if (m_aliveTurtles.empty())
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
        m_turtlePoseSubscribers = poseSubscribers;
    }

    void TurtleController::processTurtlePose(const celestial_turtle_interface::msg::Turtle &turtle, const turtlesim::msg::Pose::SharedPtr pose_msg)
    {
        // RCLCPP_INFO(get_logger(), "Turtle name: %s", turtle.name.c_str());
        // RCLCPP_INFO(get_logger(), "Turtle Pose Y: %.2f", std::abs(pose_msg->y - m_hunterPose->y));
        // RCLCPP_INFO(get_logger(), "Turtle Pose X: %.2f", std::abs(pose_msg->x - m_hunterPose->x));

        if (pose_msg->y <= 0)
        {
            RCLCPP_INFO(get_logger(), "%s reached the ground. Terminating the Game", turtle.name.c_str());
            std_msgs::msg::Bool killMsg;
            killMsg.data = true;
            m_shutdownRequest->publish(killMsg);
            rclcpp::shutdown();
        }
        else if (std::abs(pose_msg->y - m_hunterPose->y) <= 1.0 && std::abs(pose_msg->x - m_hunterPose->x) <= 1.0)
        {
            auto client = this->create_client<celestial_turtle_interface::srv::KillTurtle>("kill_turtle");
            m_killThreads.push_back(std::thread(std::bind(&TurtleController::killTurtle, this, turtle.name)));
        }
    }

    void TurtleController::killTurtle(const std::string &turtleName)
    {
        auto client = this->create_client<celestial_turtle_interface::srv::KillTurtle>("kill_turtle");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for service server to be up!!");
        }

        auto request = std::make_shared<celestial_turtle_interface::srv::KillTurtle::Request>();
        request->name = turtleName;

        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Killed a turtle with name =  %s", turtleName.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to Kill a turtle with name =  %s", turtleName.c_str());
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed!");
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
