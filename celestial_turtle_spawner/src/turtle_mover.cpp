#include "celestial_turtle_spawner/turtle_mover.hpp"

namespace celestial_turtle_spawner
{
    TurtleMover::TurtleMover() : Node("celestial_turtle_mover")
    {
        m_aliveTurleSubscriber = this->create_subscription<celestial_turtle_interface::msg::Turtles>("alive_turtles",
                                                                                                     10, std::bind(&TurtleMover::CallbackAliveTurtles, this, std::placeholders::_1));
        m_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TurtleMover::TimerCallBack, this));
        m_stopNodeSubscriber = this->create_subscription<std_msgs::msg::Bool>("kill_nodes",
                                                                              10, std::bind(&TurtleMover::callbackKillNode, this, std::placeholders::_1));
    }
    void TurtleMover::callbackKillNode(const std_msgs::msg::Bool::SharedPtr shouldTerminate)
    {
        if (shouldTerminate->data)
        {
             RCLCPP_WARN(this->get_logger(), "Shut");
            rclcpp::shutdown();
        }
    }

    void TurtleMover::TimerCallBack()
    {
        if (m_aliveTurtles.empty())
        {
            return;
        }
        for (const auto &turtle : m_aliveTurtles)
        {
            auto cmdVelPublisher = m_cmdVelPublishers[turtle.name];

            if (cmdVelPublisher)
            {
                auto cmdVelocityMessage = geometry_msgs::msg::Twist();
                cmdVelocityMessage.linear.x = 2.0;
                cmdVelPublisher->publish(cmdVelocityMessage);
            }
        }
    }

    void TurtleMover::CallbackAliveTurtles(const celestial_turtle_interface::msg::Turtles::SharedPtr alive_turtles)
    {
        m_aliveTurtles = alive_turtles->turtles;
        std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> updatedPublishers;
        for (const auto &turtle : m_aliveTurtles)
        {
            auto topic = turtle.name + "/cmd_vel";
            auto cmdVelPublisher = m_cmdVelPublishers[turtle.name];

            if (!cmdVelPublisher)
            {
                cmdVelPublisher = this->create_publisher<geometry_msgs::msg::Twist>(topic, 10);
            }

            updatedPublishers[turtle.name] = cmdVelPublisher;
        }
        m_cmdVelPublishers = updatedPublishers;

        // Remove publishers for turtles that are no longer alive
        for (auto it = m_cmdVelPublishers.begin(); it != m_cmdVelPublishers.end();)
        {
            const std::string &publisherName = it->first;
            if (updatedPublishers.find(publisherName) == updatedPublishers.end())
            {
                // Publisher is not in the updated list, indicating an inactive turtle
                RCLCPP_INFO(this->get_logger(), "Removing publisher for turtle %s.", publisherName.c_str());
                it = m_cmdVelPublishers.erase(it); // Remove the publisher from the map
            }
            else
            {
                ++it;
            }
        }
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<celestial_turtle_spawner::TurtleMover>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}