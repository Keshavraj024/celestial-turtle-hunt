
#include "celestial_turtle_lib/utils.hpp"
namespace celestial_turtle_lib
{
    void offTrailLine(rclcpp::Node *node, const std::string &turtleName)
    {
        auto client = node->create_client<turtlesim::srv::SetPen>("/" + turtleName + "/set_pen");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
        }
        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
        request->off = 1;

        auto result = client->async_send_request(request);
    }

    void killTurtle(rclcpp::Node *node, const std::string &turlteName)
    {
        auto client = node->create_client<turtlesim::srv::Kill>("kill");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(node->get_logger(), "Waiiting for kill service");
        }
        auto request = std::make_shared<turtlesim::srv::Kill::Request>();
        request->name = turlteName;

        try
        {
            client->async_send_request(request);
            RCLCPP_INFO(node->get_logger(), "Killed a turtle with name =  %s", request->name.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node->get_logger(), "Could not Kill the turtle!");
        }
    }
}