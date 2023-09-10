
#include "celestial_turtle_spawner/utils.hpp"
namespace celestial_turtle_spawner
{
    void offTrailLine(rclcpp::Node* node, const std::string &turtleName)
    {
        auto client = node->create_client<turtlesim::srv::SetPen>("/" + turtleName + "/set_pen");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
        }
        auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
        request->off = 1;

        auto result = client->async_send_request(request);
    }
}