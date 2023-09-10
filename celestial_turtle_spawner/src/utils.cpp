
#include "celestial_turtle_spawner/utils.hpp"

void offTrailLine(const std::string &turtleName)
{
    auto client = this->create_client<turtlesim::srv::SetPen>("/"+turtleName+"/set_pen");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "Waiiting for Spawn service");
    }
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->off = 1;

    auto result = client->async_send_request(request);

}