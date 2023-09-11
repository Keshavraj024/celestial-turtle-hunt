#include "celestial_turtle_hunter/turtle_hunter.hpp"
#include "chrono"

TurtleHunter::TurtleHunter() : Node("turtle_hunter_node")
{
    this->declare_parameter("spawn_position_x", 0.0);
    this->declare_parameter("spawn_position_y", 0.0);
    this->declare_parameter("spawn_orientation", 0.0);

    m_spawnPositionX = this->get_parameter("spawn_position_x").as_double();
    m_spawnPositionY = this->get_parameter("spawn_position_y").as_double();
    m_spawnOrientation = this->get_parameter("spawn_orientation").as_double();
    m_spawnThread = std::thread(std::bind(&TurtleHunter::spawnTurle, this));
}


void TurtleHunter::spawnTurle()
{
    celestial_turtle_lib::killTurtle(this,"turtle1");
    auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "Waiiting for Spawn service");
    }
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = m_spawnPositionX;
    request->y = m_spawnPositionY;
    request->theta = m_spawnOrientation;
    request->name = "turtleHunter";

    auto result = client->async_send_request(request);
    try
    {
        auto response = result.get();

        if (response->name.empty())
        {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Created a turtle with name =  %s", response->name.c_str());
        celestial_turtle_lib::offTrailLine(this, response->name);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not spawn a turtle!");
    }
}
