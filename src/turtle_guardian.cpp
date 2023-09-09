#include "turtle_guardian/turtle_guardian.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "chrono"

TurtleGuardian::TurtleGuardian() : Node("turtle_guardian_node")
{
    m_killThread = std::thread(std::bind(&TurtleGuardian::killTurtle, this, "turtle1"));
    m_spawnThread = std::thread(std::bind(&TurtleGuardian::spawnTurle, this));
}

void TurtleGuardian::killTurtle(const std::string &turlteName)
{
    auto client = this->create_client<turtlesim::srv::Kill>("kill");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "Waiiting for kill service");
    }
    auto request = std::make_shared<turtlesim::srv::Kill::Request>();
    request->name = turlteName;

    auto result = client->async_send_request(request);

}

void TurtleGuardian::spawnTurle()
{
    auto client = this->create_client<turtlesim::srv::Spawn>("spawn");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "Waiiting for Spawn service");
    }
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = 5.544445;
    request->y = 1.0;
    request->theta = M_PI / 2.0;
    request->name = "TurtleGuardian";
    
    auto result = client->async_send_request(request);
    try
    {
        auto response = result.get();

        if (response->name.empty())
        {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Created a turtle with name =  %s", response->name.c_str());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not spawn a turtle!");
    }
}