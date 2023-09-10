#include "celestial_turtle_spawner/turtle_spawner.hpp"
#include "celestial_turtle_spawner/utils.hpp"

namespace celestial_turtle_spawner
{
    TurtleSpawner::TurtleSpawner() : Node("celestial_turtle_spawner")
    {
        this->declare_parameter("max_turtles", 3);
        m_maxTurtles = this->get_parameter("max_turtles").as_int();
        m_timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&TurtleSpawner::TimerCallBack, this));
        m_aliveTurtlePublisher = this->create_publisher<celestial_turtle_interface::msg::Turtles>("alive_turtles", 10);
    }

    void TurtleSpawner::SetMaxTurtlesToSpawn(unsigned int maxTurtles)
    {
        m_maxTurtles = maxTurtles;
    }

    void TurtleSpawner::TimerCallBack()
    {
        m_spawnThreads.push_back(std::thread(std::bind(&TurtleSpawner::SpawnTurtle, this)));
    }

    void TurtleSpawner::SpawnTurtle()
    {
        if (m_aliveTurtles.size() >= m_maxTurtles)
        {
            return;
        }

        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for spwan service server to be up!!");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = GenerateRandomNumber(0.0, 10.5);
        request->y = 10.5;
        request->theta = -M_PI / 2;

        auto future = client->async_send_request(request);
        try
        {
            auto response = future.get();

            if (response->name.empty())
            {
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Created a turtle with name =  %s", response->name.c_str());
            auto turtle = celestial_turtle_interface::msg::Turtle();
            turtle.x = request->x;
            turtle.y = request->y;
            turtle.theta = request->theta;
            turtle.name = response->name;
            m_aliveTurtles.push_back(turtle);
            PublishAliveTurtles();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not spawn a turtle!");
        }
    }

    void TurtleSpawner::PublishAliveTurtles()
    {
        RCLCPP_ERROR(this->get_logger(), "PublishAliveTurtles");
        auto message = celestial_turtle_interface::msg::Turtles();
        message.turtles = m_aliveTurtles;
        m_aliveTurtlePublisher->publish(message);
    }

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<celestial_turtle_spawner::TurtleSpawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}