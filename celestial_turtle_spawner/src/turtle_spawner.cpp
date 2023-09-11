#include "celestial_turtle_spawner/turtle_spawner.hpp"

namespace celestial_turtle_spawner
{
    TurtleSpawner::TurtleSpawner() : Node("celestial_turtle_spawner")
    {
        this->declare_parameter("max_turtles", 1);
        m_maxTurtles = this->get_parameter("max_turtles").as_int();
        m_timer = this->create_wall_timer(std::chrono::seconds(3), std::bind(&TurtleSpawner::TimerCallBack, this));
        m_aliveTurtlePublisher = this->create_publisher<celestial_turtle_interface::msg::Turtles>("alive_turtles", 10);
        m_killTurtleServer = this->create_service<celestial_turtle_interface::srv::KillTurtle>("kill_turtle",
                                                                                               std::bind(&TurtleSpawner::killTurtleCallback, this, std::placeholders::_1, std::placeholders::_2));
        m_shutdownRequestSubscriber = this->create_subscription<std_msgs::msg::Bool>("kill_nodes",
                                                                              10, std::bind(&TurtleSpawner::shutdownRequestCallback, this, std::placeholders::_1));
    }
    void TurtleSpawner::shutdownRequestCallback(const std_msgs::msg::Bool::SharedPtr shouldTerminate)
    {
        if (shouldTerminate->data)
        {
            rclcpp::shutdown();
        }
    }

    void TurtleSpawner::killTurtleCallback(const celestial_turtle_interface::srv::KillTurtle::Request::SharedPtr &request,
                                           const celestial_turtle_interface::srv::KillTurtle::Response::SharedPtr &response)
    {
        if (request->name.empty())
        {
            response->success = false;
        }
        m_killThreads.push_back(std::thread(celestial_turtle_lib::killTurtle, this, request->name));
        auto it = std::find_if(m_turtleSpawned.begin(), m_turtleSpawned.end(), [&request](const celestial_turtle_interface::msg::Turtle &turtle)
                               { return turtle.name == request->name; });

        if (it != m_turtleSpawned.end())
        {
            m_turtleSpawned.erase(it);
            PublishAliveTurtles();
            response->success = true;
        }
        else
        {
            response->success = false;
        }
    }

    void TurtleSpawner::TimerCallBack()
    {
        m_spawnThreads.push_back(std::thread(std::bind(&TurtleSpawner::SpawnTurtle, this)));
    }

    void TurtleSpawner::SpawnTurtle()
    {
        if (m_turtleSpawned.size() >= m_maxTurtles)
        {
            return;
        }

        auto client = this->create_client<turtlesim::srv::Spawn>("spawn");

        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for spwan service server to be up!!");
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = celestial_turtle_lib::GenerateRandomNumber(0.0, 10.5);
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
            m_turtleSpawned.push_back(turtle);
            celestial_turtle_lib::offTrailLine(this, turtle.name);
            PublishAliveTurtles();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not spawn a turtle!");
        }
    }

    void TurtleSpawner::PublishAliveTurtles()
    {
        m_aliveTurtles.turtles = m_turtleSpawned;
        m_aliveTurtlePublisher->publish(m_aliveTurtles);
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