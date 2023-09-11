/**
 * @file celestial_turtle_spawner/turtle_spawner.hpp
 * @brief Spawns a turtle
 */
#ifndef CELESTIAL_TURTLE_SPAWNER__TURTLE_SPAWNER_HPP_
#define CELESTIAL_TURTLE_SPAWNER__TURTLE_SPAWNER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "celestial_turtle_interface/msg/turtles.hpp"
#include "celestial_turtle_interface/msg/turtle.hpp"
#include "celestial_turtle_interface/srv/kill_turtle.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "celestial_turtle_lib/utils.hpp"
#include "std_msgs/msg/bool.hpp"

namespace celestial_turtle_spawner
{
    class TurtleSpawner : public rclcpp::Node
    {
    public:
        TurtleSpawner();

    private:
        void callbackKillNode(const std_msgs::msg::Bool::SharedPtr shouldTerminate);
        void TimerCallBack();
        /**
         * @brief Spawns a new turtle at a random pose
         */
        void SpawnTurtle();

        /**
         * Publishes the information of the alive turtles
         */
        void PublishAliveTurtles();

        void killTurtleCallback(const celestial_turtle_interface::srv::KillTurtle::Request::SharedPtr &request,
                                 const celestial_turtle_interface::srv::KillTurtle::Response::SharedPtr &response);

        std::vector<std::thread> m_killThreads;
        rclcpp::TimerBase::SharedPtr m_timer;  
        rclcpp::Service<celestial_turtle_interface::srv::KillTurtle>::SharedPtr m_killTurtleServer;                                                        /**< Calls the spawn turtle service*/
        std::vector<std::thread> m_spawnThreads;                                                       /**< Threads calling spawn turtle service*/
        std::vector<celestial_turtle_interface::msg::Turtle> m_aliveTurtles;                           /**< List of alive turtles*/
        rclcpp::Publisher<celestial_turtle_interface::msg::Turtles>::SharedPtr m_aliveTurtlePublisher; /**< Publishes the alive turtles information*/
        std::size_t m_maxTurtles{1};
        celestial_turtle_interface::msg::Turtles m_aliveTurtle;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_stopNodeSubscriber;
    };

} /* namespace celestial_turtle_spawner */
#endif /* CELESTIAL_TURTLE_SPAWNER__TURTLE_SPAWNER_HPP_ */