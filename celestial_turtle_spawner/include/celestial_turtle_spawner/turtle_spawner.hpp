/**
 * @file celestial_turtle_spawner/turtle_spawner.hpp
 * @brief Spawns a turtle
 */
#ifndef CELESTIAL_TURTLE_SPAWNER__TURTLE_SPAWNER_HPP_
#define CELESTIAL_TURTLE_SPAWNER__TURTLE_SPAWNER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "celestial_turtle_interface/msg/turtles.hpp"
#include "celestial_turtle_interface/msg/turtle.hpp"
#include "turtlesim/srv/spawn.hpp"

namespace celestial_turtle_spawner
{
    class TurtleSpawner : public rclcpp::Node
    {
    public:
        TurtleSpawner();

        void SetMaxTurtlesToSpawn(unsigned int maxTurtles);

    private:
        void TimerCallBack();
        /**
         * @brief Spawns a new turtle at a random pose
         */
        void SpawnTurtle();

        /**
         * Publishes the information of the alive turtles
         */
        void PublishAliveTurtles();

        rclcpp::TimerBase::SharedPtr m_timer;                                                          /**< Calls the spawn turtle service*/
        std::vector<std::thread> m_spawnThreads;                                                       /**< Threads calling spawn turtle service*/
        std::vector<celestial_turtle_interface::msg::Turtle> m_aliveTurtles;                           /**< List of alive turtles*/
        rclcpp::Publisher<celestial_turtle_interface::msg::Turtles>::SharedPtr m_aliveTurtlePublisher; /**< Publishes the alive turtles information*/
        unsigned int m_maxTurtles{3};
        rclcpp::TimerBase::SharedPtr m_aliveTurtlePublishertimer;
        celestial_turtle_interface::msg::Turtles m_aliveTurtle;
    };

} /* namespace celestial_turtle_spawner */
#endif /* CELESTIAL_TURTLE_SPAWNER__TURTLE_SPAWNER_HPP_ */