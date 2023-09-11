/**
 * @file celestial_turtle_spawner/turtle_spawner.hpp
 * @brief This file defines the TurtleSpawner class, which spawns turtles and manages their lifecycle.
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
    /**
     * @brief The TurtleSpawner class spawns turtles and manages their lifecycle.
     */
    class TurtleSpawner : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructs a TurtleSpawner instance.
         */
        TurtleSpawner();

    private:
        /**
         * @brief Callback to handle turtle termination requests.
         * @param shouldTerminate A boolean message indicating whether to terminate turtles.
         */
        void shutdownRequestCallback(const std_msgs::msg::Bool::SharedPtr shouldTerminate);

        /**
         * @brief Timer callback for periodic turtle spawning.
         */
        void TimerCallBack();

        /**
         * @brief Spawns a new turtle at a random pose.
         */
        void SpawnTurtle();

        /**
         * @brief Publishes the information of the alive turtles.
         */
        void PublishAliveTurtles();

        /**
         * @brief Callback to handle turtle termination service requests.
         * @param request The request to terminate a turtle.
         * @param response The response indicating the result of the termination request.
         */
        void killTurtleCallback(const celestial_turtle_interface::srv::KillTurtle::Request::SharedPtr &request,
                                const celestial_turtle_interface::srv::KillTurtle::Response::SharedPtr &response);

        std::vector<std::thread> m_killThreads; /**< Stores threads responsible for killing turtles. */

        rclcpp::TimerBase::SharedPtr m_timer; /**< Shared pointer to manage a timer. */

        rclcpp::Service<celestial_turtle_interface::srv::KillTurtle>::SharedPtr m_killTurtleServer; /**< Shared pointer to manage a service for killing turtles. */

        std::vector<std::thread> m_spawnThreads; /**< Stores threads responsible for spawning turtles. */

        std::vector<celestial_turtle_interface::msg::Turtle> m_turtleSpawned; /**< Stores information about turtles that have been spawned. */

        rclcpp::Publisher<celestial_turtle_interface::msg::Turtles>::SharedPtr m_aliveTurtlePublisher; /**< Shared pointer to publish information about alive turtles. */

        bool m_playByLevel{false}; /**< Indicates whether the game is set to be played by levels (true) or not (false). */
        
        std::size_t m_maxTurtles{1}; /**< Specifies the maximum number of turtles that can be spawned. */

        celestial_turtle_interface::msg::Turtles m_aliveTurtles; /**< Stores information about the currently alive turtles. */

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_shutdownRequestSubscriber; /**< Shared pointer to subscribe to shutdown requests. */
    };

} /* namespace celestial_turtle_spawner */

#endif /* CELESTIAL_TURTLE_SPAWNER__TURTLE_SPAWNER_HPP_ */
