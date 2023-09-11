#ifndef _CELESTIAL_TURTLE_HUNTER_H_
#define _CELESTIAL_TURTLE_HUNTER_H_

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "std_msgs/msg/bool.hpp"
#include "celestial_turtle_lib/utils.hpp"

namespace celestial_turtle_hunter
{
    /**
     * @class TurtleHunter
     * @brief Controls the turtle hunter in the celestial turtle environment.
     */
    class TurtleHunter : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructor for the TurtleHunter class.
         */
        TurtleHunter();

    private:
        /**
         * @brief Spawns a turtle in the environment.
         */
        void spawnTurle();

        /**
         * @brief Callback function to receive shutdown requests.
         * @param shouldTerminate A boolean message indicating whether to terminate.
         */
        void shutdownRequestCallback(const std_msgs::msg::Bool::SharedPtr shouldTerminate);

        std::thread m_killThread;                                                         /**< Thread for killing turtles. */
        std::thread m_spawnThread;                                                        /**< Thread for spawning turtles. */
        double m_spawnPositionX;                                                          /**< X-coordinate for turtle spawning position. */
        double m_spawnPositionY;                                                          /**< Y-coordinate for turtle spawning position. */
        double m_spawnOrientation;                                                        /**< Orientation for turtle spawning. */
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_shutdownRequestSubscriber; /**< Subscriber for shutdown requests. */
    };
} // namespace celestial_turtle_hunter

#endif
