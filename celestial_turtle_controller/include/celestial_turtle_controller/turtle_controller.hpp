#ifndef _TURTLE_CONTROLLER_HPP_
#define _TURTLE_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "std_msgs/msg/bool.hpp"
#include "celestial_turtle_interface/msg/turtles.hpp"
#include "celestial_turtle_interface/msg/turtle.hpp"
#include "turtlesim/msg/pose.hpp"
#include "celestial_turtle_lib/utils.hpp"
#include "celestial_turtle_interface/srv/kill_turtle.hpp"

namespace celestial_turtle_controller
{
    /**
     * @class TurtleController
     * @brief Controls the celestial turtle hunter and its interaction with turtles.
     */
    class TurtleController : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructor for the TurtleController class.
         */
        TurtleController();

    private:
        /**
         * @brief Kills a specified turtle.
         * @param turtleName The name of the turtle to kill.
         */
        void killTurtle(const std::string &turtleName);

        /**
         * @brief Callback function to receive the hunter's pose.
         * @param pose A message containing the hunter's pose.
         */
        void hunterPoseCallback(const turtlesim::msg::Pose::SharedPtr pose);

        /**
         * @brief Process the pose of a turtle and perform actions.
         * @param turtle The turtle's information.
         * @param pose_msg A message containing the turtle's pose.
         */
        void processTurtlePose(const celestial_turtle_interface::msg::Turtle &turtle, const turtlesim::msg::Pose::SharedPtr pose_msg);

        /**
         * @brief Callback function to receive information about alive turtles.
         * @param alive_turtles A list of alive turtles.
         */
        void callbackAliveTurtles(const celestial_turtle_interface::msg::Turtles::SharedPtr alive_turtles);

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_shutdownRequest;                              /**< Publisher for stopping nodes. */
        std::vector<celestial_turtle_interface::msg::Turtle> m_aliveTurtles;                              /**< List of alive turtles. */
        std::vector<rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr> m_turtlePoseSubscribers;       /**< Subscribers for turtle poses. */
        std::vector<std::thread> m_killThreads;                                                           /**< Threads for killing turtles. */
        turtlesim::msg::Pose::SharedPtr m_hunterPose;                                                     /**< Pointer to the hunter's pose. */
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_hunterPoseSubscriber;                     /**< Subscriber for the hunter's pose. */
        rclcpp::Subscription<celestial_turtle_interface::msg::Turtles>::SharedPtr m_aliveTurleSubscriber; /**< Subscriber for alive turtles information. */
    };

} /* namespace celestial_turtle_controller */
#endif /* _TURTLE_CONTROLLER_HPP_ */
