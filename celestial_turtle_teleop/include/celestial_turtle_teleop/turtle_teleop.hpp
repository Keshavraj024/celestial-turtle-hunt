#ifndef _TURTLE_TELEOP_H_
#define _TURTLE_TELEOP_H_

#include "rclcpp/rclcpp.hpp"
#include <termios.h>
#include <unistd.h>
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp"

/**
 * @class TurtleTeleop
 * @brief Controls the teleoperation of a turtle.
 */
class TurtleTeleop : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the TurtleTeleop class.
     */
    TurtleTeleop();

private:
    /**
     * @brief Callback for receiving shutdown requests.
     * @param shouldTerminate A boolean message indicating whether to terminate.
     */
    void shutdownRequestCallback(const std_msgs::msg::Bool::SharedPtr shouldTerminate);
    /**
     * @brief Moves the turtle based on user input.
     */
    void moveTurtle();

    /**
     * @brief Callback function to receive turtle pose information.
     * @param poseMsg A message containing the turtle's pose.
     */
    void poseCallback(const turtlesim::msg::Pose::SharedPtr poseMsg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_cmdVelPublisher;        /**< Publisher for velocity commands. */
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr m_turtlePoseSubscriber;     /**< Subscriber for turtle pose information. */
    geometry_msgs::msg::Twist m_vel;                                                  /**< Velocity command message. */
    turtlesim::msg::Pose::SharedPtr m_pose;                                           /**< Pointer to the turtle's pose. */
    std::thread m_moveThread;                                                         /**< Thread for moving the turtle. */
    double m_minX{0.5};                                                               /**< Minimum X-coordinate for turtle movement. */
    double m_maxX{10.0};                                                              /**< Maximum X-coordinate for turtle movement. */
    double m_linearVel{2.0};                                                          /**< Linear velocity for turtle movement. */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_shutdownRequestSubscriber; /**< Shared pointer to subscribe to shutdown requests. */
};

#endif
