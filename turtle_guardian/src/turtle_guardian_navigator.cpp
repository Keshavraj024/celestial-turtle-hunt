#include "turtle_guardian/turtle_guardian_navigator.hpp"

TurtleGuardianNavigator::TurtleGuardianNavigator() : Node("turtle_guardian_navigator_node")
{
    this->declare_parameter("linear_velocity", 0.0);
    m_linearVel = this->get_parameter("linear_velocity").as_double();
    m_cmdVelPublisher = this->create_publisher<geometry_msgs::msg::Twist>("/TurtleGuardian/cmd_vel", 10);
    m_turtlePoseSubscriber = this->create_subscription<turtlesim::msg::Pose>("/TurtleGuardian/pose", 10,
                                                                             std::bind(&TurtleGuardianNavigator::poseCallback, this, std::placeholders::_1));
    m_moveThread = std::thread(std::bind(&TurtleGuardianNavigator::moveTurtle, this));
}

void TurtleGuardianNavigator::poseCallback(const turtlesim::msg::Pose::SharedPtr poseMsg)
{
    m_pose = poseMsg;
}

void TurtleGuardianNavigator::moveTurtle()
{
    struct termios oldt, newt;
    char ch[3]; // Buffer to store escape sequence

    // Disable buffering for stdin
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    do
    {
        if (read(STDIN_FILENO, ch, 3) == 3)
        {
            if (ch[0] == '\033' && ch[1] == '[')
            {
                if (ch[2] == 'D')
                {
                    if (m_pose && m_pose->x <= m_minX)
                    {
                        m_vel.linear.y = 0.0;
                    }
                    else
                        m_vel.linear.y = m_linearVel;
                    m_cmdVelPublisher->publish(m_vel);
                }
                else if (ch[2] == 'C')
                {
                    if (m_pose && m_pose->x >= m_maxX)
                    {
                        m_vel.linear.y = 0.0;
                    }
                    else
                        m_vel.linear.y = -m_linearVel;
                    m_cmdVelPublisher->publish(m_vel);
                }
            }
        }
    } while (ch[0] != 'q' && ch[0] != 'Q');

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    rclcpp::shutdown();
    
    // Restore the original terminal settings
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleGuardianNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
