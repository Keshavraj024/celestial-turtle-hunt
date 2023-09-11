#include "celestial_turtle_teleop/turtle_teleop.hpp"

TurtleTeleop::TurtleTeleop() : Node("turtle_teleop_node")
{
    this->declare_parameter("linear_velocity", 1.0);
    m_linearVel = this->get_parameter("linear_velocity").as_double();
    m_cmdVelPublisher = this->create_publisher<geometry_msgs::msg::Twist>("/TurtleGuardian/cmd_vel", 10);
    m_turtlePoseSubscriber = this->create_subscription<turtlesim::msg::Pose>("/TurtleGuardian/pose", 10,
                                                                             std::bind(&TurtleTeleop::poseCallback, this, std::placeholders::_1));
    m_moveThread = std::thread(std::bind(&TurtleTeleop::moveTurtle, this));
}

void TurtleTeleop::poseCallback(const turtlesim::msg::Pose::SharedPtr poseMsg)
{
    m_pose = poseMsg;
}

void TurtleTeleop::moveTurtle()
{
    struct termios oldt, newt;
    char ch[3]; 

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
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
