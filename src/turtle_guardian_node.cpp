#include "turtle_guardian/turtle_guardian.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleGuardian>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}