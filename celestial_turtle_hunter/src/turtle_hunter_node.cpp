#include "celestial_turtle_hunter/turtle_hunter.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<celestial_turtle_hunter::TurtleHunter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}