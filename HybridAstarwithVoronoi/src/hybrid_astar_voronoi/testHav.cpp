#include "hav_ros_node.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HAVROSNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}