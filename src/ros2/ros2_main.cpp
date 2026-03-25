#include <rclcpp/rclcpp.hpp>
#include "buff_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gutcpp::BuffNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
