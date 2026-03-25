#ifdef AMENT_CMAKE_FOUND

#include <rclcpp/rclcpp.hpp>
#include "buff_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gutcpp::BuffNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#else

#include <iostream>

int main() {
    std::cerr << "buff_node requires ROS 2. Build with ament_cmake in a ROS 2 workspace." << std::endl;
    return 1;
}

#endif
