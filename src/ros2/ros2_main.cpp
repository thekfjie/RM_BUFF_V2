#include <exception>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "buff_node.hpp"

int main(int argc, char** argv) {
    bool rclInitialized = false;
    try {
        rclcpp::init(argc, argv);
        rclInitialized = true;

        auto node = std::make_shared<gutcpp::BuffNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "buff_node fatal error: " << ex.what() << std::endl;
    } catch (...) {
        std::cerr << "buff_node fatal error: unknown exception" << std::endl;
    }

    if (rclInitialized) {
        rclcpp::shutdown();
    }
    return 1;
}
